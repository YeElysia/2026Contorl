#include <Arduino.h>
#include <OneButton.h>
#include <string.h>

#include "ChassisControl.h"
#include "ChassisMotionPort.h"
#include "DebugLog.h"
#include "GraspMotionPorts.h"
#include "GraspVisionPorts.h"
#include "MaixCamV2.h"
#include "MaixProRingAlignment.h"
#include "MaixVisionService.h"
#include "MechanismTaskExecutor.h"
#include "MissionRoutes.h"
#include "chassis_config.h"
#include "debug_config.h"
#include "mechanism_config.h"
#include "mission_config.h"
#include "vision_config.h"

#ifndef FIRST_LAYER_PLACEMENT_TEST
#error "This test requires FIRST_LAYER_PLACEMENT_TEST in the selected environment."
#endif

namespace
{
// 本测试没有抓取任务；意外请求必须拒绝，不能假报执行成功。
class IdleGraspVision : public IGraspVisionProvider
{
public:
    void begin() override {}
    bool startTracking(uint8_t) override { return false; }
    void update() override {}
    bool takeObservation(GraspObservation &) override { return false; }
    void stop() override {}
    bool faulted() const override { return false; }
};

class IdleForwardPositioner : public IGraspForwardPositioner
{
public:
    bool moveBodyRelative(float, float) override { return false; }
    bool busy() const override { return false; }
    bool faulted() const override { return false; }
    void stop() override {}
    void release() override {}
};

enum class TestState : uint8_t
{
    Initializing, Ready, Preparing, Aligning, Placing, Passed, Failed, Cancelled
};

HardwareSerial serialImu(chassis_config::IMU_RX_PIN, chassis_config::IMU_TX_PIN);
HardwareSerial serialVision(vision_config::RX_PIN, vision_config::TX_PIN);
HardwareSerial serialMechanismStepper(mechanism_config::STEPPER_RX_PIN, mechanism_config::STEPPER_TX_PIN);
HardwareSerial serialMechanismBase(mechanism_config::BASE_RX_PIN, mechanism_config::BASE_TX_PIN);
HardwareSerial serialMechanismServo(mechanism_config::SERVO_RX_PIN, mechanism_config::SERVO_TX_PIN);
HardwareSerial serialDebug(debug_config::RX_PIN, debug_config::TX_PIN);

ChassisControl chassis(&serialImu);
ChassisMotionPort alignmentChassis(chassis, ChassisOwner::Alignment);
maixcam::MaixCamV2 camera(serialVision);
maixcam::MaixVisionService vision(camera);
MaixProRingAlignment alignment(vision, alignmentChassis);
IdleGraspVision idleGraspVision;
IdleForwardPositioner idleForwardPositioner;
MechanismTaskExecutor mechanism(
    serialMechanismStepper, serialMechanismBase, serialMechanismServo,
    idleGraspVision, idleForwardPositioner);
OneButton startButton(mission_config::START_BUTTON_PIN, true, true);

BatchMission batch = {{1, 2, 3}, {1, 2, 3}, {1, 2, 3}};
uint8_t loadedSlotMask = 7;
TestState state = TestState::Initializing;
bool resetRequired = false;
uint32_t lastReportMs = 0;
char commandLine[80] = {};
size_t commandLength = 0;
bool discardLine = false;

const char *stateName()
{
    switch (state)
    {
    case TestState::Initializing: return "INIT";
    case TestState::Ready: return "READY";
    case TestState::Preparing: return "PREPARING";
    case TestState::Aligning: return "ALIGNING";
    case TestState::Placing: return "PLACING";
    case TestState::Passed: return "PASS";
    case TestState::Failed: return "FAILED";
    case TestState::Cancelled: return "CANCELLED";
    }
    return "UNKNOWN";
}

bool running()
{
    return state == TestState::Preparing || state == TestState::Aligning ||
           state == TestState::Placing;
}

void setState(TestState next)
{
    state = next;
    serialDebug.print("STATE ");
    serialDebug.println(stateName());
}

void printRetryWarning()
{
    serialDebug.println(
        "RETRY: remove already placed materials from the rings, check the gripper, "
        "reload the declared tray slots and manually reposition at STORAGE_ANCHOR yaw=-180. "
        "No automatic unloading, replenishment, repositioning or restart.");
}

void stopTest()
{
    // cancel释放对准底盘所有权及其视觉token，无需独立操作共享相机。
    alignment.cancel();
    mechanism.cancel();
    chassis.stop();
}

void failTest(const char *reason)
{
    if (state == TestState::Failed)
        return;
    resetRequired = true;
    setState(TestState::Failed);
    serialDebug.print("FAILED reason=");
    serialDebug.println(reason);
    stopTest();
    serialDebug.println("RESET REQUIRED: fix the fault and reset the controller; run locked, no automatic fault clearing.");
    printRetryWarning();
}

void updateSequence()
{
    if (state == TestState::Failed)
        return;
    // 全局检查，包含初始化和非运行阶段；cancel后不重复报告原失败。
    if (chassis.state() == ChassisControl::State::Fault)
    {
        failTest(chassis.faultMessage());
        return;
    }
    if (mechanism.result() == AsyncResult::Failed)
    {
        failTest(mechanism.faultMessage());
        return;
    }
    if (alignment.result() == AsyncResult::Failed)
    {
        failTest("storage alignment failed");
        return;
    }
    if (resetRequired)
        return;

    switch (state)
    {
    case TestState::Initializing:
        if (mechanism.ready() && mechanism.result() != AsyncResult::Running)
        {
            setState(TestState::Ready);
            serialDebug.println("Initialization complete; run still requires IMU ready and chassis idle.");
        }
        break;
    case TestState::Preparing:
        if (mechanism.result() == AsyncResult::Succeeded)
        {
            if (!alignment.start({Station::Storage, 0, 0}))
                failTest("storage alignment start rejected");
            else
                setState(TestState::Aligning);
        }
        else if (mechanism.result() != AsyncResult::Running)
            failTest("preparation ended without Succeeded");
        break;
    case TestState::Aligning:
        if (alignment.result() == AsyncResult::Succeeded && !chassis.busy())
        {
            if (!mechanism.start(StationTask::StoreFinishedProduct, 0, batch))
                failTest("first-layer placement start rejected");
            else
                setState(TestState::Placing);
        }
        else if (alignment.result() != AsyncResult::Running &&
                 alignment.result() != AsyncResult::Succeeded)
            failTest("alignment ended without Succeeded");
        break;
    case TestState::Placing:
        if (mechanism.result() == AsyncResult::Succeeded)
        {
            setState(TestState::Passed);
            serialDebug.println("PASS: first-layer command sequence completed ONLY; manually confirm physical placement in each selected ring.");
            printRetryWarning();
        }
        else if (mechanism.result() != AsyncResult::Running)
            failTest("placement ended without Succeeded");
        break;
    default:
        break;
    }
}

bool idleForCommand()
{
    return !resetRequired && state != TestState::Initializing && !running() &&
           mechanism.ready() && mechanism.result() != AsyncResult::Running &&
           alignment.result() != AsyncResult::Running && !chassis.busy() &&
           chassis.state() != ChassisControl::State::Fault;
}

void requestRun()
{
    updateSequence();
    if (!idleForCommand() || !chassis.imuReady())
    {
        serialDebug.println("REJECT run: initialization incomplete, busy, IMU not ready or reset required; discarded, never queued.");
        return;
    }
    serialDebug.println("RUN: operator declares tray loaded and chassis manually at STORAGE_ANCHOR yaw=-180; alignment WILL micro-adjust chassis.");
    const auto &anchor = mission_routes::STORAGE_ANCHOR;
    if (!chassis.resetWorldPose(anchor.position.xMm, anchor.position.yMm, anchor.yawDeg))
    {
        failTest("STORAGE_ANCHOR pose reset rejected");
        return;
    }
    if (!mechanism.prepareFirstLayerTest(loadedSlotMask))
    {
        failTest("preloaded slot declaration rejected");
        return;
    }
    if (!mechanism.prepareForTravel(TravelDestination::Storage))
    {
        failTest("storage preparation rejected");
        return;
    }
    setState(TestState::Preparing);
}

void requestCancel()
{
    updateSequence();
    if (state == TestState::Failed)
    {
        serialDebug.println("REJECT cancel: FAILED remains latched; reset required.");
        return;
    }
    if (state == TestState::Cancelled)
        return;
    const bool initializationInterrupted = state == TestState::Initializing || !mechanism.ready();
    stopTest();
    resetRequired = resetRequired || initializationInterrupted;
    setState(TestState::Cancelled);
    printRetryWarning();
    if (resetRequired)
        serialDebug.println("RESET REQUIRED: initialization cancelled; no automatic reinitialization.");
}

void printStatus()
{
    const auto &motion = mechanism.motionDebug();
    const auto &ring = alignment.debugState();
    serialDebug.print("FIRST_LAYER t=");
    serialDebug.print(millis());
    serialDebug.print(" state=");
    serialDebug.print(stateName());
    serialDebug.print(" phase=");
    serialDebug.print(mechanism.debugPhase());
    serialDebug.print(" motion_target_lift_base_extension=");
    serialDebug.print(motion.liftTarget, 1);
    serialDebug.print(',');
    serialDebug.print(motion.baseTarget, 1);
    serialDebug.print(',');
    serialDebug.print(motion.extensionTarget, 1);
    serialDebug.print(" declared_slot_mask=");
    serialDebug.print(loadedSlotMask);
    serialDebug.print(" slot1_2_3_target_rings=");
    for (uint8_t i = 0; i < MATERIALS_PER_BATCH; ++i)
    {
        if (i != 0) serialDebug.print(',');
        serialDebug.print(batch.storagePositions[i]);
    }
    serialDebug.print(" alignment_result=");
    serialDebug.print(static_cast<uint8_t>(alignment.result()));
    serialDebug.print(" observation=");
    serialDebug.print(ring.hasObservation ? 1 : 0);
    serialDebug.print(" found=");
    serialDebug.print(ring.found ? 1 : 0);
    serialDebug.print(" dx_dy=");
    serialDebug.print(ring.dx);
    serialDebug.print(',');
    serialDebug.print(ring.dy);
    serialDebug.print(" error_dx_dy=");
    serialDebug.print(ring.dx - vision_config::RING_TARGET_DX_PX);
    serialDebug.print(',');
    serialDebug.print(ring.dy - vision_config::RING_TARGET_DY_PX);
    serialDebug.print(" quality=");
    serialDebug.print(ring.quality);
    serialDebug.print(" stable=");
    serialDebug.print(ring.stableFrames);
    serialDebug.print(" move_pending=");
    serialDebug.print(ring.movePending ? 1 : 0);
    serialDebug.print(" mechanism_ready=");
    serialDebug.print(mechanism.ready() ? 1 : 0);
    serialDebug.print(" imu_ready=");
    serialDebug.print(chassis.imuReady() ? 1 : 0);
    serialDebug.print(" chassis_busy=");
    serialDebug.print(chassis.busy() ? 1 : 0);
    serialDebug.print(" reset_required=");
    serialDebug.println(resetRequired ? 1 : 0);
}

void printHelp()
{
    serialDebug.println("Commands: setplan <r1> <r2> <r3> | load <mask> | run | cancel | status | help");
    serialDebug.println("setplan: three distinct single digits 1..3, maps tray slots 1..3 to storage rings; configuration idle-only.");
    serialDebug.println("load: single digit 1..7, declaration ONLY, no material detection. Bits 0..2 = tray slots 1..3; 1/2/4 = single item, 7 = all three.");
    serialDebug.println("PB9 click = run, never queued. Round=0 first layer only. Async result: 0=Idle 1=Running 2=Succeeded 3=Failed.");
    serialDebug.println("Targets are commands, not physical confirmations; per-item progress uses mechanism internal LOG, not an invented item field.");
}

bool singleDigit(const char *token, char maximum)
{
    return token[0] >= '1' && token[0] <= maximum && token[1] == '\0';
}

void processCommand()
{
    char *tokens[4] = {};
    size_t count = 0;
    char *cursor = commandLine;
    while (*cursor != '\0')
    {
        while (*cursor == ' ' || *cursor == '\t') ++cursor;
        if (*cursor == '\0') break;
        if (count == 4)
        {
            serialDebug.println("REJECT command: too many arguments.");
            return;
        }
        tokens[count++] = cursor;
        while (*cursor != '\0' && *cursor != ' ' && *cursor != '\t') ++cursor;
        if (*cursor != '\0') *cursor++ = '\0';
    }
    if (count == 0) return;
    if (count == 1 && strcmp(tokens[0], "run") == 0) requestRun();
    else if (count == 1 && strcmp(tokens[0], "cancel") == 0) requestCancel();
    else if (count == 1 && strcmp(tokens[0], "status") == 0) printStatus();
    else if (count == 1 && strcmp(tokens[0], "help") == 0) printHelp();
    else if ((count == 4 && strcmp(tokens[0], "setplan") == 0) ||
             (count == 2 && strcmp(tokens[0], "load") == 0))
    {
        const bool plan = count == 4;
        uint8_t values[3] = {};
        uint8_t ringMask = 0;
        for (size_t i = 1; i < count; ++i)
        {
            if (!singleDigit(tokens[i], plan ? '3' : '7'))
            {
                serialDebug.println("REJECT config: setplan requires single digits 1..3; load requires a single digit 1..7.");
                return;
            }
            values[i - 1] = static_cast<uint8_t>(tokens[i][0] - '0');
            const uint8_t bit = static_cast<uint8_t>(1U << values[i - 1]);
            if (plan && (ringMask & bit) != 0)
            {
                serialDebug.println("REJECT setplan: target rings must be distinct; no configuration changed.");
                return;
            }
            ringMask |= bit;
        }
        updateSequence();
        if (!idleForCommand())
        {
            serialDebug.println("REJECT config: initialized and idle only; reset-required states locked.");
            return;
        }
        if (plan) memcpy(batch.storagePositions, values, sizeof(values));
        else loadedSlotMask = values[0];
        serialDebug.println("OK config (declaration only; no hardware movement)");
        printStatus();
    }
    else serialDebug.println("REJECT command: invalid syntax or trailing arguments; use help.");
}

void updateSerial()
{
    for (uint8_t budget = 0; budget < 64 && serialDebug.available() > 0; ++budget)
    {
        const char ch = static_cast<char>(serialDebug.read());
        if (ch == '\n' || ch == '\r')
        {
            if (discardLine)
                serialDebug.println("REJECT line: overlong or invalid bytes; entire line discarded.");
            else
            {
                commandLine[commandLength] = '\0';
                processCommand();
            }
            commandLength = 0;
            discardLine = false;
        }
        else if (!discardLine)
        {
            if (commandLength >= sizeof(commandLine) - 1 ||
                (ch != '\t' && (ch < ' ' || ch > '~')))
            {
                discardLine = true;
                commandLength = 0;
            }
            else commandLine[commandLength++] = ch;
        }
    }
}

void updateLed()
{
    bool on = false;
    if (state == TestState::Initializing || running()) on = (millis() / 500) % 2 != 0;
    else if (state == TestState::Passed) on = true;
    else if (state == TestState::Failed) on = (millis() / 150) % 2 != 0;
    digitalWrite(mission_config::STATUS_LED_PIN, on ? HIGH : LOW);
}
} // namespace

void setup()
{
    delay(4000);
    pinMode(mission_config::STATUS_LED_PIN, OUTPUT);
    digitalWrite(mission_config::STATUS_LED_PIN, LOW);
    DebugLog::begin(&serialDebug, debug_config::BAUD, DebugLog::Level::INFO);
    serialDebug.println("FIRST_LAYER_PLACEMENT boot: initialization WILL MOVE the mechanism; keep clear. Manually preload only when safe after initialization.");
    serialDebug.println("WARNING: load only declares preloaded tray slots; it does NOT detect materials. Default slots 1,2,3 loaded (mask=7), target rings 1,2,3.");
    serialDebug.println("Before each run, manually place chassis at storage station STORAGE_ANCHOR (2070,1220 mm), heading yaw=-180. Pose reset does NOT move the car there.");
    serialDebug.println("Vision alignment WILL micro-adjust the chassis. This test performs storage FIRST LAYER only; PASS requires manual physical inspection.");
    printHelp();
    startButton.reset();
    startButton.attachClick(requestRun);
    chassis.begin();
    vision.begin(vision_config::BAUD);
    mechanism.begin();
    updateSequence();
}

void loop()
{
    chassis.update();
    vision.update();
    updateSequence();
    mechanism.update();
    updateSequence();
    alignment.update();
    updateSequence();
    startButton.tick();
    updateSerial();
    const uint32_t now = millis();
    if (now - lastReportMs >= 500)
    {
        lastReportMs = now;
        printStatus();
    }
    updateLed();
}
