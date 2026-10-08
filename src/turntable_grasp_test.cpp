#include <Arduino.h>
#include <OneButton.h>
#include <string.h>

#include "ChassisControl.h"
#include "ChassisMotionPort.h"
#include "DebugLog.h"
#include "GraspForwardPositioner.h"
#include "MaixCamV2.h"
#include "MaixProGraspVision.h"
#include "MaixVisionService.h"
#include "MechanismTaskExecutor.h"
#include "MissionRoutes.h"
#include "chassis_config.h"
#include "debug_config.h"
#include "mechanism_config.h"
#include "mission_config.h"
#include "vision_config.h"

namespace
{
enum class TestState : uint8_t
{
    Initializing,
    Ready,
    Running,
    Passed,
    Failed,
    Cancelled
};

HardwareSerial serialImu(
    chassis_config::IMU_RX_PIN,
    chassis_config::IMU_TX_PIN);
HardwareSerial serialVision(
    vision_config::RX_PIN,
    vision_config::TX_PIN);
HardwareSerial serialMechanismStepper(
    mechanism_config::STEPPER_RX_PIN,
    mechanism_config::STEPPER_TX_PIN);
HardwareSerial serialMechanismBase(
    mechanism_config::BASE_RX_PIN,
    mechanism_config::BASE_TX_PIN);
HardwareSerial serialMechanismServo(
    mechanism_config::SERVO_RX_PIN,
    mechanism_config::SERVO_TX_PIN);
HardwareSerial serialDebug(
    debug_config::RX_PIN,
    debug_config::TX_PIN);

ChassisControl chassis(&serialImu);
ChassisMotionPort graspChassis(chassis, ChassisOwner::Grasp);
GraspForwardPositioner graspForwardPositioner(graspChassis);
maixcam::MaixCamV2 camera(serialVision);
maixcam::MaixVisionService vision(camera);
MaixProGraspVision graspVision(vision);
MechanismTaskExecutor mechanism(
    serialMechanismStepper,
    serialMechanismBase,
    serialMechanismServo,
    graspVision,
    graspForwardPositioner);
OneButton startButton(mission_config::START_BUTTON_PIN, true, true);

BatchMission batch = {{1, 3, 4}, {1, 2, 3}, {1, 2, 3}};
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
    case TestState::Running: return "RUNNING";
    case TestState::Passed: return "PASS";
    case TestState::Failed: return "FAILED";
    case TestState::Cancelled: return "CANCELLED";
    }
    return "UNKNOWN";
}

void printMasks()
{
    const auto &debug = mechanism.graspDebug();
    serialDebug.print(" collectedMask=0x");
    serialDebug.print(debug.collectedMask, HEX);
    serialDebug.print(" missedMask=0x");
    serialDebug.print(debug.missedMask, HEX);
}

void printRetryWarning()
{
    serialDebug.println(
        "RETRY: manually empty the storage tray and check the gripper; "
        "reposition the chassis at MATERIAL_ANCHOR before a new run. "
        "No automatic unloading, repositioning or restart.");
}

void failTest(const char *reason, bool hardFault)
{
    if (resetRequired && state == TestState::Failed)
        return;

    resetRequired = resetRequired || hardFault;
    state = TestState::Failed;
    serialDebug.print("FAILED reason=");
    serialDebug.print(reason);
    printMasks();
    serialDebug.println();
    mechanism.cancel();
    chassis.stop();
    if (resetRequired)
        serialDebug.println("RESET REQUIRED: fix the hardware fault, then reset the controller. run is locked.");
    printRetryWarning();
}

void updateSequence()
{
    if (resetRequired && state == TestState::Failed)
        return;

    if (chassis.state() == ChassisControl::State::Fault)
    {
        failTest(chassis.faultMessage(), true);
        return;
    }
    if (mechanism.result() == AsyncResult::Failed)
    {
        serialDebug.print("FAULT source=");
        serialDebug.println(state == TestState::Initializing ? "initialization" : "mechanism");
        failTest(mechanism.faultMessage(), true);
        return;
    }
    if (state == TestState::Initializing &&
        mechanism.ready() && mechanism.result() != AsyncResult::Running)
    {
        state = TestState::Ready;
        serialDebug.println("READY mechanism initialization complete; run still requires IMU ready and chassis idle.");
    }
    if (state != TestState::Running)
        return;

    if (mechanism.result() == AsyncResult::Succeeded)
    {
        const auto &debug = mechanism.graspDebug();
        if (debug.collectedMask != 7 || debug.missedMask != 0)
        {
            failTest("collection incomplete", false);
            return;
        }
        state = TestState::Passed;
        serialDebug.print("PASS round=0");
        printMasks();
        serialDebug.println(" command sequence completed only; manually confirm all three physical materials.");
        printRetryWarning();
    }
    else if (mechanism.result() != AsyncResult::Running)
    {
        failTest("collection ended without Succeeded", true);
    }
}

bool idleForCommand()
{
    return !resetRequired &&
           state != TestState::Initializing &&
           state != TestState::Running &&
           mechanism.ready() &&
           mechanism.result() != AsyncResult::Running &&
           !chassis.busy() &&
           chassis.state() != ChassisControl::State::Fault;
}

void requestRun()
{
    updateSequence();
    if (!idleForCommand() || !chassis.imuReady())
    {
        serialDebug.println("REJECT run: initialization incomplete, busy, IMU not ready or reset required; request discarded, not queued.");
        return;
    }

    serialDebug.println("RUN: operator must have emptied the tray and repositioned the chassis at MATERIAL_ANCHOR; grasping may move the chassis.");
    if (!chassis.resetWorldPose(
            mission_routes::MATERIAL_ANCHOR.position.xMm,
            mission_routes::MATERIAL_ANCHOR.position.yMm,
            mission_routes::MATERIAL_ANCHOR.yawDeg))
    {
        failTest("MATERIAL_ANCHOR pose reset rejected", true);
        return;
    }
    if (!mechanism.start(StationTask::CollectMaterial, 0, batch))
    {
        failTest("CollectMaterial start rejected", true);
        return;
    }
    state = TestState::Running;
    serialDebug.println("RUNNING CollectMaterial round=0; no placement or travel preparation.");
    updateSequence();
}

void requestCancel()
{
    updateSequence();
    if (resetRequired && state == TestState::Failed)
    {
        serialDebug.println("REJECT cancel: hardware failure remains latched; reset required.");
        return;
    }
    if (state == TestState::Cancelled)
        return;

    const bool initializationInterrupted = !mechanism.ready();
    mechanism.cancel();
    chassis.stop();
    resetRequired = resetRequired || initializationInterrupted;
    state = TestState::Cancelled;
    serialDebug.print("CANCELLED");
    printMasks();
    serialDebug.println();
    printRetryWarning();
    if (resetRequired)
        serialDebug.println("RESET REQUIRED: initialization was cancelled and is not complete; no automatic reinitialization.");
}

void printStatus()
{
    const auto &debug = mechanism.graspDebug();
    serialDebug.print("GRASP t=");
    serialDebug.print(millis());
    serialDebug.print(" state=");
    serialDebug.print(stateName());
    serialDebug.print(" phase=");
    serialDebug.print(mechanism.debugPhase());
    serialDebug.print(" item_slot_1based=");
    serialDebug.print(debug.item);
    serialDebug.print(" attempt=");
    serialDebug.print(debug.attempt);
    printMasks();
    serialDebug.print(" dx=");
    serialDebug.print(debug.dx);
    serialDebug.print(" dy=");
    serialDebug.print(debug.dy);
    serialDebug.print(" quality=");
    serialDebug.print(debug.quality);
    serialDebug.print(" extension_target=");
    serialDebug.print(debug.extensionTarget, 1);
    serialDebug.print(" extension_command=");
    serialDebug.print(mechanism.motionDebug().extensionTarget, 1);
    serialDebug.print(" forward_mm=");
    serialDebug.print(debug.forwardOffsetMm, 1);
    serialDebug.print(" right_mm=");
    serialDebug.print(debug.rightOffsetMm, 1);
    serialDebug.print(" tracking=");
    serialDebug.print(debug.tracking ? 1 : 0);
    serialDebug.print(" found=");
    serialDebug.print(debug.found ? 1 : 0);
    serialDebug.print(" observation=");
    serialDebug.print(debug.hasObservation ? 1 : 0);
    serialDebug.print(" mechanism_ready=");
    serialDebug.print(mechanism.ready() ? 1 : 0);
    serialDebug.print(" imu_ready=");
    serialDebug.print(chassis.imuReady() ? 1 : 0);
    serialDebug.print(" chassis_busy=");
    serialDebug.print(chassis.busy() ? 1 : 0);
    serialDebug.print(" chassis_fault=");
    serialDebug.print(chassis.state() == ChassisControl::State::Fault ? 1 : 0);
    serialDebug.print(" reset_required=");
    serialDebug.print(resetRequired ? 1 : 0);
    serialDebug.print(" colors=");
    for (uint8_t i = 0; i < MATERIALS_PER_BATCH; ++i)
    {
        if (i != 0)
            serialDebug.print(',');
        serialDebug.print(batch.colors[i]);
    }
    serialDebug.println();
}

void printHelp()
{
    serialDebug.println("Commands: set <color1> <color2> <color3> | run | cancel | status | help");
    serialDebug.println("Colors: 1=red 2=yellow 3=blue 4=green; exactly three distinct colors, idle-only set. PB9 click = run, never queued.");
    serialDebug.println("item_slot_1based: 1..3 maps to batch index 0..2; 0 means no item yet. Mask bit 0..2 maps to batch index 0..2.");
    serialDebug.println("Hardware faults require controller reset. cancel does not unload materials or restore pose; manually verify physical materials even after command completion.");
}

void processCommand()
{
    char *tokens[5] = {};
    size_t count = 0;
    char *cursor = commandLine;
    while (*cursor != '\0')
    {
        while (*cursor == ' ' || *cursor == '\t')
            ++cursor;
        if (*cursor == '\0')
            break;
        if (count == 5)
        {
            serialDebug.println("REJECT command: too many arguments.");
            return;
        }
        tokens[count++] = cursor;
        while (*cursor != '\0' && *cursor != ' ' && *cursor != '\t')
            ++cursor;
        if (*cursor != '\0')
            *cursor++ = '\0';
    }
    if (count == 0)
        return;

    if (count == 1 && strcmp(tokens[0], "run") == 0)
        requestRun();
    else if (count == 1 && strcmp(tokens[0], "cancel") == 0)
        requestCancel();
    else if (count == 1 && strcmp(tokens[0], "status") == 0)
        printStatus();
    else if (count == 1 && strcmp(tokens[0], "help") == 0)
        printHelp();
    else if (count == 4 && strcmp(tokens[0], "set") == 0)
    {
        uint8_t colors[3] = {};
        uint8_t mask = 0;
        for (uint8_t i = 0; i < 3; ++i)
        {
            const char *token = tokens[i + 1];
            if (token[0] < '1' || token[0] > '4' || token[1] != '\0')
            {
                serialDebug.println("REJECT set: colors must be single digits 1..4.");
                return;
            }
            colors[i] = static_cast<uint8_t>(token[0] - '0');
            const uint8_t bit = static_cast<uint8_t>(1U << colors[i]);
            if ((mask & bit) != 0)
            {
                serialDebug.println("REJECT set: colors must be distinct.");
                return;
            }
            mask |= bit;
        }
        updateSequence();
        if (!idleForCommand())
        {
            serialDebug.println("REJECT set: idle and initialized only; reset-required faults are locked.");
            return;
        }
        memcpy(batch.colors, colors, sizeof(colors));
        serialDebug.println("OK set");
        printStatus();
    }
    else
        serialDebug.println("REJECT command: invalid syntax or trailing arguments; use help.");
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
            else
                commandLine[commandLength++] = ch;
        }
    }
}

void updateLed()
{
    bool on = false;
    if (state == TestState::Running)
        on = (millis() / 500) % 2 != 0;
    else if (state == TestState::Passed)
        on = true;
    else if (state == TestState::Failed)
        on = (millis() / 150) % 2 != 0;
    digitalWrite(mission_config::STATUS_LED_PIN, on ? HIGH : LOW);
}
}

void setup()
{
    delay(4000);
    pinMode(mission_config::STATUS_LED_PIN, OUTPUT);
    digitalWrite(mission_config::STATUS_LED_PIN, LOW);
    DebugLog::begin(&serialDebug, debug_config::BAUD, DebugLog::Level::INFO);
    serialDebug.println("TURNTABLE_GRASP boot: mechanism initialization WILL MOVE the mechanism; keep clear.");
    serialDebug.println("WARNING: run performs real grasping and chassis micro-adjustments. Empty the tray, check the gripper and place the chassis at MATERIAL_ANCHOR before run.");
    serialDebug.println("Completion means commands/actions completed, NOT physical grasp confirmation; manually inspect the materials.");
    printHelp();
    startButton.reset();
    startButton.attachClick(requestRun);
    chassis.begin();
    chassis.resetWorldPose(
        mission_routes::MATERIAL_ANCHOR.position.xMm,
        mission_routes::MATERIAL_ANCHOR.position.yMm,
        mission_routes::MATERIAL_ANCHOR.yawDeg);
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
