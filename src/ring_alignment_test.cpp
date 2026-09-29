#include <Arduino.h>
#include <OneButton.h>

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

namespace
{
/**
 * @brief 圆环测试不执行原料抓取，使用空视觉端口满足机构模块接口。
 */
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

/**
 * @brief 圆环测试不执行原料区底盘修正，端口保持空闲。
 */
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
    WaitingForStart,
    PreparingView,
    Aligning,
    Completed,
    Fault
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
ChassisMotionPort alignmentChassis(chassis, ChassisOwner::Alignment);
maixcam::MaixCamV2 camera(serialVision);
maixcam::MaixVisionService vision(camera);
IdleGraspVision idleGraspVision;
MaixProRingAlignment alignment(vision, alignmentChassis);
IdleForwardPositioner idleForwardPositioner;
MechanismTaskExecutor mechanism(
    serialMechanismStepper,
    serialMechanismBase,
    serialMechanismServo,
    idleGraspVision,
    idleForwardPositioner);

OneButton startButton(
    mission_config::START_BUTTON_PIN,
    true,
    true);

TestState state = TestState::WaitingForStart;
bool startRequested = false;
uint32_t lastReportMs = 0;

void onStartClicked()
{
    startRequested = true;
    serialDebug.println("EVT button_click");
}

const char *stateName()
{
    switch (state)
    {
    case TestState::WaitingForStart:
        return "READY";
    case TestState::PreparingView:
        return "PREP";
    case TestState::Aligning:
        return "ALIGN";
    case TestState::Completed:
        return "DONE";
    case TestState::Fault:
        return "FAULT";
    }
    return "UNKNOWN";
}

void enterFault(const char *message)
{
    chassis.stop();
    alignment.cancel();
    mechanism.cancel();
    state = TestState::Fault;
    serialDebug.print("FAULT ");
    serialDebug.println(message);
}

void updateSequence()
{
    if (mechanism.result() == AsyncResult::Failed)
    {
        enterFault(mechanism.faultMessage());
        return;
    }

    switch (state)
    {
    case TestState::WaitingForStart:
    case TestState::Completed:
    case TestState::Fault:
        if (!startRequested ||
            !mechanism.ready() ||
            mechanism.result() == AsyncResult::Running)
        {
            return;
        }

        startRequested = false;
        if (!mechanism.prepareForTravel(
                TravelDestination::RoughProcessing))
        {
            enterFault("mechanism preparation rejected");
            return;
        }
        state = TestState::PreparingView;
        break;

    case TestState::PreparingView:
    {
        if (mechanism.result() != AsyncResult::Succeeded)
            return;

        AlignmentRequest request;
        request.station = Station::RoughProcessing;
        request.round = 0;
        if (!alignment.start(request))
        {
            enterFault("ring alignment start rejected");
            return;
        }
        state = TestState::Aligning;
        break;
    }

    case TestState::Aligning:
        if (alignment.result() == AsyncResult::Succeeded)
            state = TestState::Completed;
        else if (alignment.result() == AsyncResult::Failed)
            enterFault("ring alignment failed");
        break;

    }
}

void printReport()
{
    const uint32_t now = millis();
    if (now - lastReportMs <
        debug_config::STARTUP_REPORT_INTERVAL_MS)
    {
        return;
    }
    lastReportMs = now;

    const ChassisControl::Pose2D pose = chassis.worldPose();
    const MaixProRingAlignment::DebugState &ring =
        alignment.debugState();

    serialDebug.print("RING t=");
    serialDebug.print(now);
    serialDebug.print(" state=");
    serialDebug.print(stateName());
    serialDebug.print(" pose=");
    serialDebug.print(pose.xMm, 0);
    serialDebug.print(",");
    serialDebug.print(pose.yMm, 0);
    serialDebug.print(",");
    serialDebug.print(pose.yawDeg, 0);
    serialDebug.print(" found=");
    serialDebug.print(ring.hasObservation && ring.found ? 1 : 0);
    serialDebug.print(" dx=");
    serialDebug.print(ring.dx);
    serialDebug.print(" dy=");
    serialDebug.print(ring.dy);
    serialDebug.print(" err=");
    serialDebug.print(
        ring.dx - vision_config::RING_TARGET_DX_PX);
    serialDebug.print(",");
    serialDebug.print(
        ring.dy - vision_config::RING_TARGET_DY_PX);
    serialDebug.print(" q=");
    serialDebug.print(ring.quality);
    serialDebug.print(" moveF=");
    serialDebug.print(ring.forwardMm, 1);
    serialDebug.print(" moveR=");
    serialDebug.print(ring.rightMm, 1);
    serialDebug.print(" stable=");
    serialDebug.print(ring.stableFrames);
    serialDebug.print(" moving=");
    serialDebug.print(ring.movePending ? 1 : 0);
    serialDebug.print(" target=");
    serialDebug.print(ring.targetMode, HEX);
    serialDebug.print(":");
    serialDebug.print(ring.targetSelector);
    serialDebug.print(" last=");
    serialDebug.print(ring.lastMode, HEX);
    serialDebug.print(":");
    serialDebug.print(ring.lastTargetId);
    serialDebug.print(" obs=");
    serialDebug.print(ring.observations);
    serialDebug.print(" ign=");
    serialDebug.println(ring.ignoredObservations);
}

void updateLed()
{
    if (state == TestState::Fault)
    {
        digitalWrite(
            mission_config::STATUS_LED_PIN,
            (millis() / 150) % 2 ? HIGH : LOW);
    }
    else if (state == TestState::Completed)
    {
        digitalWrite(mission_config::STATUS_LED_PIN, HIGH);
    }
    else
    {
        digitalWrite(
            mission_config::STATUS_LED_PIN,
            (millis() / 500) % 2 ? HIGH : LOW);
    }
}
} // namespace

void setup()
{
    delay(4000);
    pinMode(mission_config::STATUS_LED_PIN, OUTPUT);
    digitalWrite(mission_config::STATUS_LED_PIN, LOW);

    // 通过DebugLog输出视觉服务的REQUEST/ACTIVE/RETRY日志。
    DebugLog::begin(&serialDebug, debug_config::BAUD, DebugLog::Level::INFO);
    serialDebug.println("RING boot");

    startButton.reset();
    startButton.attachClick(onStartClicked);

    chassis.begin();
    chassis.resetWorldPose(
        mission_routes::ROUGH_ANCHOR.position.xMm,
        mission_routes::ROUGH_ANCHOR.position.yMm,
        mission_routes::ROUGH_ANCHOR.yawDeg);
    vision.begin(vision_config::BAUD);
    mechanism.begin();
}

void loop()
{
    startButton.tick();
    chassis.update();
    vision.update();
    mechanism.update();
    alignment.update();
    updateSequence();
    printReport();
    updateLed();
}
