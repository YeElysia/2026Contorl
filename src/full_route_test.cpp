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
#include "RouteExecutor.h"
#include "chassis_config.h"
#include "debug_config.h"
#include "field_config.h"
#include "mechanism_config.h"
#include "mission_config.h"
#include "vision_config.h"

namespace
{
using namespace mission_routes;

// 本测试不抓取原料，使用空端口满足机构模块接口。
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

#ifdef ROUTE_TEST_UPPER_RIGHT
constexpr StartZone TEST_START_ZONE = StartZone::UpperRight;
#else
constexpr StartZone TEST_START_ZONE = StartZone::LowerRight;
#endif
// 分步测试，每次调试串口发送'n'或点击PB9前进一步：
//   1) 启停区 -> 原料区 -> 粗加工区，到达后停车；
//   2) 机械臂转到圆环识别位，视觉对准粗加工区圆环，完成后停车；
//   3) 开到暂存区，停车。
constexpr char NEXT_COMMAND = 'n';

struct Leg
{
    const char *name;
    RouteDefinition route;
};

constexpr Leg LEGS[] = {
    {"HOME->SCAN", ROUTE_TO_SCAN},
    {"SCAN->MATERIAL#1", ROUTE_SCAN_TO_MATERIAL},
    {"MATERIAL->ROUGH#1", ROUTE_MATERIAL_TO_ROUGH[0]},
    {"ROUGH->STORAGE#1", ROUTE_ROUGH_TO_STORAGE[0]},
};
constexpr size_t LEG_COUNT = sizeof(LEGS) / sizeof(LEGS[0]);
constexpr size_t ROUGH_LEG_COUNT = 3;

HardwareSerial serialImu(
    chassis_config::IMU_RX_PIN,
    chassis_config::IMU_TX_PIN);
HardwareSerial serialDebug(
    debug_config::RX_PIN,
    debug_config::TX_PIN);
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

ChassisControl chassis(&serialImu);
ChassisMotionPort routeChassis(chassis, ChassisOwner::Route);
ChassisMotionPort alignmentChassis(chassis, ChassisOwner::Alignment);
RouteExecutor routeExecutor(routeChassis);
maixcam::MaixCamV2 camera(serialVision);
maixcam::MaixVisionService vision(camera);
IdleGraspVision idleGraspVision;
IdleForwardPositioner idleForwardPositioner;
MaixProRingAlignment alignment(vision, alignmentChassis);
MechanismTaskExecutor mechanism(
    serialMechanismStepper,
    serialMechanismBase,
    serialMechanismServo,
    idleGraspVision,
    idleForwardPositioner);

enum class TestState : uint8_t
{
    WaitingForImu,
    WaitingForStart,
    Running,
    AtRough,
    PreparingView,
    Aligning,
    Aligned,
    Done,
    Failed
};

OneButton startButton(mission_config::START_BUTTON_PIN, true, true);

TestState testState = TestState::WaitingForImu;
bool nextPending = false;
size_t legIndex = 0;
uint32_t legStartedMs = 0;
uint32_t runStartedMs = 0;
FieldPose expected = {};

const char *stateName()
{
    switch (testState)
    {
    case TestState::WaitingForImu:
        return "WAIT_IMU";
    case TestState::WaitingForStart:
        return "READY";
    case TestState::Running:
        return "RUNNING(ignored)";
    case TestState::AtRough:
        return "AT_ROUGH";
    case TestState::PreparingView:
        return "PREP_VIEW(ignored)";
    case TestState::Aligning:
        return "ALIGNING(ignored)";
    case TestState::Aligned:
        return "ALIGNED";
    case TestState::Done:
        return "DONE";
    case TestState::Failed:
        return "FAILED(ignored)";
    }
    return "?";
}

void onStartClicked()
{
    nextPending = true;
    serialDebug.print("ROUTE BUTTON state=");
    serialDebug.println(stateName());
}

// 非阻塞读取调试串口；行驶中收到的命令直接丢弃，避免跑完后立即重跑。
void pollSerialCommand()
{
    while (serialDebug.available())
    {
        const int data = serialDebug.read();
        if (data == NEXT_COMMAND || data == NEXT_COMMAND - 'a' + 'A')
        {
            nextPending = true;
            serialDebug.print("ROUTE RX n state=");
            serialDebug.println(stateName());
        }
    }
}

bool takeNext()
{
    const bool next = nextPending;
    nextPending = false;
    return next;
}

// 路线终点：最后一个带坐标动作的坐标，最后一个带航向动作的航向。
FieldPose routeEndPose(RouteDefinition route, FieldPose from)
{
    for (size_t i = 0; i < route.count; ++i)
    {
        const RouteAction &action = route.actions[i];
        if (action.type != RouteActionType::RotateTo)
            from.position = {action.xMm, action.yMm};
        if (action.type == RouteActionType::RotateTo ||
            action.type == RouteActionType::MoveToPose)
            from.yawDeg = action.yawDeg;
    }
    return from;
}

void printPose(const char *label, float xMm, float yMm, float yawDeg)
{
    serialDebug.print(label);
    serialDebug.print("=");
    serialDebug.print(xMm, 1);
    serialDebug.print(",");
    serialDebug.print(yMm, 1);
    serialDebug.print(",");
    serialDebug.print(yawDeg, 2);
}

void startLeg()
{
    const Leg &leg = LEGS[legIndex];
    if (!routeExecutor.start(leg.route))
    {
        testState = TestState::Failed;
        serialDebug.print("ROUTE START_REJECTED leg=");
        serialDebug.print(leg.name);
        serialDebug.print(" fault=");
        serialDebug.println(chassis.faultMessage());
        return;
    }

    expected = routeEndPose(leg.route, expected);
    legStartedMs = millis();
    testState = TestState::Running;

    serialDebug.print("ROUTE LEG ");
    serialDebug.print(legIndex + 1);
    serialDebug.print("/");
    serialDebug.print(LEG_COUNT);
    serialDebug.print(" ");
    serialDebug.print(leg.name);
    serialDebug.print(" actions=");
    serialDebug.println(leg.route.count);
}

void startRun()
{
    const field_config::StartPose start =
        field_config::startPose(TEST_START_ZONE);
    if (!chassis.resetWorldPose(start.xMm, start.yMm, start.yawDeg))
    {
        serialDebug.print("ROUTE RESET_REJECTED fault=");
        serialDebug.println(chassis.faultMessage());
        return;
    }

    expected = {{start.xMm, start.yMm}, start.yawDeg};
    legIndex = 0;
    runStartedMs = millis();
    startLeg();
}

void reportStation()
{
    const ChassisControl::Pose2D pose = chassis.worldPose();

    serialDebug.print("ROUTE ARRIVED ");
    serialDebug.print(LEGS[legIndex].name);
    serialDebug.print(" elapsed=");
    serialDebug.print(millis() - legStartedMs);
    serialDebug.print("ms ");
    printPose("expect", expected.position.xMm, expected.position.yMm, expected.yawDeg);
    serialDebug.print(" ");
    printPose("odom", pose.xMm, pose.yMm, pose.yawDeg);
    serialDebug.print(" imuYaw=");
    serialDebug.println(chassis.yawDeg(), 2);
}

void onLegFinished()
{
    reportStation();
    ++legIndex;

    if (legIndex == ROUGH_LEG_COUNT)
    {
        testState = TestState::AtRough;
        serialDebug.println(
            "ROUTE AT_ROUGH, send 'n' to rotate arm to ring view and align");
        return;
    }

    if (legIndex >= LEG_COUNT)
    {
        testState = TestState::Done;
        serialDebug.print("ROUTE DONE total=");
        serialDebug.print(millis() - runStartedMs);
        serialDebug.println("ms, stopped at STORAGE; return car to start before sending 'n' again");
        return;
    }

    startLeg();
}

void enterFault(const char *message)
{
    chassis.stop();
    alignment.cancel();
    mechanism.cancel();
    testState = TestState::Failed;

    const ChassisControl::Pose2D pose = chassis.worldPose();
    serialDebug.print("ROUTE FAULT ");
    serialDebug.print(message);
    serialDebug.print(" ");
    printPose("odom", pose.xMm, pose.yMm, pose.yawDeg);
    serialDebug.println(", reset board to retry");
}

// 第二步：机械臂转到圆环识别位，然后视觉对准粗加工区圆环。
void startViewPrepare()
{
    if (!mechanism.ready() || mechanism.result() == AsyncResult::Running)
    {
        serialDebug.println("ROUTE WAIT mechanism not ready, send 'n' again");
        return;
    }
    if (!mechanism.prepareForTravel(TravelDestination::RoughProcessing))
    {
        enterFault("mechanism preparation rejected");
        return;
    }
    testState = TestState::PreparingView;
    serialDebug.println("ROUTE ARM -> ring view position");
}

void printAlignReport()
{
    static uint32_t lastMs = 0;
    const uint32_t now = millis();
    if (now - lastMs < 300)
        return;
    lastMs = now;

    const ChassisControl::Pose2D pose = chassis.worldPose();
    const MaixProRingAlignment::DebugState &ring = alignment.debugState();

    serialDebug.print("ROUTE RING t=");
    serialDebug.print(now);
    serialDebug.print(" pose=");
    serialDebug.print(pose.xMm, 1);
    serialDebug.print(",");
    serialDebug.print(pose.yMm, 1);
    serialDebug.print(",");
    serialDebug.print(pose.yawDeg, 1);
    serialDebug.print(" found=");
    serialDebug.print(ring.hasObservation && ring.found ? 1 : 0);
    serialDebug.print(" dx=");
    serialDebug.print(ring.dx);
    serialDebug.print(" dy=");
    serialDebug.print(ring.dy);
    serialDebug.print(" err=");
    serialDebug.print(ring.dx - vision_config::RING_TARGET_DX_PX);
    serialDebug.print(",");
    serialDebug.print(ring.dy - vision_config::RING_TARGET_DY_PX);
    serialDebug.print(" q=");
    serialDebug.print(ring.quality);
    serialDebug.print(" stable=");
    serialDebug.print(ring.stableFrames);
    serialDebug.print(" moving=");
    serialDebug.print(ring.movePending ? 1 : 0);
    serialDebug.print(" obs=");
    serialDebug.print(ring.observations);
    serialDebug.print(" ign=");
    serialDebug.print(ring.ignoredObservations);
    serialDebug.print(" last=");
    serialDebug.print(ring.lastMode, HEX);
    serialDebug.print(":");
    serialDebug.println(ring.lastTargetId);
}

void updateAlignStage()
{
    if (testState == TestState::Aligning)
        printAlignReport();

    if (mechanism.result() == AsyncResult::Failed)
    {
        enterFault(mechanism.faultMessage());
        return;
    }

    if (testState == TestState::PreparingView)
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
        testState = TestState::Aligning;
        serialDebug.println("ROUTE ALIGN start ring alignment");
    }
    else if (alignment.result() == AsyncResult::Succeeded)
    {
        testState = TestState::Aligned;
        const ChassisControl::Pose2D pose = chassis.worldPose();
        serialDebug.print("ROUTE ALIGNED ");
        printPose("odom", pose.xMm, pose.yMm, pose.yawDeg);
        serialDebug.println(", stopped; send 'n' to drive to STORAGE");
    }
    else if (alignment.result() == AsyncResult::Failed)
    {
        enterFault("ring alignment failed");
    }
}

void onLegFailed()
{
    testState = TestState::Failed;
    const ChassisControl::Pose2D pose = chassis.worldPose();

    serialDebug.print("ROUTE FAULT leg=");
    serialDebug.print(LEGS[legIndex].name);
    serialDebug.print(" fault=");
    serialDebug.print(routeExecutor.faultMessage());
    serialDebug.print(" ");
    printPose("odom", pose.xMm, pose.yMm, pose.yawDeg);
    serialDebug.println(", reset board to retry");
}

void updateStatusLed()
{
    bool on = false;
    switch (testState)
    {
    case TestState::Running:
        on = true;
        break;
    case TestState::PreparingView:
    case TestState::Aligning:
        on = true;
        break;
    case TestState::Failed:
        on = (millis() / 150) % 2;
        break;
    default:
        break;
    }
    digitalWrite(mission_config::STATUS_LED_PIN, on ? HIGH : LOW);
}
} // namespace

void setup()
{
    delay(1000);

    pinMode(mission_config::STATUS_LED_PIN, OUTPUT);
    digitalWrite(mission_config::STATUS_LED_PIN, LOW);

    // 底盘内部的MOVE_START/MOVE_COMPLETE等日志也经由DebugLog输出。
    DebugLog::begin(&serialDebug, debug_config::BAUD, DebugLog::Level::INFO);
    chassis.begin();
    vision.begin(vision_config::BAUD);
    mechanism.begin();

    startButton.reset();
    startButton.attachClick(onStartClicked);

    serialDebug.println("ROUTE boot waiting for IMU");
}

void loop()
{
    // 速度模式有100ms看门狗，循环内只处理串口命令、底盘和路线，不做阻塞操作。
    pollSerialCommand();
    startButton.tick();
    chassis.update();
    vision.update();
    mechanism.update();
    alignment.update();
    routeExecutor.update();

    switch (testState)
    {
    case TestState::WaitingForImu:
        if (chassis.imuReady())
        {
            testState = TestState::WaitingForStart;
            takeNext();
            serialDebug.print("ROUTE READY zone=");
            serialDebug.print(
                TEST_START_ZONE == StartZone::UpperRight ? "UPPER_RIGHT" : "LOWER_RIGHT");
            serialDebug.print(" legs=");
            serialDebug.print(LEG_COUNT);
            serialDebug.println(", send 'n' or click start button (PB9) to start");
        }
        break;

    case TestState::WaitingForStart:
    case TestState::Done:
        if (takeNext())
            startRun();
        break;

    case TestState::Running:
        takeNext();
        if (routeExecutor.result() == AsyncResult::Succeeded)
            onLegFinished();
        else if (routeExecutor.result() == AsyncResult::Failed)
            onLegFailed();
        break;

    case TestState::AtRough:
        if (takeNext())
            startViewPrepare();
        break;

    case TestState::PreparingView:
    case TestState::Aligning:
        takeNext();
        updateAlignStage();
        break;

    case TestState::Aligned:
        if (takeNext())
            startLeg();
        break;

    case TestState::Failed:
        takeNext();
        break;
    }

    updateStatusLed();
}
