#include <Arduino.h>

#include "ChassisControl.h"
#include "ChassisMotionPort.h"
#include "DebugLog.h"
#include "MissionRoutes.h"
#include "RouteExecutor.h"
#include "chassis_config.h"
#include "debug_config.h"
#include "field_config.h"
#include "mission_config.h"

namespace
{
using namespace mission_routes;

constexpr StartZone TEST_START_ZONE = StartZone::LowerRight;
// 调试串口发送'n'后一次跑完全部路段，到站只打印位姿不停留。
constexpr char NEXT_COMMAND = 'n';

struct Leg
{
    const char *name;
    RouteDefinition route;
};

// 与MissionController两轮任务的路线顺序一致，但不做视觉重定位。
constexpr Leg LEGS[] = {
    {"HOME->SCAN", ROUTE_TO_SCAN},
    {"SCAN->MATERIAL#1", ROUTE_SCAN_TO_MATERIAL},
    {"MATERIAL->ROUGH#1", ROUTE_MATERIAL_TO_ROUGH[0]},
    {"ROUGH->STORAGE#1", ROUTE_ROUGH_TO_STORAGE[0]},
    {"STORAGE->MATERIAL#2", ROUTE_STORAGE_TO_MATERIAL_SECOND},
    {"MATERIAL->ROUGH#2", ROUTE_MATERIAL_TO_ROUGH[1]},
    {"ROUGH->STORAGE#2", ROUTE_ROUGH_TO_STORAGE[1]},
    {"STORAGE->HOME", storageToHomeRoute(TEST_START_ZONE)},
};
constexpr size_t LEG_COUNT = sizeof(LEGS) / sizeof(LEGS[0]);

HardwareSerial serialImu(
    chassis_config::IMU_RX_PIN,
    chassis_config::IMU_TX_PIN);
HardwareSerial serialDebug(
    debug_config::RX_PIN,
    debug_config::TX_PIN);

ChassisControl chassis(&serialImu);
ChassisMotionPort routeChassis(chassis, ChassisOwner::Route);
RouteExecutor routeExecutor(routeChassis);

enum class TestState : uint8_t
{
    WaitingForImu,
    WaitingForStart,
    Running,
    Done,
    Failed
};

TestState testState = TestState::WaitingForImu;
bool nextPending = false;
size_t legIndex = 0;
uint32_t legStartedMs = 0;
uint32_t runStartedMs = 0;
FieldPose expected = {};

// 非阻塞读取调试串口；行驶中收到的命令直接丢弃，避免跑完后立即重跑。
void pollSerialCommand()
{
    while (serialDebug.available())
    {
        const int data = serialDebug.read();
        if (data == NEXT_COMMAND || data == NEXT_COMMAND - 'a' + 'A')
            nextPending = true;
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

    if (legIndex >= LEG_COUNT)
    {
        testState = TestState::Done;
        serialDebug.print("ROUTE DONE total=");
        serialDebug.print(millis() - runStartedMs);
        serialDebug.println("ms, send 'n' to run again");
        return;
    }

    startLeg();
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

    serialDebug.println("ROUTE boot waiting for IMU");
}

void loop()
{
    // 速度模式有100ms看门狗，循环内只处理串口命令、底盘和路线，不做阻塞操作。
    pollSerialCommand();
    chassis.update();
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
            serialDebug.println(", send 'n' to start");
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

    case TestState::Failed:
        takeNext();
        break;
    }

    updateStatusLed();
}
