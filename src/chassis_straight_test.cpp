#include <Arduino.h>
#include <OneButton.h>

#include "ChassisControl.h"
#include "chassis_config.h"
#include "debug_config.h"
#include "mission_config.h"

namespace
{
constexpr float TEST_DISTANCE_MM = 1000.0F;

HardwareSerial serialImu(
    chassis_config::IMU_RX_PIN,
    chassis_config::IMU_TX_PIN);
HardwareSerial serialDebug(
    debug_config::RX_PIN,
    debug_config::TX_PIN);

ChassisControl chassis(&serialImu);
OneButton startButton(
    mission_config::START_BUTTON_PIN,
    true,
    true);

bool startRequested = false;
bool moving = false;
bool forward = true;
bool readyReported = false;
uint32_t motionStartedMs = 0;

void onStartClicked()
{
    if (!moving)
        startRequested = true;
}

void printReady()
{
    serialDebug.print("STRAIGHT READY distance=");
    serialDebug.print(TEST_DISTANCE_MM, 0);
    serialDebug.print(" rpm=");
    serialDebug.print(chassis_config::DRIVE_RPM, 0);
    serialDebug.print(" accel=");
    serialDebug.print(
        chassis_config::DRIVE_ACCEL_RPM_PER_S,
        0);
    serialDebug.print(" microsteps=");
    serialDebug.println(chassis_config::MICROSTEPS);
}

void startMotion()
{
    startRequested = false;
    const float distance =
        forward ? TEST_DISTANCE_MM : -TEST_DISTANCE_MM;

    if (!chassis.moveRelative(
            distance,
            0.0F,
            chassis_config::DRIVE_RPM,
            chassis_config::DRIVE_ACCEL_RPM_PER_S))
    {
        serialDebug.print("STRAIGHT START_REJECTED fault=");
        serialDebug.println(chassis.faultMessage());
        return;
    }

    moving = true;
    motionStartedMs = millis();
    digitalWrite(mission_config::STATUS_LED_PIN, HIGH);

    serialDebug.print("STRAIGHT START direction=");
    serialDebug.println(forward ? "FORWARD" : "BACKWARD");
}

void finishMotion()
{
    moving = false;
    digitalWrite(mission_config::STATUS_LED_PIN, LOW);

    if (chassis.state() == ChassisControl::State::Fault)
    {
        serialDebug.print("STRAIGHT FAULT ");
        serialDebug.println(chassis.faultMessage());
        return;
    }

    const ChassisControl::Pose2D pose = chassis.worldPose();
    serialDebug.print("STRAIGHT DONE elapsed=");
    serialDebug.print(millis() - motionStartedMs);
    serialDebug.print(" pose=");
    serialDebug.print(pose.xMm, 0);
    serialDebug.print(",");
    serialDebug.print(pose.yMm, 0);
    serialDebug.print(",");
    serialDebug.println(pose.yawDeg, 1);

    // 下一次单击反向行驶，便于架空状态下重复比较。
    forward = !forward;
}
} // namespace

void setup()
{
    delay(1000);

    pinMode(mission_config::STATUS_LED_PIN, OUTPUT);
    digitalWrite(mission_config::STATUS_LED_PIN, LOW);

    serialDebug.begin(debug_config::BAUD);
    chassis.begin();
    chassis.resetWorldPose(0.0F, 0.0F, 0.0F);

    startButton.reset();
    startButton.attachClick(onStartClicked);

    serialDebug.println("STRAIGHT boot waiting for IMU");
}

void loop()
{
    /*
     * 测试运行期间只调用按键和底盘，不查询机械臂、相机、扫码器或
     * 串口屏，也不周期打印日志，确保观察到的顿挫来自底盘本身。
     */
    startButton.tick();
    chassis.update();

    if (!readyReported && chassis.imuReady())
    {
        readyReported = true;
        printReady();
    }

    if (startRequested && !moving && chassis.imuReady())
        startMotion();

    if (moving && !chassis.busy())
        finishMotion();
}
