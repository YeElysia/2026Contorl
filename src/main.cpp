#include <Arduino.h>
#include <OneButton.h>

#include "ChassisControl.h"
#include "GraspForwardPositioner.h"
#include "MissionController.h"
#include "MaixProGraspVision.h"
#include "MaixProRingAlignment.h"
#include "MechanismTaskExecutor.h"
#include "NextionMissionDisplay.h"
#include "QRCodeMissionProvider.h"
#include "RouteExecutor.h"
#include "StartupDiagnostics.h"
#include "chassis_config.h"
#include "debug_config.h"
#include "field_config.h"
#include "mechanism_config.h"
#include "mission_config.h"
#include "vision_config.h"

namespace
{
    HardwareSerial serialImu(
        chassis_config::IMU_RX_PIN,
        chassis_config::IMU_TX_PIN);
    HardwareSerial serialQr(
        mission_config::QR_RX_PIN,
        mission_config::QR_TX_PIN);
    HardwareSerial serialScreen(
        mission_config::SCREEN_RX_PIN,
        mission_config::SCREEN_TX_PIN);
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
    GraspForwardPositioner graspForwardPositioner(chassis);
    RouteExecutor routeExecutor(chassis);

    QRCodeMissionProvider missionData(
        serialQr,
        mission_config::QR_BAUD,
        mission_config::QR_TIMEOUT_MS);
    maixcam::MaixCamV2 camera(serialVision);
    MaixProGraspVision graspVision(camera);
    MaixProRingAlignment alignment(camera, graspVision, chassis);
    MechanismTaskExecutor stationTask(
        serialMechanismStepper,
        serialMechanismBase,
        serialMechanismServo,
        graspVision,
        graspForwardPositioner);

    MissionController mission(
        routeExecutor,
        missionData,
        alignment,
        stationTask);

    NextionMissionDisplay display(
        serialScreen,
        mission,
        missionData);
    StartupDiagnostics diagnostics(
        serialDebug,
        mission_config::START_BUTTON_PIN,
        mission,
        routeExecutor,
        chassis,
        alignment,
        stationTask);

    OneButton startButton(
        mission_config::START_BUTTON_PIN,
        true,
        true);

    void onStartButtonClicked()
    {
        diagnostics.noteButtonClick();

        /*
         * 发车瞬间锁定串口屏选择，并以对应启停区重建世界坐标。
         * n4尚未返回时使用安全默认值0（右下启停区）。
         */
        if (mission.state() == MissionController::State::Startup ||
            mission.state() ==
                MissionController::State::WaitingForStart)
        {
            const StartZone zone = display.selectedStartZone();
            diagnostics.noteStartZoneSelection(
                zone,
                display.hasStartZoneSelection());
            mission.selectStartZone(zone);

            const field_config::StartPose pose =
                field_config::startPose(zone);
            chassis.resetWorldPose(
                pose.xMm,
                pose.yMm,
                pose.yawDeg);
        }

        mission.requestStart();
    }

    void updateStatusLed()
    {
        if (mission.faulted())
        {
            // 故障时快闪，避免与正常完成后的熄灭状态混淆。
            digitalWrite(
                mission_config::STATUS_LED_PIN,
                (millis() / 150) % 2 ? HIGH : LOW);
        }
        else if (mission.running())
        {
            digitalWrite(mission_config::STATUS_LED_PIN, HIGH);
        }
        else
        {
            digitalWrite(mission_config::STATUS_LED_PIN, LOW);
        }
    }
} // namespace

void setup()
{
    delay(4000); // 等待电源稳定，避免启动时电压下降导致串口初始化失败。
    pinMode(mission_config::STATUS_LED_PIN, OUTPUT);
    digitalWrite(mission_config::STATUS_LED_PIN, LOW);

    startButton.reset();
    startButton.attachClick(onStartButtonClicked);

    chassis.begin();
    /*
     * 底盘内部记录的是场地绝对坐标。此时IMU可能尚未输出首帧，
     * resetWorldPose会保留初始航向，收到首帧后自动建立航向零点。
     */
    chassis.resetWorldPose(
        field_config::START_X_MM,
        field_config::START_Y_MM,
        field_config::START_YAW_DEG);
    camera.begin(vision_config::BAUD);
    missionData.begin();
    mission.begin(mission_config::STARTUP_STABLE_MS);
    diagnostics.begin(debug_config::BAUD);
    stationTask.begin();
    display.begin(
        mission_config::SCREEN_BAUD,
        mission_config::SCREEN_RESTART_WAIT_MS);
}

void loop()
{
    /*
     * 所有模块都采用非阻塞update()：
     * - ChassisControl解析IMU并产生STEP脉冲；
     * - MissionController推进路线、视觉和机构任务；
     * - OneButton处理按键消抖。
     */
    // 按键必须优先扫描，避免机构串口查询拉长循环后漏掉短按。
    startButton.tick();
    chassis.update();
    mission.update();
    diagnostics.update(
        debug_config::STARTUP_REPORT_INTERVAL_MS);
    display.update();
    updateStatusLed();
}
