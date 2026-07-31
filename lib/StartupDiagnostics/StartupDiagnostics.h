#pragma once

#include <Arduino.h>

#include "ChassisControl.h"
#include "MaixProRingAlignment.h"
#include "MechanismTaskExecutor.h"
#include "MissionController.h"
#include "RouteExecutor.h"

/**
 * @brief 一键启动链路的低频ASCII诊断器。
 *
 * 只观察状态，不参与按键、底盘或机械臂控制。输出使用独立串口，
 * 禁止发送传感器原始帧，确保115200监视器中始终是可读文本。
 */
class StartupDiagnostics
{
public:
    StartupDiagnostics(
        HardwareSerial &serial,
        uint32_t buttonPin,
        MissionController &mission,
        RouteExecutor &route,
        ChassisControl &chassis,
        MaixProRingAlignment &alignment,
        MechanismTaskExecutor &mechanism);

    void begin(uint32_t baud);
    void noteButtonClick();
    void noteStartZoneSelection(
        StartZone zone,
        bool receivedFromScreen);
    void update(
        uint32_t intervalMs,
        bool outputAllowed = true);

private:
    HardwareSerial &_serial;
    uint32_t _buttonPin;
    MissionController &_mission;
    RouteExecutor &_route;
    ChassisControl &_chassis;
    MaixProRingAlignment &_alignment;
    MechanismTaskExecutor &_mechanism;

    uint32_t _clickCount = 0;
    uint32_t _lastReportMs = 0;
    int _lastButtonLevel = HIGH;
    MissionController::State _lastMissionState =
        static_cast<MissionController::State>(0xFF);

    void printSnapshot(const char *reason);
    void printServoState(
        const char *label,
        const MechanismTaskExecutor::ServoDebugState &state);
    void printGraspState(
        const MechanismTaskExecutor::GraspDebugState &state);
    static const char *missionStateName(MissionController::State state);
    static const char *asyncResultName(AsyncResult result);
    static const char *chassisStateName(ChassisControl::State state);
};
