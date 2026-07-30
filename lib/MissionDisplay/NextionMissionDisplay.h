#pragma once

#include <Arduino.h>

#include "MissionController.h"
#include "MissionPorts.h"

/**
 * @brief 淘晶驰/Nextion比赛状态显示器。
 *
 * 显示器只读取任务状态，不会改变任务状态，也不直接访问底盘、
 * 视觉或机械臂。所有发送均为事件驱动：状态或二维码变化时才发送。
 */
class NextionMissionDisplay
{
public:
    NextionMissionDisplay(
        HardwareSerial &serial,
        MissionController &mission,
        IMissionDataProvider &missionData);

    void begin(uint32_t baud, uint32_t restartWaitMs);
    void update();

private:
    HardwareSerial &_serial;
    MissionController &_mission;
    IMissionDataProvider &_missionData;

    MissionController::State _lastState =
        static_cast<MissionController::State>(0xFF);
    char _lastQrText[16] = {};
    uint32_t _screenReadyMs = 0;
    bool _initialized = false;

    void updateState();
    void updateQrText();
    void showState(MissionController::State state);

    void sendCommand(const char *command);
    void setText(
        const char *component,
        const char *text);

    static const char *stateText(
        MissionController::State state);
};
