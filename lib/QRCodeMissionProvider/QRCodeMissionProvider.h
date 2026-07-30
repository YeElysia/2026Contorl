#pragma once

#include <Arduino.h>

#include "MissionPorts.h"

/**
 * @brief GM75扫码器的非阻塞任务数据提供者。
 *
 * 通信行为参考new_project/lib/QRScan：
 * - 串口逐字节读取，不阻塞主循环；
 * - CR结束一帧，同时兼容CRLF；
 * - 超长帧整帧丢弃，等待下一个CR重新同步；
 * - 有效帧解析为颜色和圆环位置。
 *
 * 帧格式与new_project一致：
 * 134+123+314+231，共15字节、4组三位数：
 * 第一批颜色 + 第一批位置 + 第二批颜色 + 第二批粗加工位置。
 */
class QRCodeMissionProvider : public IMissionDataProvider
{
public:
    QRCodeMissionProvider(
        HardwareSerial &serial,
        uint32_t baud,
        uint32_t timeoutMs);

    void begin();

    void start() override;
    void update() override;
    AsyncResult result() const override;
    const MissionPlan &plan() const override;
    const char *rawText() const override;

private:
    static constexpr uint8_t MAX_FRAME_LENGTH = 15;

    HardwareSerial &_serial;
    uint32_t _baud;
    uint32_t _timeoutMs;

    char _frame[MAX_FRAME_LENGTH + 1] = {};
    uint8_t _frameLength = 0;
    MissionPlan _plan = {};
    bool _discarding = false;
    uint32_t _startedMs = 0;
    AsyncResult _result = AsyncResult::Idle;

    void resetFrame();
    bool parseFrame();
    bool parseUniqueTriplet(
        uint8_t frameOffset,
        uint8_t minimum,
        uint8_t maximum,
        uint8_t output[MATERIALS_PER_BATCH]);
    bool calculateSecondBatchStoragePositions();
};
