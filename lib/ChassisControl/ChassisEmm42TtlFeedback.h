#pragma once

#include <Arduino.h>

/**
 * @brief EMM42 V5 TTL底盘电机反馈轮询器。
 *
 * 四个地址的顺序必须是左前、右前、左后、右后，电机共用一条
 * 专用TTL总线。控制脉冲仍由ChassisControl的
 * STEP/DIR输出；本类轮询0x3A状态和0x36实际位置，用于发现堵转、
 * 堵转保护和通信中断。未把本对象传给ChassisControl时，不会初始化
 * 或占用任何串口。
 */
class ChassisEmm42TtlFeedback
{
public:
    static constexpr uint8_t MOTOR_COUNT = 4;

    struct MotorFeedback
    {
        bool statusValid = false;
        bool positionValid = false;
        bool enabled = false;
        bool inPosition = false;
        bool stalled = false;
        bool stallProtected = false;
        int64_t encoderPosition = 0;
        uint32_t lastStatusMs = 0;
        uint32_t lastPositionMs = 0;
    };

    ChassisEmm42TtlFeedback(
        HardwareSerial &serial,
        const uint8_t (&addresses)[MOTOR_COUNT],
        uint32_t baud = 115200);

    void begin();
    void update();
    void clearFault();

    bool ready() const;
    bool healthy() const;
    const char *faultMessage() const;
    const MotorFeedback &motor(uint8_t index) const;

private:
    enum class Query : uint8_t
    {
        Status,
        Position
    };

    HardwareSerial &_serial;
    uint8_t _addresses[MOTOR_COUNT] = {};
    uint32_t _baud;
    MotorFeedback _motors[MOTOR_COUNT] = {};

    Query _query = Query::Status;
    uint8_t _motorIndex = 0;
    bool _waiting = false;
    uint32_t _querySentMs = 0;
    uint32_t _lastQueryCompletedMs = 0;
    uint8_t _rx[8] = {};
    uint8_t _rxLength = 0;
    uint8_t _consecutiveTimeouts[MOTOR_COUNT][2] = {};
    bool _started = false;
    char _fault[64] = {};

    void sendQuery();
    void consume(uint8_t data);
    bool parseResponse();
    void finishQuery(bool valid);
    void setFault(const char *message);
};
