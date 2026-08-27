#include "ChassisEmm42TtlFeedback.h"

#include <stdio.h>
#include <string.h>

namespace
{
    constexpr uint8_t STATUS_FUNCTION = 0x3A;
    constexpr uint8_t POSITION_FUNCTION = 0x36;
    constexpr uint8_t FRAME_END = 0x6B;
    // 8次查询覆盖四个电机的状态和位置，约25Hz/电机。
    constexpr uint32_t QUERY_GAP_MS = 5;
    constexpr uint32_t RESPONSE_TIMEOUT_MS = 20;
    constexpr uint8_t MAX_CONSECUTIVE_TIMEOUTS_PER_MOTOR = 3;
} // namespace

ChassisEmm42TtlFeedback::ChassisEmm42TtlFeedback(
    HardwareSerial &serial,
    const uint8_t (&addresses)[MOTOR_COUNT],
    uint32_t baud)
    : _serial(serial), _baud(baud)
{
    memcpy(_addresses, addresses, sizeof(_addresses));
}

void ChassisEmm42TtlFeedback::begin()
{
    _started = false;
    _fault[0] = '\0';
    _waiting = false;
    _rxLength = 0;
    memset(_consecutiveTimeouts, 0, sizeof(_consecutiveTimeouts));
    _motorIndex = 0;
    _query = Query::Status;

    for (uint8_t i = 0; i < MOTOR_COUNT; ++i)
    {
        _motors[i] = {};
        if (_addresses[i] == 0)
        {
            setFault("EMM42 TTL address cannot be 0");
            return;
        }
        for (uint8_t j = 0; j < i; ++j)
        {
            if (_addresses[i] == _addresses[j])
            {
                setFault("EMM42 TTL addresses must be unique");
                return;
            }
        }
    }

    _serial.begin(_baud);
    while (_serial.available())
        _serial.read();

    _started = true;
    _lastQueryCompletedMs = millis();
}

void ChassisEmm42TtlFeedback::update()
{
    if (!_started || !healthy())
        return;

    while (_serial.available())
        consume(static_cast<uint8_t>(_serial.read()));

    const uint32_t now = millis();
    if (_waiting)
    {
        if (now - _querySentMs > RESPONSE_TIMEOUT_MS)
            finishQuery(false);
        return;
    }

    if (now - _lastQueryCompletedMs >= QUERY_GAP_MS)
        sendQuery();
}

void ChassisEmm42TtlFeedback::clearFault()
{
    if (!_started)
    {
        begin();
        return;
    }

    _fault[0] = '\0';
    memset(_consecutiveTimeouts, 0, sizeof(_consecutiveTimeouts));
    _waiting = false;
    _rxLength = 0;
    _lastQueryCompletedMs = millis();
}

bool ChassisEmm42TtlFeedback::ready() const
{
    if (!_started || !healthy())
        return false;

    for (const auto &feedback : _motors)
    {
        if (!feedback.statusValid || !feedback.positionValid)
            return false;
    }
    return true;
}

bool ChassisEmm42TtlFeedback::healthy() const
{
    return _fault[0] == '\0';
}

const char *ChassisEmm42TtlFeedback::faultMessage() const
{
    return _fault;
}

const ChassisEmm42TtlFeedback::MotorFeedback &
ChassisEmm42TtlFeedback::motor(uint8_t index) const
{
    static const MotorFeedback invalid = {};
    return index < MOTOR_COUNT ? _motors[index] : invalid;
}

void ChassisEmm42TtlFeedback::sendQuery()
{
    const uint8_t function =
        _query == Query::Status
            ? STATUS_FUNCTION
            : POSITION_FUNCTION;
    const uint8_t command[3] = {
        _addresses[_motorIndex],
        function,
        FRAME_END};

    _rxLength = 0;
    _serial.write(command, sizeof(command));
    _querySentMs = millis();
    _waiting = true;
}

void ChassisEmm42TtlFeedback::consume(uint8_t data)
{
    if (!_waiting)
        return;

    const uint8_t expectedAddress = _addresses[_motorIndex];
    if (_rxLength == 0 && data != expectedAddress)
        return;

    if (_rxLength < sizeof(_rx))
        _rx[_rxLength++] = data;

    const uint8_t expectedLength =
        _query == Query::Status ? 4 : 8;
    if (_rxLength < expectedLength)
        return;

    if (parseResponse())
    {
        finishQuery(true);
        return;
    }

    /*
     * 总线上若混入前一条命令的迟到响应，保留最后一个可能的帧头，
     * 其余数据丢弃，下一字节继续拼帧。
     */
    const bool lastMayBeHeader =
        _rx[expectedLength - 1] == expectedAddress;
    _rx[0] = expectedAddress;
    _rxLength = lastMayBeHeader ? 1 : 0;
}

bool ChassisEmm42TtlFeedback::parseResponse()
{
    MotorFeedback &feedback = _motors[_motorIndex];
    const uint8_t expectedAddress = _addresses[_motorIndex];
    if (_rx[0] != expectedAddress)
        return false;

    if (_query == Query::Status)
    {
        if (_rx[1] != STATUS_FUNCTION || _rx[3] != FRAME_END)
            return false;

        const uint8_t flags = _rx[2];
        feedback.enabled = (flags & 0x01U) != 0;
        feedback.inPosition = (flags & 0x02U) != 0;
        feedback.stalled = (flags & 0x04U) != 0;
        feedback.stallProtected = (flags & 0x08U) != 0;
        feedback.statusValid = true;
        feedback.lastStatusMs = millis();

        if (!feedback.enabled ||
            feedback.stalled ||
            feedback.stallProtected)
        {
            char message[64] = {};
            snprintf(
                message,
                sizeof(message),
                !feedback.enabled
                    ? "EMM42 ID %u disabled"
                    : "EMM42 ID %u stall/protection",
                static_cast<unsigned>(expectedAddress));
            setFault(message);
        }
        return true;
    }

    if (_rx[1] != POSITION_FUNCTION || _rx[7] != FRAME_END)
        return false;

    const uint32_t magnitude =
        (static_cast<uint32_t>(_rx[3]) << 24) |
        (static_cast<uint32_t>(_rx[4]) << 16) |
        (static_cast<uint32_t>(_rx[5]) << 8) |
        static_cast<uint32_t>(_rx[6]);
    feedback.encoderPosition =
        _rx[2] == 0
            ? static_cast<int64_t>(magnitude)
            : -static_cast<int64_t>(magnitude);
    feedback.positionValid = true;
    feedback.lastPositionMs = millis();
    return true;
}

void ChassisEmm42TtlFeedback::finishQuery(bool valid)
{
    _waiting = false;
    _rxLength = 0;
    _lastQueryCompletedMs = millis();
    const uint8_t queryIndex =
        _query == Query::Status ? 0 : 1;

    if (valid)
    {
        _consecutiveTimeouts[_motorIndex][queryIndex] = 0;
    }
    else if (++_consecutiveTimeouts[_motorIndex][queryIndex] >=
             MAX_CONSECUTIVE_TIMEOUTS_PER_MOTOR)
    {
        char message[64] = {};
        snprintf(
            message,
            sizeof(message),
            "EMM42 TTL ID %u no response",
            static_cast<unsigned>(_addresses[_motorIndex]));
        setFault(message);
        return;
    }

    if (_query == Query::Status)
    {
        _query = Query::Position;
    }
    else
    {
        _query = Query::Status;
        _motorIndex = (_motorIndex + 1U) % MOTOR_COUNT;
    }
}

void ChassisEmm42TtlFeedback::setFault(const char *message)
{
    strncpy(_fault, message, sizeof(_fault) - 1);
    _fault[sizeof(_fault) - 1] = '\0';
    _waiting = false;
}
