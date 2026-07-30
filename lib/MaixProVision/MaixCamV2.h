#pragma once

#include <Arduino.h>

/**
 * @brief MaixCAM/MaixPro V2串口协议。
 *
 * 协议与new_project/lib/MaixCam保持一致：
 * AA 55 + VERSION TYPE SEQ LEN PAYLOAD + CRC8 + 0D 0A。
 */
namespace maixcam
{
constexpr uint8_t VERSION = 0x01;
constexpr uint8_t MODE_GRAB = 0xCC;
constexpr uint8_t MODE_IDLE = 0xFF;
constexpr uint8_t CMD_SET_TARGET = 0x10;
constexpr uint8_t CMD_PING = 0x11;
constexpr uint8_t CMD_RESET = 0x13;

struct Detection
{
    uint8_t mode = MODE_IDLE;
    uint8_t targetId = 0;
    bool found = false;
    int16_t dx = 0;
    int16_t dy = 0;
    uint16_t cx = 0;
    uint16_t cy = 0;
    uint16_t size = 0;
    uint8_t quality = 0;
};

class MaixCamV2
{
public:
    explicit MaixCamV2(HardwareSerial &serial);

    void begin(uint32_t baudrate);
    void poll();
    uint8_t setTarget(uint8_t mode, uint8_t selector);
    uint8_t ping();
    uint8_t reset();

    bool takeDetection(Detection &result);
    bool takeAck(
        uint8_t &command,
        uint8_t &result,
        uint8_t &mode,
        uint8_t &selector);

private:
    static constexpr size_t BUFFER_SIZE = 280;

    HardwareSerial &_serial;
    uint8_t _buffer[BUFFER_SIZE] = {};
    size_t _buffered = 0;
    uint8_t _sequence = 0;

    Detection _detection = {};
    uint8_t _ack[4] = {};
    bool _detectionReady = false;
    bool _ackReady = false;

    uint8_t sendFrame(
        uint8_t type,
        const uint8_t *payload,
        uint8_t length);
    void consume();
    void handleFrame(
        uint8_t type,
        const uint8_t *payload,
        uint8_t length);
    void discard(size_t count);

    static uint8_t crc8(const uint8_t *data, size_t length);
    static uint16_t readU16(const uint8_t *data);
};
} // namespace maixcam
