#pragma once

#include <Arduino.h>

/**
 * @brief MaixCAM/MaixPro V2串口协议。
 *
 * 协议与new_project/lib/MaixCam保持一致：
 * AA 55 + VERSION TYPE SEQ LEN PAYLOAD + CRC8 + 0D 0A。
 * 只有MaixVisionService直接使用本类。
 */
namespace maixcam
{
constexpr uint8_t VERSION = 0x01;
constexpr uint8_t MODE_GRAB = 0xCC;
constexpr uint8_t MODE_RING = 0xEE;
constexpr uint8_t MODE_IDLE = 0xFF;
constexpr uint8_t CMD_SET_TARGET = 0x10;
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

struct Ack
{
    // 相机ACK帧头的SEQ回显请求SEQ。
    uint8_t sequence = 0;
    uint8_t command = 0;
    uint8_t result = 0;
    uint8_t mode = MODE_IDLE;
    uint8_t selector = 0;
};

enum class FrameType : uint8_t
{
    None,
    Ack,
    Detection
};

class MaixCamV2
{
public:
    explicit MaixCamV2(HardwareSerial &serial);

    void begin(uint32_t baudrate);

    /**
     * @brief 读取串口直到解析出一帧ACK或检测帧。
     * @return 串口读空仍无完整帧时返回FrameType::None。
     */
    FrameType poll(Ack &ack, Detection &detection);
    uint8_t setTarget(uint8_t mode, uint8_t selector);
    uint8_t reset();

private:
    static constexpr size_t BUFFER_SIZE = 64;
    static constexpr uint8_t TYPE_ACK = 0x80;
    static constexpr uint8_t TYPE_DETECTION = 0x81;
    static constexpr uint8_t ACK_LENGTH = 4;
    static constexpr uint8_t DETECTION_LENGTH = 14;
    // 其他相机上报帧（状态、任务码）都很短，长度字节更大视为损坏。
    static constexpr uint8_t OTHER_MAX_LENGTH = 32;

    HardwareSerial &_serial;
    uint8_t _buffer[BUFFER_SIZE] = {};
    size_t _buffered = 0;
    uint8_t _sequence = 0;

    uint8_t sendFrame(
        uint8_t type,
        const uint8_t *payload,
        uint8_t length);
    FrameType parse(Ack &ack, Detection &detection);
    void discard(size_t count);

    static uint8_t crc8(const uint8_t *data, size_t length);
    static uint16_t readU16(const uint8_t *data);
};
} // namespace maixcam
