#include "MaixCamV2.h"

#include <string.h>

namespace maixcam
{
MaixCamV2::MaixCamV2(HardwareSerial &serial)
    : _serial(serial)
{
}

void MaixCamV2::begin(uint32_t baudrate)
{
    _serial.begin(baudrate);
    _buffered = 0;
}

FrameType MaixCamV2::poll(Ack &ack, Detection &detection)
{
    for (;;)
    {
        const FrameType type = parse(ack, detection);
        if (type != FrameType::None)
            return type;

        if (_serial.available() <= 0)
            return FrameType::None;
        const int value = _serial.read();
        if (value < 0)
            return FrameType::None;

        if (_buffered == BUFFER_SIZE)
            discard(1);
        _buffer[_buffered++] = static_cast<uint8_t>(value);
    }
}

uint8_t MaixCamV2::setTarget(uint8_t mode, uint8_t selector)
{
    const uint8_t payload[] = {mode, selector};
    return sendFrame(CMD_SET_TARGET, payload, sizeof(payload));
}

uint8_t MaixCamV2::reset()
{
    return sendFrame(CMD_RESET, nullptr, 0);
}

uint8_t MaixCamV2::sendFrame(
    uint8_t type,
    const uint8_t *payload,
    uint8_t length)
{
    const uint8_t sequence = ++_sequence;
    uint8_t body[4 + 255] = {};
    body[0] = VERSION;
    body[1] = type;
    body[2] = sequence;
    body[3] = length;
    if (length > 0 && payload != nullptr)
        memcpy(body + 4, payload, length);

    const uint8_t header[] = {0xAA, 0x55};
    const uint8_t tail[] = {0x0D, 0x0A};
    const uint8_t checksum = crc8(body, 4U + length);

    _serial.write(header, sizeof(header));
    _serial.write(body, 4U + length);
    _serial.write(checksum);
    _serial.write(tail, sizeof(tail));
    return sequence;
}

FrameType MaixCamV2::parse(Ack &ack, Detection &detection)
{
    while (_buffered >= 2)
    {
        size_t start = 0;
        while (start + 1 < _buffered &&
               !(_buffer[start] == 0xAA &&
                 _buffer[start + 1] == 0x55))
        {
            ++start;
        }
        if (start > 0)
            discard(start);
        if (_buffered < 6)
            return FrameType::None;

        const uint8_t type = _buffer[3];
        const uint8_t length = _buffer[5];
        // 先按类型校验长度，损坏的LEN不会让解析器空等后续多帧。
        const bool lengthValid =
            type == TYPE_ACK         ? length == ACK_LENGTH
            : type == TYPE_DETECTION ? length == DETECTION_LENGTH
                                     : length <= OTHER_MAX_LENGTH;
        if (_buffer[2] != VERSION || !lengthValid)
        {
            discard(1);
            continue;
        }

        const size_t frameLength = 9U + length;
        if (_buffered < frameLength)
            return FrameType::None;

        const size_t payloadEnd = 6U + length;
        const bool valid =
            _buffer[payloadEnd + 1] == 0x0D &&
            _buffer[payloadEnd + 2] == 0x0A &&
            crc8(_buffer + 2, 4U + length) ==
                _buffer[payloadEnd];
        if (!valid)
        {
            discard(1);
            continue;
        }

        const uint8_t *payload = _buffer + 6;
        FrameType parsed = FrameType::None;
        if (type == TYPE_ACK)
        {
            ack.sequence = _buffer[4];
            ack.command = payload[0];
            ack.result = payload[1];
            ack.mode = payload[2];
            ack.selector = payload[3];
            parsed = FrameType::Ack;
        }
        else if (type == TYPE_DETECTION)
        {
            detection.mode = payload[0];
            detection.targetId = payload[1];
            detection.found = payload[2] == 1;
            detection.dx =
                static_cast<int16_t>(readU16(payload + 3));
            detection.dy =
                static_cast<int16_t>(readU16(payload + 5));
            detection.cx = readU16(payload + 7);
            detection.cy = readU16(payload + 9);
            detection.size = readU16(payload + 11);
            detection.quality = payload[13];
            parsed = FrameType::Detection;
        }

        discard(frameLength);
        if (parsed != FrameType::None)
            return parsed;
    }
    return FrameType::None;
}

void MaixCamV2::discard(size_t count)
{
    if (count >= _buffered)
    {
        _buffered = 0;
        return;
    }

    memmove(_buffer, _buffer + count, _buffered - count);
    _buffered -= count;
}

uint8_t MaixCamV2::crc8(const uint8_t *data, size_t length)
{
    uint8_t value = 0;
    for (size_t i = 0; i < length; ++i)
    {
        value ^= data[i];
        for (uint8_t bit = 0; bit < 8; ++bit)
        {
            value = (value & 0x80)
                        ? static_cast<uint8_t>((value << 1) ^ 0x07)
                        : static_cast<uint8_t>(value << 1);
        }
    }
    return value;
}

uint16_t MaixCamV2::readU16(const uint8_t *data)
{
    return static_cast<uint16_t>(data[0]) |
           (static_cast<uint16_t>(data[1]) << 8);
}
} // namespace maixcam
