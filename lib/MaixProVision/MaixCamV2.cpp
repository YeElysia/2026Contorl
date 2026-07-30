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
    _detectionReady = false;
    _ackReady = false;
}

void MaixCamV2::poll()
{
    while (_serial.available() > 0)
    {
        const int value = _serial.read();
        if (value < 0)
            break;

        if (_buffered == BUFFER_SIZE)
            discard(1);
        _buffer[_buffered++] = static_cast<uint8_t>(value);
        consume();
    }
}

uint8_t MaixCamV2::setTarget(uint8_t mode, uint8_t selector)
{
    const uint8_t payload[] = {mode, selector};
    return sendFrame(CMD_SET_TARGET, payload, sizeof(payload));
}

uint8_t MaixCamV2::ping()
{
    return sendFrame(CMD_PING, nullptr, 0);
}

uint8_t MaixCamV2::reset()
{
    return sendFrame(CMD_RESET, nullptr, 0);
}

bool MaixCamV2::takeDetection(Detection &result)
{
    if (!_detectionReady)
        return false;

    result = _detection;
    _detectionReady = false;
    return true;
}

bool MaixCamV2::takeAck(
    uint8_t &command,
    uint8_t &result,
    uint8_t &mode,
    uint8_t &selector)
{
    if (!_ackReady)
        return false;

    command = _ack[0];
    result = _ack[1];
    mode = _ack[2];
    selector = _ack[3];
    _ackReady = false;
    return true;
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

void MaixCamV2::consume()
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
        if (_buffered < 9)
            return;

        const uint8_t length = _buffer[5];
        const size_t frameLength = 9U + length;
        if (_buffered < frameLength)
            return;

        const size_t payloadEnd = 6U + length;
        const bool valid =
            _buffer[2] == VERSION &&
            _buffer[payloadEnd + 1] == 0x0D &&
            _buffer[payloadEnd + 2] == 0x0A &&
            crc8(_buffer + 2, 4U + length) ==
                _buffer[payloadEnd];

        if (!valid)
        {
            discard(1);
            continue;
        }

        handleFrame(_buffer[3], _buffer + 6, length);
        discard(frameLength);
    }
}

void MaixCamV2::handleFrame(
    uint8_t type,
    const uint8_t *payload,
    uint8_t length)
{
    if (type == 0x80 && length == 4)
    {
        memcpy(_ack, payload, sizeof(_ack));
        _ackReady = true;
        return;
    }

    if (type == 0x81 && length == 14)
    {
        _detection.mode = payload[0];
        _detection.targetId = payload[1];
        _detection.found = payload[2] == 1;
        _detection.dx =
            static_cast<int16_t>(readU16(payload + 3));
        _detection.dy =
            static_cast<int16_t>(readU16(payload + 5));
        _detection.cx = readU16(payload + 7);
        _detection.cy = readU16(payload + 9);
        _detection.size = readU16(payload + 11);
        _detection.quality = payload[13];
        _detectionReady = true;
    }
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
