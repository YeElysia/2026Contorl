#pragma once

#include <stddef.h>
#include <stdint.h>

namespace emm42_probe
{
inline bool isValidStatusFrame(
    const uint8_t *data,
    size_t length,
    uint8_t expectedId)
{
    return data != nullptr &&
           length == 4 &&
           data[0] == expectedId &&
           data[1] == 0x3A &&
           data[3] == 0x6B;
}
} // namespace emm42_probe
