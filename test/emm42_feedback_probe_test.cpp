#include <cassert>
#include <cstdint>

#include "emm42_feedback_probe.h"

int main()
{
    const uint8_t valid[] = {7, 0x3A, 0x03, 0x6B};
    const uint8_t wrongId[] = {6, 0x3A, 0x03, 0x6B};
    const uint8_t wrongFunction[] = {7, 0x36, 0x03, 0x6B};
    const uint8_t truncated[] = {7, 0x3A, 0x03};

    assert(emm42_probe::isValidStatusFrame(valid, sizeof(valid), 7));
    assert(!emm42_probe::isValidStatusFrame(wrongId, sizeof(wrongId), 7));
    assert(!emm42_probe::isValidStatusFrame(
        wrongFunction,
        sizeof(wrongFunction),
        7));
    assert(!emm42_probe::isValidStatusFrame(
        truncated,
        sizeof(truncated),
        7));
}
