#include <Arduino.h>

#include "debug_config.h"
#include "emm42_feedback_probe.h"
#include "mechanism_config.h"

namespace
{
HardwareSerial serialStepper(
    mechanism_config::STEPPER_RX_PIN,
    mechanism_config::STEPPER_TX_PIN);
HardwareSerial serialBase(
    mechanism_config::BASE_RX_PIN,
    mechanism_config::BASE_TX_PIN);
HardwareSerial serialDebug(
    debug_config::RX_PIN,
    debug_config::TX_PIN);

struct ProbeTarget
{
    const char *name;
    HardwareSerial *serial;
    uint8_t id;
};

ProbeTarget targets[] = {
    {"lift", &serialStepper, mechanism_config::LIFT_STEPPER_ID},
    {"extension", &serialStepper, mechanism_config::EXTENSION_STEPPER_ID},
    {"base", &serialBase, mechanism_config::BASE_STEPPER_ID},
};

constexpr uint8_t STATUS_FUNCTION = 0x3A;
constexpr uint8_t FRAME_END = 0x6B;
constexpr uint32_t RESPONSE_TIMEOUT_MS = 100;
constexpr uint32_t PROBE_INTERVAL_MS = 500;

uint8_t targetIndex = 0;
uint32_t lastProbeMs = 0;

void printHexByte(uint8_t value)
{
    if (value < 0x10)
        serialDebug.print('0');
    serialDebug.print(value, HEX);
}

size_t readResponse(
    HardwareSerial &serial,
    uint8_t *buffer,
    size_t capacity)
{
    size_t length = 0;
    const uint32_t startedMs = millis();

    while (millis() - startedMs < RESPONSE_TIMEOUT_MS)
    {
        while (serial.available() > 0)
        {
            const int value = serial.read();
            if (value >= 0 && length < capacity)
                buffer[length++] = static_cast<uint8_t>(value);
        }

        if (length >= 4 && buffer[length - 1] == FRAME_END)
            break;
    }

    return length;
}

void probe(const ProbeTarget &target)
{
    HardwareSerial &serial = *target.serial;
    while (serial.available() > 0)
        serial.read();

    const uint8_t request[] = {
        target.id,
        STATUS_FUNCTION,
        FRAME_END};

    serialDebug.print("TX ");
    serialDebug.print(target.name);
    serialDebug.print(" id=");
    serialDebug.print(target.id);
    serialDebug.print(": ");
    for (uint8_t value : request)
    {
        printHexByte(value);
        serialDebug.print(' ');
    }
    serialDebug.println();

    serial.write(request, sizeof(request));
    serial.flush();

    uint8_t response[32] = {};
    const size_t responseLength =
        readResponse(serial, response, sizeof(response));

    serialDebug.print("RX ");
    serialDebug.print(target.name);
    serialDebug.print(" id=");
    serialDebug.print(target.id);
    serialDebug.print(": ");

    if (responseLength == 0)
    {
        serialDebug.println("TIMEOUT");
        return;
    }

    for (size_t i = 0; i < responseLength; ++i)
    {
        printHexByte(response[i]);
        serialDebug.print(' ');
    }

    serialDebug.println(
        emm42_probe::isValidStatusFrame(
            response,
            responseLength,
            target.id)
            ? "VALID"
            : "INVALID");
}
} // namespace

void setup()
{
    delay(2000);
    serialDebug.begin(debug_config::BAUD);
    serialStepper.begin(mechanism_config::BUS_BAUD);
    serialBase.begin(mechanism_config::BUS_BAUD);

    serialDebug.println("EMM42 mechanism TTL feedback probe");
    serialDebug.println("Read-only test: no motion commands are sent.");
}

void loop()
{
    const uint32_t now = millis();
    if (now - lastProbeMs < PROBE_INTERVAL_MS)
        return;

    lastProbeMs = now;
    probe(targets[targetIndex]);
    targetIndex = (targetIndex + 1) %
                  (sizeof(targets) / sizeof(targets[0]));
}
