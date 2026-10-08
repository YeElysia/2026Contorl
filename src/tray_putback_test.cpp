#include <Arduino.h>
#include <OneButton.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "GraspVisionPorts.h"
#include "MechanismTaskExecutor.h"
#include "debug_config.h"
#include "mechanism_config.h"
#include "mission_config.h"

namespace
{
using namespace mechanism_config;

class IdleVision : public IGraspVisionProvider
{
public:
    void begin() override {}
    bool startTracking(uint8_t) override { return false; }
    void update() override {}
    bool takeObservation(GraspObservation &) override { return false; }
    void stop() override {}
    bool faulted() const override { return false; }
};

class IdleForwardPositioner : public IGraspForwardPositioner
{
public:
    bool moveBodyRelative(float, float) override { return false; }
    bool busy() const override { return false; }
    bool faulted() const override { return false; }
    void stop() override {}
    void release() override {}
};

HardwareSerial serialStepper(STEPPER_RX_PIN, STEPPER_TX_PIN);
HardwareSerial serialBase(BASE_RX_PIN, BASE_TX_PIN);
HardwareSerial serialServo(SERVO_RX_PIN, SERVO_TX_PIN);
HardwareSerial serialDebug(debug_config::RX_PIN, debug_config::TX_PIN);

IdleVision idleVision;
IdleForwardPositioner idleForward;
MechanismTaskExecutor mechanism(
    serialStepper, serialBase, serialServo, idleVision, idleForward);
OneButton startButton(mission_config::START_BUTTON_PIN, true, true);

// 防止串口输入手误（如多打一位数）把机构送出实车行程；按实车用过的点位留出余量。
constexpr float ANGLE_MIN = -180.0F, ANGLE_MAX = 180.0F;
constexpr float BASE_MIN = 0.0F, BASE_MAX = 3600.0F;
constexpr float EXT_MIN = 700.0F, EXT_MAX = 1800.0F;
constexpr float LIFT_MIN = 0.0F, LIFT_MAX = 1300.0F;

float slotAngle = TRAY_SLOT_ANGLE[1];
float baseTarget = BASE_TRAY_TRANSFER;
float extensionTarget = EXTENSION_TRAY_TRANSFER;
float liftDownTarget = LIFT_TRAY_TRANSFER;

char commandBuffer[64] = {};
uint8_t commandLength = 0;
bool runRequested = false;
bool testActive = false;
bool readyReported = false;
uint32_t runStartedMs = 0;

void printParameters()
{
    serialDebug.print("PARAM angle=");
    serialDebug.print(slotAngle, 1);
    serialDebug.print(" base=");
    serialDebug.print(baseTarget, 1);
    serialDebug.print(" ext=");
    serialDebug.print(extensionTarget, 1);
    serialDebug.print(" lift=");
    serialDebug.println(liftDownTarget, 1);
}

void printHelp()
{
    serialDebug.println("TRAY_PUTBACK ready");
    serialDebug.println("slot <1..3>   tray angle from config slot");
    serialDebug.println("angle <deg>   tray servo angle override");
    serialDebug.println("base <v>      base target, 0.1 deg");
    serialDebug.println("ext <v>       extension target, 0.1 mm (smaller = more extended)");
    serialDebug.println("lift <v>      lift-down target, 0.1 mm (smaller = higher)");
    serialDebug.println("reset | show | run | help; PB9 = run");
    serialDebug.println("run: close gripper -> lift HOME -> base+EXT_HOME+tray -> ext -> lift down -> open -> lift HOME");
    serialDebug.println("Gripper is OPEN after a run; put material between the fingers before the next run.");
    printParameters();
}

bool parseValue(const char *text, float minimum, float maximum, float &out)
{
    char *end = nullptr;
    const float value = strtof(text, &end);
    if (end == text || *end != '\0' || !(value >= minimum && value <= maximum))
        return false;
    out = value;
    return true;
}

void requestRun()
{
    if (testActive || mechanism.result() == AsyncResult::Running)
    {
        serialDebug.println("ERR mechanism busy");
        return;
    }
    runRequested = true;
}

void setParameter(const char *name, const char *arg)
{
    float value = 0.0F;
    if (strcmp(name, "slot") == 0)
    {
        if (!parseValue(arg, 1.0F, 3.0F, value))
        {
            serialDebug.println("ERR slot must be 1..3");
            return;
        }
        slotAngle = TRAY_SLOT_ANGLE[static_cast<uint8_t>(value)];
    }
    else if (strcmp(name, "angle") == 0 && parseValue(arg, ANGLE_MIN, ANGLE_MAX, value))
        slotAngle = value;
    else if (strcmp(name, "base") == 0 && parseValue(arg, BASE_MIN, BASE_MAX, value))
        baseTarget = value;
    else if (strcmp(name, "ext") == 0 && parseValue(arg, EXT_MIN, EXT_MAX, value))
        extensionTarget = value;
    else if (strcmp(name, "lift") == 0 && parseValue(arg, LIFT_MIN, LIFT_MAX, value))
        liftDownTarget = value;
    else
    {
        serialDebug.println("ERR bad value or out of range (angle -180..180, base 0..3600, ext 700..1800, lift 0..1300)");
        return;
    }
    printParameters();
}

void processCommand(char *line)
{
    char *arg = strchr(line, ' ');
    if (arg != nullptr)
    {
        *arg++ = '\0';
        while (*arg == ' ')
            ++arg;
    }

    if (strcmp(line, "run") == 0 && arg == nullptr)
        requestRun();
    else if (strcmp(line, "show") == 0)
        printParameters();
    else if (strcmp(line, "help") == 0)
        printHelp();
    else if (strcmp(line, "reset") == 0)
    {
        slotAngle = TRAY_SLOT_ANGLE[1];
        baseTarget = BASE_TRAY_TRANSFER;
        extensionTarget = EXTENSION_TRAY_TRANSFER;
        liftDownTarget = LIFT_TRAY_TRANSFER;
        printParameters();
    }
    else if (arg != nullptr && *arg != '\0')
        setParameter(line, arg);
    else
        serialDebug.println("ERR unknown command; type help");
}

void pollCommands()
{
    while (serialDebug.available())
    {
        const char value = static_cast<char>(serialDebug.read());
        if (value == '\r' || value == '\n')
        {
            if (commandLength > 0)
            {
                commandBuffer[commandLength] = '\0';
                processCommand(commandBuffer);
                commandLength = 0;
            }
            continue;
        }
        if (commandLength + 1 < sizeof(commandBuffer))
            commandBuffer[commandLength++] = value;
    }
}
} // namespace

void setup()
{
    delay(2000);
    serialDebug.begin(debug_config::BAUD);
    serialDebug.println("TRAY_PUTBACK boot: initialization WILL MOVE the mechanism; keep clear.");
    startButton.reset();
    startButton.attachClick(requestRun);
    // 初始化后载物盘停在0号收纳位。
    mechanism.begin(0);
}

void loop()
{
    startButton.tick();
    pollCommands();
    mechanism.update();

    if (!readyReported && mechanism.ready() &&
        mechanism.result() != AsyncResult::Running)
    {
        readyReported = true;
        printHelp();
    }

    if (runRequested)
    {
        runRequested = false;
        if (!mechanism.startTrayPutBackTest(
                slotAngle, baseTarget, extensionTarget, liftDownTarget))
        {
            serialDebug.println("ERR action rejected (initializing or busy)");
        }
        else
        {
            testActive = true;
            runStartedMs = millis();
            serialDebug.print("RUN ");
            printParameters();
        }
    }

    if (!testActive)
        return;

    if (mechanism.result() == AsyncResult::Failed)
    {
        testActive = false;
        serialDebug.print("FAIL ");
        serialDebug.println(mechanism.faultMessage());
        return;
    }
    if (mechanism.result() != AsyncResult::Succeeded)
        return;

    testActive = false;
    serialDebug.print("DONE ms=");
    serialDebug.print(millis() - runStartedMs);
    serialDebug.print(' ');
    printParameters();
}
