#include <Arduino.h>
#include <OneButton.h>
#include <stdio.h>
#include <string.h>

#include "GraspVisionPorts.h"
#include "MechanismTaskExecutor.h"
#include "debug_config.h"
#include "mechanism_config.h"
#include "mission_config.h"

namespace
{
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
};

HardwareSerial serialStepper(
    mechanism_config::STEPPER_RX_PIN,
    mechanism_config::STEPPER_TX_PIN);
HardwareSerial serialBase(
    mechanism_config::BASE_RX_PIN,
    mechanism_config::BASE_TX_PIN);
HardwareSerial serialServo(
    mechanism_config::SERVO_RX_PIN,
    mechanism_config::SERVO_TX_PIN);
HardwareSerial serialDebug(
    debug_config::RX_PIN,
    debug_config::TX_PIN);

IdleVision idleVision;
IdleForwardPositioner idleForward;
MechanismTaskExecutor mechanism(
    serialStepper,
    serialBase,
    serialServo,
    idleVision,
    idleForward);
OneButton startButton(
    mission_config::START_BUTTON_PIN,
    true,
    true);

uint16_t candidateSpeed[4] = {
    mechanism_config::BASE_SPEED,
    mechanism_config::TRAY_TO_ROUGH_RING_BASE_SPEED,
    mechanism_config::TRAY_TO_ROUGH_RING_BASE_SPEED,
    mechanism_config::TRAY_TO_ROUGH_RING_BASE_SPEED};
uint8_t candidateAcceleration[4] = {
    mechanism_config::BASE_ACCELERATION,
    mechanism_config::TRAY_TO_ROUGH_RING_BASE_ACCELERATION,
    mechanism_config::TRAY_TO_ROUGH_RING_BASE_ACCELERATION,
    mechanism_config::TRAY_TO_ROUGH_RING_BASE_ACCELERATION};

char commandBuffer[64] = {};
uint8_t commandLength = 0;
uint8_t selectedRing = 1;
bool runRequested = false;
bool testActive = false;
bool readyReported = false;

void printParameters()
{
    serialDebug.println("PARAM ring speed accel");
    for (uint8_t ring = 1; ring <= 3; ++ring)
    {
        serialDebug.print(ring);
        serialDebug.print(' ');
        serialDebug.print(candidateSpeed[ring]);
        serialDebug.print(' ');
        serialDebug.println(candidateAcceleration[ring]);
    }
}

void printHelp()
{
    serialDebug.println("ROUGH_PLACEMENT_CAL ready");
    serialDebug.println("set <ring> <speed> <accel>");
    serialDebug.println("run <ring>  | run | show | help");
    serialDebug.println("PB9 runs selected ring");
    serialDebug.println("Place material in storage tray slot 3 before RUN");
    printParameters();
}

void requestRun(uint8_t ring)
{
    if (ring < 1 || ring > 3)
    {
        serialDebug.println("ERR ring must be 1..3");
        return;
    }
    if (testActive || mechanism.result() == AsyncResult::Running)
    {
        serialDebug.println("ERR mechanism busy");
        return;
    }
    selectedRing = ring;
    runRequested = true;
}

void processCommand(char *line)
{
    unsigned int ring = 0;
    unsigned int speed = 0;
    unsigned int acceleration = 0;

    if (sscanf(line, "set %u %u %u", &ring, &speed, &acceleration) == 3)
    {
        if (ring < 1 || ring > 3 ||
            speed == 0 || speed > UINT16_MAX ||
            acceleration == 0 || acceleration > UINT8_MAX)
        {
            serialDebug.println("ERR range: ring=1..3 speed=1..65535 accel=1..255");
            return;
        }
        candidateSpeed[ring] = static_cast<uint16_t>(speed);
        candidateAcceleration[ring] =
            static_cast<uint8_t>(acceleration);
        selectedRing = static_cast<uint8_t>(ring);
        printParameters();
        return;
    }

    if (sscanf(line, "run %u", &ring) == 1)
    {
        requestRun(static_cast<uint8_t>(ring));
        return;
    }
    if (strcmp(line, "run") == 0)
    {
        requestRun(selectedRing);
        return;
    }
    if (strcmp(line, "show") == 0)
    {
        printParameters();
        return;
    }
    if (strcmp(line, "help") == 0)
    {
        printHelp();
        return;
    }
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

void onButtonClick()
{
    requestRun(selectedRing);
}
} // namespace

void setup()
{
    delay(2000);
    serialDebug.begin(debug_config::BAUD);
    serialDebug.println("ROUGH_PLACEMENT_CAL boot");
    startButton.reset();
    startButton.attachClick(onButtonClick);
    // 标定期间固定使用储料盘3号槽，后续各轮测试不再旋转料盘。
    mechanism.begin(3);
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
        if (!mechanism.startRoughPlacementCalibration(
                selectedRing,
                candidateSpeed[selectedRing],
                candidateAcceleration[selectedRing]))
        {
            serialDebug.println("ERR calibration action rejected");
        }
        else
        {
            testActive = true;
            serialDebug.print("RUN ring=");
            serialDebug.print(selectedRing);
            serialDebug.print(" speed=");
            serialDebug.print(candidateSpeed[selectedRing]);
            serialDebug.print(" accel=");
            serialDebug.println(candidateAcceleration[selectedRing]);
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
    const MechanismTaskExecutor::MotionDebugState &motion =
        mechanism.motionDebug();
    serialDebug.print("DONE ring=");
    serialDebug.print(selectedRing);
    serialDebug.print(" speed=");
    serialDebug.print(candidateSpeed[selectedRing]);
    serialDebug.print(" accel=");
    serialDebug.print(candidateAcceleration[selectedRing]);
    serialDebug.print(" base_ms=");
    if (motion.roughPlacementConfirmedMs >=
            motion.roughPlacementIssuedMs &&
        motion.roughPlacementIssuedMs != 0)
    {
        serialDebug.println(
            motion.roughPlacementConfirmedMs -
            motion.roughPlacementIssuedMs);
    }
    else
    {
        serialDebug.println("NA");
    }
}
