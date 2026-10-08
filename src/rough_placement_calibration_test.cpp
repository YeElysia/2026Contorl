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
    void release() override {}
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

mechanism_config::RingPose candidatePose[4] = {
    mechanism_config::ROUGH_RING_POSES[0],
    mechanism_config::ROUGH_RING_POSES[1],
    mechanism_config::ROUGH_RING_POSES[2],
    mechanism_config::ROUGH_RING_POSES[3]};

char commandBuffer[64] = {};
uint8_t commandLength = 0;
uint8_t selectedRing = 1;
bool runRequested = false;
bool testActive = false;
bool readyReported = false;
bool runAllActive = false;
uint8_t targetRings[3] = {1, 2, 3};
uint8_t loadedSlotMask = 7;
uint8_t currentSlot = 1;
bool resetRequired = false;
bool commandOverflow = false;
const char *testState = "INITIALIZING";

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
    serialDebug.println("POSE ring base lift ext (0.1 unit)");
    for (uint8_t ring = 1; ring <= 3; ++ring)
    {
        serialDebug.print(ring);
        serialDebug.print(' ');
        serialDebug.print(candidatePose[ring].base, 0);
        serialDebug.print(' ');
        serialDebug.print(candidatePose[ring].lift, 0);
        serialDebug.print(' ');
        serialDebug.println(candidatePose[ring].extension, 0);
    }
}

void printHelp()
{
    serialDebug.println("Commands: set <ring> <speed> <accel> | pose <ring> <base> <lift> <ext> | setplan <s1> <s2> <s3> | load <mask> | run | cancel | status | help");
    serialDebug.println("setplan: tray slots 1..3 map to target rings; default is 1 2 3.");
    serialDebug.println("load: declaration only, bit0..2 = tray slots 1..3; default is 7.");
    serialDebug.println("run/PB9: place all three in sequence; use run <ring> for one ring.");
    serialDebug.println("cancel: stop current action; status: print current state and parameters.");
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
    if (!mechanism.ready() || resetRequired || runRequested)
    {
        serialDebug.println("REJECT not ready or reset required");
        return;
    }
    selectedRing = ring;
    runRequested = true;
}

bool selectLoadedSlot(uint8_t firstSlot)
{
    for (uint8_t slot = firstSlot; slot <= 3; ++slot)
    {
        if ((loadedSlotMask & (1U << (slot - 1))) == 0)
            continue;
        currentSlot = slot;
        selectedRing = targetRings[slot - 1];
        return true;
    }
    return false;
}

void requestBatch()
{
    if (!mechanism.ready() || resetRequired || testActive || runRequested ||
        mechanism.result() == AsyncResult::Running)
    {
        serialDebug.println("REJECT busy, not ready or reset required");
        return;
    }
    if (!selectLoadedSlot(1)) return;
    runAllActive = true;
    runRequested = true;
}

void printStatus()
{
    serialDebug.print("ROUGH_PLACEMENT state=");
    serialDebug.print(testActive ? "RUNNING" : "IDLE");
    serialDebug.print(" ring=");
    serialDebug.print(selectedRing);
    serialDebug.print(" result=");
    serialDebug.print(static_cast<uint8_t>(mechanism.result()));
    serialDebug.print(" phase=");
    serialDebug.println(mechanism.debugPhase());
    printParameters();
}

void processCommand(char *line)
{
    serialDebug.print("RX: ");
    serialDebug.println(line);

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

    long base = 0;
    long lift = 0;
    long extension = 0;
    if (sscanf(line, "pose %u %ld %ld %ld",
               &ring, &base, &lift, &extension) == 4)
    {
        if (ring < 1 || ring > 3 ||
            base < 0 || base > 3600 ||
            lift < 0 || lift > 1500 ||
            extension < 700 || extension > 1800)
        {
            serialDebug.println(
                "ERR range: ring=1..3 base=0..3600 lift=0..1500 ext=700..1800");
            return;
        }
        candidatePose[ring].base = static_cast<float>(base);
        candidatePose[ring].lift = static_cast<float>(lift);
        candidatePose[ring].extension = static_cast<float>(extension);
        selectedRing = static_cast<uint8_t>(ring);
        printParameters();
        return;
    }

    if (sscanf(line, "setplan %u %u %u", &ring, &speed, &acceleration) == 3)
    {
        if (ring < 1 || ring > 3 || speed < 1 || speed > 3 ||
            acceleration < 1 || acceleration > 3 ||
            ring == speed || ring == acceleration || speed == acceleration)
        {
            serialDebug.println("REJECT setplan: use three distinct ring numbers 1..3");
            return;
        }
        targetRings[0] = static_cast<uint8_t>(ring);
        targetRings[1] = static_cast<uint8_t>(speed);
        targetRings[2] = static_cast<uint8_t>(acceleration);
        printStatus();
        return;
    }

    if (sscanf(line, "load %u", &ring) == 1 && ring <= 7)
    {
        loadedSlotMask = static_cast<uint8_t>(ring);
        printStatus();
        return;
    }

    if (sscanf(line, "run %u", &ring) == 1)
    {
        runAllActive = false;
        requestRun(static_cast<uint8_t>(ring));
        return;
    }
    if (strcmp(line, "runall") == 0)
    {
        runAllActive = true;
        selectedRing = 1;
        runRequested = true;
        return;
    }
    if (strcmp(line, "run") == 0)
    {
        runAllActive = true;
        selectedRing = 1;
        runRequested = true;
        return;
    }
    if (strcmp(line, "cancel") == 0)
    {
        runAllActive = false;
        if (testActive || mechanism.result() == AsyncResult::Running)
            mechanism.cancel();
        testActive = false;
        serialDebug.println("CANCELLED");
        return;
    }
    if (strcmp(line, "status") == 0 || strcmp(line, "show") == 0)
    {
        printStatus();
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
    // 串口命令必须以回车或换行结束，不做超时自动提交。
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
        if (!mechanism.startRoughPlacementCalibration(
                selectedRing,
                candidateSpeed[selectedRing],
                candidateAcceleration[selectedRing],
                candidatePose[selectedRing],
                runAllActive ? selectedRing : 0))
        {
            runAllActive = false;
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
        runAllActive = false;
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
    serialDebug.print(" pose={");
    serialDebug.print(candidatePose[selectedRing].base, 0);
    serialDebug.print("F, ");
    serialDebug.print(candidatePose[selectedRing].lift, 0);
    serialDebug.print("F, ");
    serialDebug.print(candidatePose[selectedRing].extension, 0);
    serialDebug.print("F}");
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

    if (runAllActive)
    {
        if (selectedRing < 3)
        {
            ++selectedRing;
            runRequested = true;
        }
        else
        {
            runAllActive = false;
            serialDebug.println("RUNALL DONE");
        }
    }
}
