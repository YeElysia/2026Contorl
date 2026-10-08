#include <Arduino.h>
#include <OneButton.h>

#include "DebugLog.h"
#include "TTL_STEPPER.h"
#include "debug_config.h"
#include "mechanism_config.h"
#include "mission_config.h"

/*
 * 机械臂小幅度调用测试：不初始化底盘、相机、舵机和任务层。
 * 升降、伸缩共用USART2，基座使用独立串口；串口参数、电机参数、
 * 指令接口（runToNewPosition / setAngle）、状态轮询和到位判定
 * 均与MechanismTaskExecutor一致，只把行程缩小到几毫米/几度。
 *
 * 使用前让机械臂处于上电初始位（驱动器上电时的位置即绝对零点）：
 * 测试以 LIFT_HOME / EXTENSION_INITIAL / BASE_INITIAL 为基准偏移。
 * 按下PB9或向调试串口发送'n'开始；每轮之间空闲IDLE_GAP_MS，
 * 复现主程序“空闲一段时间后第一条升降指令”的调用方式。
 */

using namespace mechanism_config;

namespace
{
#ifdef SMALL_MOVE_FULL_RANGE
constexpr float LIFT_DELTA = LIFT_TURNTABLE - LIFT_HOME;             // 0.1 mm
constexpr float EXTENSION_DELTA = EXTENSION_HOME - EXTENSION_INITIAL; // 0.1 mm
constexpr float BASE_DELTA = BASE_HOME - BASE_INITIAL;               // 0.1 deg
#else
constexpr float LIFT_DELTA = 50.0F;      // 0.1 mm
constexpr float EXTENSION_DELTA = 50.0F; // 0.1 mm
constexpr float BASE_DELTA = 50.0F;      // 0.1 deg
#endif
constexpr uint8_t CYCLE_COUNT = 10;
constexpr uint32_t IDLE_GAP_MS = 4000;

HardwareSerial serialStepper(STEPPER_RX_PIN, STEPPER_TX_PIN);
HardwareSerial serialBase(BASE_RX_PIN, BASE_TX_PIN);
HardwareSerial serialDebug(debug_config::RX_PIN, debug_config::TX_PIN);

TTL_Protocol stepperProtocol(&serialStepper, BUS_BAUD);
TTL_Protocol baseProtocol(&serialBase, BUS_BAUD);
TTL_Stepper lift(LIFT_STEPPER_ID, &stepperProtocol);
TTL_Stepper extension(EXTENSION_STEPPER_ID, &stepperProtocol);
TTL_Stepper baseMotor(BASE_STEPPER_ID, &baseProtocol);

OneButton startButton(mission_config::START_BUTTON_PIN, true, true);

struct Axis
{
    const char *name;
    TTL_Stepper *motor;
    bool angle;
    float target;
    bool active;
    bool issued;
    bool done;
    bool faultLogged;
    uint8_t onPosCount;
    uint32_t startedMs;
    uint32_t lastPollMs;
    uint32_t queryStartedMs;
};

enum AxisMask : uint8_t
{
    MASK_LIFT = 1,
    MASK_EXT = 2,
    MASK_BASE = 4,
};

struct Stage
{
    const char *name;
    uint8_t mask;
    float lift;
    float extension;
    float base;
};

#if defined(SMALL_MOVE_ONLY_BASE)
constexpr uint8_t ARM_MASK = MASK_BASE;
#elif defined(SMALL_MOVE_ONLY_EXT)
constexpr uint8_t ARM_MASK = MASK_EXT;
#else
constexpr uint8_t ARM_MASK = MASK_EXT | MASK_BASE;
#endif

const Stage STAGES[] = {
    {"lift redundant home", MASK_LIFT, LIFT_HOME, 0, 0},
    {"lift down", MASK_LIFT, LIFT_HOME + LIFT_DELTA, 0, 0},
    {"base+extension out", ARM_MASK, 0,
     EXTENSION_INITIAL + EXTENSION_DELTA, BASE_INITIAL + BASE_DELTA},
    {"lift up home", MASK_LIFT, LIFT_HOME, 0, 0},
    {"base+extension back", ARM_MASK, 0,
     EXTENSION_INITIAL, BASE_INITIAL},
};
constexpr uint8_t STAGE_COUNT = sizeof(STAGES) / sizeof(STAGES[0]);

enum class RunState : uint8_t
{
    Idle,
    Running,
    Gap,
    Finished,
    Failed
};

Axis axes[3] = {
    {"lift", &lift, false, 0, false, false, false, false, 0, 0, 0, 0},
    {"extension", &extension, false, 0, false, false, false, false, 0, 0, 0, 0},
    {"base", &baseMotor, true, 0, false, false, false, false, 0, 0, 0, 0},
};

RunState state = RunState::Idle;
uint8_t stageIndex = 0;
uint8_t cycleIndex = 0;
uint32_t gapStartedMs = 0;
uint32_t stageStartedMs = 0;
uint32_t timeoutCount = 0;
uint32_t faultFlagCount = 0;

TTL_Stepper *sharedPollOwner = nullptr;
uint32_t lastSharedPollMs = 0;
bool liftCommanded = false;
float lastLiftTarget = 0;

bool probeDriver(TTL_Protocol &protocol, const char *name, uint8_t id)
{
    protocol.emptyCache();
    const uint8_t query[3] = {id, 0x3A, 0x6B};
    protocol.serial->write(query, sizeof(query));
    uint8_t rx[16] = {};
    uint8_t rxCount = 0;
    protocol.Emm_V5_Receive_Data(rx, &rxCount);
    char hex[48] = "NONE";
    for (uint8_t i = 0, pos = 0; i < rxCount && i < 12; ++i)
    {
        pos += snprintf(hex + pos, sizeof(hex) - pos, "%02X ", rx[i]);
    }
    LOGF_INFO("SmallMove", "precheck %s(id=%u) rx=%u: %s", name, id, rxCount, hex);
    return rxCount >= 4;
}

bool precheckDrivers()
{
    const bool liftOk = probeDriver(stepperProtocol, "lift", LIFT_STEPPER_ID);
    const bool extOk = probeDriver(stepperProtocol, "extension", EXTENSION_STEPPER_ID);
    const bool baseOk = probeDriver(baseProtocol, "base", BASE_STEPPER_ID);
    return liftOk && extOk && baseOk;
}

void startRequested()
{
    if (state == RunState::Idle || state == RunState::Finished ||
        state == RunState::Failed)
    {
        if (!precheckDrivers())
        {
            LOGF_ERROR("SmallMove",
                       "precheck FAILED: driver silent before any move, power-cycle drivers and retry");
            state = RunState::Failed;
            return;
        }
        stageIndex = 0;
        cycleIndex = 0;
        timeoutCount = 0;
        faultFlagCount = 0;
        liftCommanded = false;
        sharedPollOwner = nullptr;
        LOGF_INFO("SmallMove", "START cycles=%u gap=%lums",
                  CYCLE_COUNT, static_cast<unsigned long>(IDLE_GAP_MS));
        state = RunState::Gap;
        gapStartedMs = millis() - IDLE_GAP_MS;
    }
}

// 与MechanismTaskExecutor::pollStepperState相同的轮询策略。
bool pollState(Axis &axis, uint32_t now)
{
    TTL_Stepper &motor = *axis.motor;
    const bool sharedBus = &motor == &lift || &motor == &extension;

    if (sharedBus)
    {
        if (sharedPollOwner != nullptr && sharedPollOwner != &motor)
            return false;

        if (motor.Ask_State && axis.queryStartedMs != 0 &&
            now - axis.queryStartedMs >= STEPPER_QUERY_RESPONSE_TIMEOUT_MS)
        {
            motor.recDate_Clear();
            axis.queryStartedMs = 0;
            sharedPollOwner = nullptr;
            return false;
        }

        if (now - lastSharedPollMs < STEPPER_POLL_MS)
            return false;

        if (sharedPollOwner == nullptr)
            sharedPollOwner = &motor;

        lastSharedPollMs = now;
        const bool responsePending = motor.Ask_State;
        motor.state_update();
        if (!responsePending && motor.Ask_State)
            axis.queryStartedMs = now;

        if (!motor.Ask_State)
        {
            axis.queryStartedMs = 0;
            sharedPollOwner = nullptr;
        }
        return responsePending && !motor.Ask_State;
    }

    if (motor.Ask_State && axis.queryStartedMs != 0 &&
        now - axis.queryStartedMs >= STEPPER_QUERY_RESPONSE_TIMEOUT_MS)
    {
        motor.recDate_Clear();
        axis.queryStartedMs = 0;
        return false;
    }

    if (axis.lastPollMs != 0 && now - axis.lastPollMs < STEPPER_POLL_MS)
        return false;

    axis.lastPollMs = now;
    const bool responsePending = motor.Ask_State;
    motor.state_update();
    if (!responsePending && motor.Ask_State)
        axis.queryStartedMs = now;
    else if (!motor.Ask_State)
        axis.queryStartedMs = 0;
    return responsePending && !motor.Ask_State;
}

bool issueCommand(Axis &axis)
{
    TTL_Stepper &motor = *axis.motor;
    uint16_t velocity = motor.Speed;
    uint8_t acceleration = motor.Acceleration;
    bool accepted;

    LOGF_INFO("SmallMove", "cmd %s target_x10=%ld",
              axis.name, static_cast<long>(axis.target * 10.0F));

    if (axis.angle)
    {
        accepted = motor.setAngle(axis.target, velocity, acceleration);
    }
    else
    {
        if (&motor == &lift && liftCommanded &&
            axis.target < lastLiftTarget - 0.5F)
        {
            velocity = LIFT_UP_SPEED;
            acceleration = LIFT_UP_ACCELERATION;
        }
        accepted = motor.runToNewPosition(axis.target, velocity, acceleration);
    }

    if (&motor == &lift)
    {
        liftCommanded = true;
        lastLiftTarget = axis.target;
    }
    return accepted;
}

// 返回：0进行中，1完成，-1失败。
int8_t updateAxis(Axis &axis)
{
    const uint32_t now = millis();
    TTL_Stepper &motor = *axis.motor;

    if (!axis.issued)
    {
        if (!issueCommand(axis))
            return -1;
        axis.issued = true;
        axis.startedMs = now;
        return 0;
    }

    if (now - axis.startedMs >= STEPPER_TIMEOUT_MS)
    {
        ++timeoutCount;
        LOGF_ERROR("SmallMove", "%s timeout", axis.name);
        return -1;
    }

    if (!pollState(axis, now))
        return 0;

    if (motor.onPos_state)
    {
        const bool critical =
            &motor == &lift && fabsf(axis.target - LIFT_HOME) < 0.5F;
        if (critical)
        {
            if (axis.onPosCount < LIFT_CRITICAL_CONFIRM_COUNT)
                ++axis.onPosCount;
            if (axis.onPosCount < LIFT_CRITICAL_CONFIRM_COUNT)
                return 0;
        }
        LOGF_INFO("SmallMove", "%s done %lums", axis.name,
                  static_cast<unsigned long>(now - axis.startedMs));
        return 1;
    }

    axis.onPosCount = 0;
    if ((motor.locked_state || motor.loPro_state) && !axis.faultLogged)
    {
        axis.faultLogged = true;
        ++faultFlagCount;
        LOGF_WARNING("SmallMove", "%s stall/protection flag locked=%d protect=%d",
                     axis.name, motor.locked_state, motor.loPro_state);
    }
    return 0;
}

void beginStage()
{
    const Stage &stage = STAGES[stageIndex];
    axes[0].target = stage.lift;
    axes[1].target = stage.extension;
    axes[2].target = stage.base;
    const uint8_t masks[3] = {MASK_LIFT, MASK_EXT, MASK_BASE};
    for (uint8_t i = 0; i < 3; ++i)
    {
        Axis &axis = axes[i];
        axis.active = (stage.mask & masks[i]) != 0;
        axis.issued = false;
        axis.done = false;
        axis.faultLogged = false;
        axis.onPosCount = 0;
        axis.lastPollMs = 0;
        axis.queryStartedMs = 0;
    }
    stageStartedMs = millis();
    LOGF_INFO("SmallMove", "cycle %u/%u stage %u/%u: %s",
              cycleIndex + 1, CYCLE_COUNT, stageIndex + 1,
              STAGE_COUNT, stage.name);
    state = RunState::Running;
}

void fail(const char *reason)
{
    LOGF_ERROR("SmallMove", "FAILED cycle=%u stage=%u (%s): %s",
               cycleIndex + 1, stageIndex + 1, STAGES[stageIndex].name,
               reason);
    state = RunState::Failed;
}

void updateTest()
{
    const uint32_t now = millis();

    if (state == RunState::Gap)
    {
        if (now - gapStartedMs >= IDLE_GAP_MS)
            beginStage();
        return;
    }
    if (state != RunState::Running)
        return;

    bool allDone = true;
    for (Axis &axis : axes)
    {
        if (!axis.active || axis.done)
            continue;

        const int8_t result = updateAxis(axis);
        if (result < 0)
        {
            fail("command rejected or timeout");
            return;
        }
        axis.done = result > 0;
        if (!axis.done)
            allDone = false;
    }
    if (!allDone)
        return;

    LOGF_INFO("SmallMove", "stage done %lums",
              static_cast<unsigned long>(now - stageStartedMs));
    ++stageIndex;
    if (stageIndex < STAGE_COUNT)
    {
        beginStage();
        return;
    }

    stageIndex = 0;
    ++cycleIndex;
    if (cycleIndex >= CYCLE_COUNT)
    {
        LOGF_INFO("SmallMove",
                  "FINISHED cycles=%u timeouts=%lu stall_flags=%lu",
                  CYCLE_COUNT, static_cast<unsigned long>(timeoutCount),
                  static_cast<unsigned long>(faultFlagCount));
        state = RunState::Finished;
        return;
    }
    gapStartedMs = now;
    state = RunState::Gap;
}

void updateStatusLed()
{
    const uint32_t now = millis();
    bool on = false;
    if (state == RunState::Running || state == RunState::Gap)
        on = true;
    else if (state == RunState::Failed)
        on = (now / 100) % 2 == 0;
    digitalWrite(mission_config::STATUS_LED_PIN, on ? HIGH : LOW);
}
} // namespace

void setup()
{
    delay(1000);
    pinMode(mission_config::STATUS_LED_PIN, OUTPUT);
    digitalWrite(mission_config::STATUS_LED_PIN, LOW);

    DebugLog::begin(&serialDebug, debug_config::BAUD, DebugLog::Level::INFO);
    LOG_INFO("SmallMove", "=== mechanism small move test ===");

    startButton.reset();
    startButton.attachClick(startRequested);

    stepperProtocol.init(&serialStepper, BUS_BAUD);
    baseProtocol.init(&serialBase, BUS_BAUD);

    lift.init(LIFT_STEPPER_ID, &stepperProtocol);
    lift.set(LIFT_SPEED, LIFT_ACCELERATION, LIFT_CW,
             LIFT_CONVERT_K, LIFT_SUBSTEP);
    extension.init(EXTENSION_STEPPER_ID, &stepperProtocol);
    extension.set(EXTENSION_SPEED, EXTENSION_ACCELERATION, EXTENSION_CW,
                  EXTENSION_CONVERT_K, EXTENSION_SUBSTEP);
    baseMotor.init(BASE_STEPPER_ID, &baseProtocol);
    baseMotor.set(BASE_SPEED, BASE_ACCELERATION, BASE_CW,
                  BASE_CONVERT_K, BASE_SUBSTEP);

    LOG_INFO("SmallMove", "ready: press PB9 or send 'n' to start");
}

void loop()
{
    startButton.tick();
    while (serialDebug.available() > 0)
    {
        if (serialDebug.read() == 'n')
            startRequested();
    }
    updateTest();
    updateStatusLed();
}
