#include "ChassisStepperDriver.h"

#include "chassis_config.h"
#include <math.h>

using namespace chassis_config;

namespace
{
    constexpr float TICK_SECONDS = 1.0F / static_cast<float>(STEP_TIMER_HZ);
    constexpr uint32_t VELOCITY_WATCHDOG_TICKS =
        VELOCITY_WATCHDOG_MS * (STEP_TIMER_HZ / 1000UL);
} // namespace

// 只屏蔽优先级不高于脉冲中断的中断，UART和SysTick照常响应。
class ChassisStepperDriver::IsrLock
{
public:
    IsrLock() : _previous(__get_BASEPRI())
    {
        __set_BASEPRI_MAX(
            STEP_TIMER_IRQ_PRIORITY << (8U - __NVIC_PRIO_BITS));
        __DSB();
        __ISB();
    }

    ~IsrLock()
    {
        __set_BASEPRI(_previous);
    }

    IsrLock(const IsrLock &) = delete;
    IsrLock &operator=(const IsrLock &) = delete;

private:
    uint32_t _previous;
};

ChassisStepperDriver *ChassisStepperDriver::s_instance = nullptr;

ChassisStepperDriver::ChassisStepperDriver() = default;

void ChassisStepperDriver::begin()
{
    for (uint8_t i = 0; i < WHEEL_COUNT; ++i)
    {
        pinMode(STEP_PINS[i], OUTPUT);
        pinMode(DIR_PINS[i], OUTPUT);
        digitalWrite(STEP_PINS[i], LOW);
        digitalWrite(DIR_PINS[i], LOW);

        Wheel &wheel = _wheels[i];
        wheel = {};
        wheel.stepPort = digitalPinToPort(STEP_PINS[i]);
        wheel.stepMask = digitalPinToBitMask(STEP_PINS[i]);
        wheel.dirPort = digitalPinToPort(DIR_PINS[i]);
        wheel.dirMask = digitalPinToBitMask(DIR_PINS[i]);
        wheel.mode = Mode::Idle;
    }
    _velocityAgeTicks = 0;
    _watchdogTripped = false;

    if (s_instance != nullptr)
        return;
    s_instance = this;

    static HardwareTimer timer(TIM13);
    timer.setOverflow(STEP_TIMER_HZ, HERTZ_FORMAT);
    timer.setInterruptPriority(STEP_TIMER_IRQ_PRIORITY, 0);
    timer.attachInterrupt(onTimerTick);
    timer.resume();
}

void ChassisStepperDriver::moveRelative(
    uint8_t wheel,
    long steps,
    float maxSpeed,
    float acceleration)
{
    if (wheel >= WHEEL_COUNT)
        return;

    IsrLock lock;
    Wheel &w = _wheels[wheel];
    w.target = w.position + static_cast<int32_t>(steps);
    w.maxSpeed = clampSpeed(fabsf(maxSpeed));
    w.acceleration = fabsf(acceleration);
    w.mode = Mode::Position;
}

void ChassisStepperDriver::setMaxSpeed(uint8_t wheel, float maxSpeed)
{
    if (wheel >= WHEEL_COUNT)
        return;

    IsrLock lock;
    _wheels[wheel].maxSpeed = clampSpeed(fabsf(maxSpeed));
}

void ChassisStepperDriver::setVelocities(
    const float (&stepsPerSecond)[WHEEL_COUNT],
    float acceleration)
{
    IsrLock lock;
    // 看门狗触发后保持零速，直到上层读取标志并进入故障。
    if (_watchdogTripped)
        return;

    for (uint8_t i = 0; i < WHEEL_COUNT; ++i)
    {
        Wheel &w = _wheels[i];
        w.mode = Mode::Velocity;
        w.velocityCommand = clampSpeed(stepsPerSecond[i]);
        w.acceleration = fabsf(acceleration);
    }
    _velocityAgeTicks = 0;
}

void ChassisStepperDriver::stop()
{
    IsrLock lock;
    for (auto &w : _wheels)
    {
        if (w.mode == Mode::Idle)
            continue;
        if (w.speed == 0.0F || w.acceleration <= 0.0F)
        {
            w.speed = 0.0F;
            w.target = w.position;
            w.mode = Mode::Position;
            continue;
        }

        // 与当前速度同向留出刚好够减速的距离，中断会沿减速曲线停下。
        const long brakingSteps = static_cast<long>(ceilf(
            w.speed * w.speed / (2.0F * w.acceleration)));
        w.target = w.position +
                   static_cast<int32_t>(w.speed > 0.0F ? brakingSteps : -brakingSteps);
        w.maxSpeed = fabsf(w.speed);
        w.mode = Mode::Position;
    }
}

void ChassisStepperDriver::halt()
{
    IsrLock lock;
    for (auto &w : _wheels)
    {
        w.mode = Mode::Idle;
        w.speed = 0.0F;
        w.velocityCommand = 0.0F;
        w.phase = 0.0F;
        w.target = w.position;
    }
}

void ChassisStepperDriver::positions(long (&out)[WHEEL_COUNT]) const
{
    IsrLock lock;
    for (uint8_t i = 0; i < WHEEL_COUNT; ++i)
        out[i] = _wheels[i].position;
}

long ChassisStepperDriver::distanceToGo(uint8_t wheel) const
{
    if (wheel >= WHEEL_COUNT)
        return 0;

    IsrLock lock;
    const Wheel &w = _wheels[wheel];
    return w.mode == Mode::Position ? w.target - w.position : 0;
}

bool ChassisStepperDriver::allStopped() const
{
    IsrLock lock;
    for (const auto &w : _wheels)
    {
        if (w.speed != 0.0F)
            return false;
        if (w.mode == Mode::Position && w.target != w.position)
            return false;
        if (w.mode == Mode::Velocity && w.velocityCommand != 0.0F)
            return false;
    }
    return true;
}

bool ChassisStepperDriver::takeWatchdogTrip()
{
    IsrLock lock;
    const bool tripped = _watchdogTripped;
    _watchdogTripped = false;
    return tripped;
}

void ChassisStepperDriver::onTimerTick()
{
    if (s_instance != nullptr)
        s_instance->onTick();
}

void ChassisStepperDriver::onTick()
{
    bool pulsed[WHEEL_COUNT];
    bool anyVelocity = false;
    for (uint8_t i = 0; i < WHEEL_COUNT; ++i)
    {
        Wheel &w = _wheels[i];
        pulsed[i] = w.stepHigh;
        if (w.stepHigh)
        {
            w.stepPort->BSRR = w.stepMask << 16;
            w.stepHigh = false;
        }
        anyVelocity = anyVelocity || w.mode == Mode::Velocity;
    }

    if (anyVelocity &&
        !_watchdogTripped &&
        ++_velocityAgeTicks > VELOCITY_WATCHDOG_TICKS)
    {
        _watchdogTripped = true;
        for (auto &w : _wheels)
            w.velocityCommand = 0.0F;
    }

    for (uint8_t i = 0; i < WHEEL_COUNT; ++i)
        tickWheel(_wheels[i], pulsed[i]);
}

void ChassisStepperDriver::tickWheel(Wheel &w, bool pulsedLastTick)
{
    float targetSpeed = 0.0F;
    switch (w.mode)
    {
    case Mode::Idle:
        return;
    case Mode::Position:
    {
        const int32_t remaining = w.target - w.position;
        // 到位时的速度足够在两步内停下，就直接归零，避免为一步来回振荡。
        if (remaining == 0 &&
            w.speed * w.speed <= 4.0F * w.acceleration)
        {
            w.speed = 0.0F;
            w.phase = 0.0F;
            return;
        }
        const float brakingLimit = sqrtf(
            2.0F * w.acceleration *
            fabsf(static_cast<float>(remaining)));
        const float cruise = fminf(w.maxSpeed, brakingLimit);
        targetSpeed = remaining >= 0 ? cruise : -cruise;
        break;
    }
    case Mode::Velocity:
        targetSpeed = w.velocityCommand;
        break;
    }

    if (w.acceleration > 0.0F)
    {
        const float maximumDelta = w.acceleration * TICK_SECONDS;
        const float delta = targetSpeed - w.speed;
        w.speed += fmaxf(-maximumDelta, fminf(delta, maximumDelta));
    }
    else
    {
        w.speed = targetSpeed;
    }

    if (w.speed == 0.0F)
        return;

    w.phase += fabsf(w.speed) * TICK_SECONDS;
    if (w.phase < 1.0F)
        return;

    const bool forward = w.speed > 0.0F;
    if (forward != w.dirHigh)
    {
        // 换向后隔一个tick再出脉冲，给驱动器留出DIR建立时间。
        w.dirPort->BSRR = forward ? w.dirMask : (w.dirMask << 16);
        w.dirHigh = forward;
        return;
    }
    if (pulsedLastTick)
        return;

    w.phase -= 1.0F;
    w.position += forward ? 1 : -1;
    w.stepPort->BSRR = w.stepMask;
    w.stepHigh = true;
}

float ChassisStepperDriver::clampSpeed(float speed)
{
    if (!isfinite(speed))
        return 0.0F;
    return fmaxf(-MAX_STEP_RATE, fminf(speed, MAX_STEP_RATE));
}
