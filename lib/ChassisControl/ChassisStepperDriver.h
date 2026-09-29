#pragma once

#include <Arduino.h>

/**
 * @brief 四轮STEP/DIR脉冲发生器，脉冲全部在TIM13定时中断里产生。
 *
 * 主循环只下发目标位置或目标速度，不再负责出脉冲，因此串口、屏幕、
 * 视觉处理造成的循环抖动不会影响轮速。所有接口都会短暂屏蔽本中断
 * （BASEPRI，不影响UART和SysTick），可以在loop()中任意调用。
 *
 * 位置模式：按梯形速度曲线走到目标，巡航速度可在运动中修改。
 * 速度模式：按加速度限制跟随指令速度；超过VELOCITY_WATCHDOG_MS
 * 没有刷新指令时，中断自行减速到零并置看门狗标志。
 */
class ChassisStepperDriver
{
public:
    static constexpr uint8_t WHEEL_COUNT = 4;

    ChassisStepperDriver();

    void begin();

    // 位置模式：从当前位置相对移动steps步。steps为0时原地保持。
    void moveRelative(
        uint8_t wheel,
        long steps,
        float maxSpeed,
        float acceleration);
    // 位置模式下修改巡航速度，超过的部分按加速度减速。
    void setMaxSpeed(uint8_t wheel, float maxSpeed);
    // 速度模式：四轮同时设置有符号速度，并刷新看门狗。
    void setVelocities(
        const float (&stepsPerSecond)[WHEEL_COUNT],
        float acceleration);
    // 按各轮当前加速度减速停车。
    void stop();
    // 立即停止并清除残余速度和目标。
    void halt();

    void positions(long (&out)[WHEEL_COUNT]) const;
    long distanceToGo(uint8_t wheel) const;
    bool allStopped() const;
    // 读取并清除速度看门狗标志。
    bool takeWatchdogTrip();

private:
    enum class Mode : uint8_t
    {
        Idle,
        Position,
        Velocity
    };

    struct Wheel
    {
        GPIO_TypeDef *stepPort;
        uint32_t stepMask;
        GPIO_TypeDef *dirPort;
        uint32_t dirMask;
        Mode mode;
        int32_t position;
        int32_t target;
        float speed;
        float maxSpeed;
        float acceleration;
        float velocityCommand;
        float phase;
        bool dirHigh;
        bool stepHigh;
    };

    class IsrLock;

    Wheel _wheels[WHEEL_COUNT] = {};
    uint32_t _velocityAgeTicks = 0;
    bool _watchdogTripped = false;

    static ChassisStepperDriver *s_instance;
    static void onTimerTick();
    void onTick();
    static void tickWheel(Wheel &wheel, bool pulsedLastTick);
    static float clampSpeed(float speed);
};
