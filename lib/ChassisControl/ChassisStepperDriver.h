#pragma once

#include <Arduino.h>

/**
 * @brief 四轮STEP/DIR脉冲发生器，脉冲全部在TIM13定时中断里产生。
 *
 * 主循环只下发目标速度，不负责出脉冲，因此串口、屏幕、视觉处理
 * 造成的循环抖动不会影响脉冲时序。所有接口都会短暂屏蔽本中断
 * （BASEPRI，不影响UART和SysTick），可以在loop()中任意调用。
 *
 * 速度模式：按加速度限制跟随指令速度；超过VELOCITY_WATCHDOG_MS
 * 没有刷新指令时，中断自行减速到零并置看门狗标志。
 * 位置模式只供stop()沿减速曲线刹停。
 */
class ChassisStepperDriver
{
public:
    static constexpr uint8_t WHEEL_COUNT = 4;

    ChassisStepperDriver();

    void begin();

    // 速度模式：四轮同时设置有符号速度，并刷新看门狗。
    void setVelocities(
        const float (&stepsPerSecond)[WHEEL_COUNT],
        float acceleration);
    // 按各轮当前加速度减速停车。
    void stop();
    // 立即停止并清除残余速度和目标。
    void halt();

    void positions(long (&out)[WHEEL_COUNT]) const;
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
