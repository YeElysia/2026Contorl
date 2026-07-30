#pragma once

#include <Arduino.h>

namespace chassis_config
{
    // STM32H750 底盘步进电机引脚（沿用现有小车接线）
    constexpr uint32_t ENABLE_PIN = PE13;
    constexpr uint32_t DIR_PINS[4] = {PD6, PE9, PD14, PC3_C};
    constexpr uint32_t STEP_PINS[4] = {PD4, PE11, PD15, PA1};

    // WIT/JY901 IMU 串口。PB12/PB13不启用，避免影响底盘。
    constexpr uint32_t IMU_RX_PIN = PD9;
    constexpr uint32_t IMU_TX_PIN = PD8;
    constexpr uint32_t IMU_BAUD = 115200;

    // 机械参数。换轮或调整细分时只修改这里。
    constexpr float WHEEL_DIAMETER_MM = 100.0f;
    constexpr float MOTOR_STEP_ANGLE_DEG = 1.8f;
    constexpr uint16_t MICROSTEPS = 32;
    constexpr float STEPS_PER_REV =
        (360.0f / MOTOR_STEP_ANGLE_DEG) * MICROSTEPS;
    constexpr float MM_PER_REV = PI * WHEEL_DIAMETER_MM;
    constexpr float STEPS_PER_MM = STEPS_PER_REV / MM_PER_REV;

    // 电机正方向。若某个轮子反转，只修改对应项。
    constexpr int8_t MOTOR_SIGN[4] = {-1, 1, -1, 1};

    /*
     * 运动参数（单位：轮子RPM、RPM/s、度）。
     *
     * 快速档用于长距离转场。AccelStepper会根据剩余距离自动减速，
     * 因此提高最高转速不会让电机以最高速度撞到目标点。
     */
    constexpr float DRIVE_RPM = 600.0f;

    /*
     * 快速档加速度从已验证的50 RPM/s逐步提高到90 RPM/s，
     * 对应约9600 pulse/s²。它明显快于原配置，但仍远低于
     * 曾导致失步的32000 pulse/s²。
     */
    constexpr float DRIVE_ACCEL_RPM_PER_S = 150.0f;

    /*
     * 精确档用于工位前20~50 mm进退、最终停车和视觉微调。
     * 保留较低速度与加速度，降低麦轮打滑和机构晃动。
     */
    constexpr float PRECISE_DRIVE_RPM = 90.0f;
    constexpr float PRECISE_DRIVE_ACCEL_RPM_PER_S = 50.0f;

    // 基础位置控制已经验证通过，现在启用IMU直行航向保持。
    constexpr bool ENABLE_HEADING_HOLD = true;
    constexpr float ROTATE_MAX_RPM = 100.0f;
    constexpr float HEADING_KP_STEPS_PER_S_PER_DEG = 35.0f;
    constexpr float HEADING_MAX_CORRECTION_RATIO = 0.30f;
    constexpr float ROTATE_KP_STEPS_PER_S_PER_DEG = 105.0f;
    constexpr float ROTATE_TOLERANCE_DEG = 0.7f;
    constexpr uint8_t ROTATE_STABLE_SAMPLES = 5;
    constexpr uint32_t IMU_STALE_TIMEOUT_MS = 250;
    constexpr uint32_t MOTION_TIMEOUT_MS = 15000;
} // namespace chassis_config
