#pragma once

#include <Arduino.h>

namespace chassis_config
{
    // STM32H750 底盘步进电机引脚（沿用现有小车接线）
    constexpr uint32_t ENABLE_PIN = PE13;
    constexpr uint32_t DIR_PINS[4] = {PD6, PE9, PD14, PC3_C};
    constexpr uint32_t STEP_PINS[4] = {PD4, PE11, PD15, PA1};

    /*
     * EMM42 V5 的PUL输入需要留足高电平宽度。AccelStepper默认值
     * 偏短，这里固定为3us，兼顾可靠识别和当前最高脉冲频率。
     */
    constexpr uint16_t STEP_PULSE_WIDTH_US = 3;

    // WIT/JY901 IMU 串口。PB12/PB13不启用，避免影响底盘。
    constexpr uint32_t IMU_RX_PIN = PD9;
    constexpr uint32_t IMU_TX_PIN = PD8;
    constexpr uint32_t IMU_BAUD = 115200;

    // EMM42 V5 TTL反馈串口（所有底盘电机共用）
    constexpr uint32_t MOTOR_TTL_RX_PIN = PA10;
    constexpr uint32_t MOTOR_TTL_TX_PIN = PA9;
    constexpr uint32_t MOTOR_TTL_BAUD = 115200;
    /*
     * 4个底盘电机的TTL地址，顺序：左前、右前、左后、右后。
     * 若电机未设置地址或地址冲突，需通过EMM42配置工具修改。
     * 地址必须唯一且非零，范围通常1-247。
     */
    constexpr uint8_t MOTOR_TTL_ADDRESSES[4] = {1, 2, 3, 4};

    // 机械参数。换轮或调整细分时只修改这里。
    constexpr float WHEEL_DIAMETER_MM = 100.0f;
    constexpr float MOTOR_STEP_ANGLE_DEG = 1.8f;
    constexpr uint16_t MICROSTEPS = 32;
    constexpr float STEPS_PER_REV =
        (360.0f / MOTOR_STEP_ANGLE_DEG) * MICROSTEPS;
    constexpr float MM_PER_REV = PI * WHEEL_DIAMETER_MM;
    constexpr float STEPS_PER_MM = STEPS_PER_REV / MM_PER_REV;

    /*
     * 四个方向的距离标定系数，含义是“目标脉冲倍率”。
     * 实车跑1000mm后，系数按 1000 / 实测距离 修正。例如只跑了
     * 980mm，则对应方向填1.0204。前后、左右分开标定以补偿麦轮
     * 安装和地面摩擦带来的非对称误差。
     */
    constexpr float FORWARD_DISTANCE_SCALE = 1.0f; // 1000.0f / 994.5f; 1.00553
    constexpr float BACKWARD_DISTANCE_SCALE = 1.0f;
    constexpr float RIGHT_DISTANCE_SCALE = 1.0f; // 1000.0f / 983.0f; 1.01729
    constexpr float LEFT_DISTANCE_SCALE = 1.0f;

    // 电机正方向。若某个轮子反转，只修改对应项。
    constexpr int8_t MOTOR_SIGN[4] = {-1, 1, -1, 1};

    /*
     * 运动参数（单位：轮子RPM、RPM/s、度）。
     *
     * 快速档用于长距离转场。AccelStepper会根据剩余距离自动减速，
     * 因此提高最高转速不会让电机以最高速度撞到目标点。
     */
    constexpr float DRIVE_RPM = 800.0f;

    /*
     * 快速档加速度
     */
    constexpr float DRIVE_ACCEL_RPM_PER_S = 200.0f;

    /*
     * 精确档用于工位前20~50 mm进退、最终停车和视觉微调。
     * 保留较低速度与加速度，降低麦轮打滑和机构晃动。
     */
    constexpr float PRECISE_DRIVE_RPM = 90.0f;
    constexpr float PRECISE_DRIVE_ACCEL_RPM_PER_S = 50.0f;
    /*
     * 原料盘视觉追踪专用档。它仍是有目标位置和软限位的闭环短动作，
     * 但不为每个视觉修正执行最终航向静定，因此可高于普通精确档。
     */
    constexpr float GRASP_TRACK_DRIVE_RPM = 110.0f;
    constexpr float GRASP_TRACK_ACCEL_RPM_PER_S = 70.0f;

    // 基础位置控制已经验证通过，现在启用IMU直行航向保持。
    constexpr bool ENABLE_HEADING_HOLD = true;
    // 旋转采用PD目标速度，并由软件斜坡真正限制轮速变化率。
    constexpr float ROTATE_MAX_RPM = 120.0f;
    constexpr float ROTATE_ACCEL_RPM_PER_S = 300.0f;
    constexpr float HEADING_KP_STEPS_PER_S_PER_DEG = 35.0f;
    constexpr float HEADING_MAX_CORRECTION_RATIO = 0.30f;
    constexpr float ROTATE_KP_STEPS_PER_S_PER_DEG = 135.0f;
    constexpr float ROTATE_KD_STEPS_PER_S_PER_DEG_PER_S = 18.0f;
    constexpr float ROTATE_YAW_RATE_FILTER_ALPHA = 0.35f;
    constexpr float ROTATE_MIN_STEPS_PER_S = 110.0f;
    constexpr float ROTATE_TOLERANCE_DEG = 0.6f;
    constexpr float TRANSLATION_FINAL_HEADING_TOLERANCE_DEG = 0.8f;
    constexpr uint8_t ROTATE_STABLE_SAMPLES = 3;
    constexpr uint32_t ROTATE_STABLE_TIME_MS = 80;
    constexpr uint32_t IMU_STALE_TIMEOUT_MS = 250;

    /*
     * 超时按每次动作的脉冲行程、速度和加速度动态估算。
     * 比理论梯形速度曲线留出充足余量，短距离精确动作不再与
     * 长距离转场共用一个武断的15秒阈值。
     */
    constexpr float MOTION_TIMEOUT_SCALE = 2.5f;
    constexpr uint32_t MOTION_TIMEOUT_MARGIN_MS = 2500;
    constexpr uint32_t MOTION_TIMEOUT_MIN_MS = 4000;
    constexpr uint32_t MOTION_TIMEOUT_MAX_MS = 30000;
    constexpr uint32_t ROTATION_TIMEOUT_BASE_MS = 4000;
    constexpr float ROTATION_TIMEOUT_PER_DEG_MS = 70.0f;
    constexpr uint32_t STOP_TIMEOUT_MS = 2500;

    // 防止错误任务数据被换算成溢出的脉冲目标。
    constexpr float MAX_SINGLE_MOVE_COMPONENT_MM = 5000.0f;

    static_assert(STEP_PULSE_WIDTH_US >= 3,
                  "EMM42 V5 step pulse width must be at least 3us");
    static_assert(FORWARD_DISTANCE_SCALE > 0.0f &&
                      BACKWARD_DISTANCE_SCALE > 0.0f &&
                      RIGHT_DISTANCE_SCALE > 0.0f &&
                      LEFT_DISTANCE_SCALE > 0.0f,
                  "chassis distance calibration scales must be positive");
} // namespace chassis_config
