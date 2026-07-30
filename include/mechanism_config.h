#pragma once

#include <Arduino.h>

/**
 * @brief 机械臂硬件和标定参数。
 *
 * 硬件ID、引脚和电机基础参数以new_project为准。
 * 位置参数已经开始按实车标定，后续仍只在本文件中调整。
 *
 * TTL步进位置单位：
 * - 升降、伸缩：0.1 mm；
 * - 底座旋转：0.1°。
 */
namespace mechanism_config
{
    // -------------------- 硬件映射（以new_project为准） --------------------
    constexpr uint32_t STEPPER_RX_PIN = PA3;
    constexpr uint32_t STEPPER_TX_PIN = PA2;
    constexpr uint32_t BASE_RX_PIN = PA10;
    constexpr uint32_t BASE_TX_PIN = PA9;
    constexpr uint32_t SERVO_RX_PIN = PC7;
    constexpr uint32_t SERVO_TX_PIN = PC6;

    constexpr uint32_t BUS_BAUD = 115200;

    constexpr uint8_t LIFT_STEPPER_ID = 7;
    constexpr uint8_t EXTENSION_STEPPER_ID = 6;
    constexpr uint8_t BASE_STEPPER_ID = 5;
    constexpr uint8_t GRIPPER_SERVO_ID = 4;
    constexpr uint8_t STORAGE_SERVO_ID = 5;

    // -------------------- 步进驱动参数（以new_project为准） --------------------
    constexpr uint16_t LIFT_SPEED = 2500;
    constexpr uint8_t LIFT_ACCELERATION = 255;
    constexpr bool LIFT_CW = false;
    constexpr float LIFT_CONVERT_K = 120.0F;
    constexpr uint16_t LIFT_SUBSTEP = 16;

    constexpr uint16_t EXTENSION_SPEED = 120;
    constexpr uint8_t EXTENSION_ACCELERATION = 150;
    constexpr bool EXTENSION_CW = true;
    constexpr float EXTENSION_CONVERT_K = 1131.0F;
    constexpr uint16_t EXTENSION_SUBSTEP = 16;

    constexpr uint16_t BASE_SPEED = 180;
    constexpr uint8_t BASE_ACCELERATION = 250;
    constexpr bool BASE_CW = true;
    constexpr float BASE_CONVERT_K = 900.0F;
    constexpr uint16_t BASE_SUBSTEP = 16;

    // -------------------- 舵机参数（以new_project为准） --------------------
    constexpr float GRIPPER_OPEN_ANGLE = 56.0F;
    constexpr float GRIPPER_CLOSE_ANGLE = 105.0F;
    constexpr float GRIPPER_OPEN_MAX_ANGLE = 55.0F;
    constexpr uint16_t GRIPPER_MAX_POWER = 700;
    /*
     * 闭合过快会在夹爪尚未包住物料时将其碰飞。600ms只控制舵机
     * 闭合速度；动作完成仍由实际角度和夹持功率反馈判断。
     */
    constexpr uint16_t GRIPPER_COMMAND_INTERVAL_MS = 600;
    constexpr float STORAGE_SPEED_DPS = 600.0F;

    // 到位反馈参数。夹紧物料时允许夹爪因接触物料而停在目标角度之前。
    constexpr float STORAGE_POSITION_TOLERANCE_DEG = 3.0F;
    constexpr float GRIPPER_POSITION_TOLERANCE_DEG = 4.0F;
    constexpr uint16_t GRIPPER_LOAD_POWER_MW = 250;
    constexpr float GRIPPER_LOAD_MIN_ANGLE = 61.0F;
    constexpr float GRIPPER_STALL_ANGLE_DELTA_DEG = 1.5F;
    constexpr uint8_t SERVO_STABLE_FEEDBACK_COUNT = 2;
    // 夹紧必须持续多帧保持负载，避免接触瞬间的功率尖峰被当作成功。
    constexpr uint8_t GRIPPER_STABLE_FEEDBACK_COUNT = 4;
    // 步进驱动偶尔会在到位瞬间短暂置位堵转标志，连续确认后才报错。
    constexpr uint8_t STEPPER_FAULT_CONFIRM_COUNT = 3;

    /*
     * 载物盘只有三个物料槽。任务可能出现四种颜色中的任意三种，因此
     * 槽位按“本批抓取顺序”分配，而不是把四种颜色固定映射到三个槽。
     */
    constexpr float TRAY_SLOT_ANGLE[4] = {
        105.0F, // 0：收纳/出发位
        -90.0F, // 1：本批第一个物料
        0.0F,   // 2：本批第二个物料
        90.0F   // 3：本批第三个物料
    };

    // -------------------- 待实车微调的位置参数 --------------------
    // 上电初始化位置，与比赛动作完成后的收纳位置相互独立。
    constexpr float LIFT_INITIAL = 120.0F;
    constexpr float EXTENSION_INITIAL = 900.0F;
    constexpr float BASE_INITIAL = 0.0F;

    constexpr float LIFT_HOME = 120.0F;
    // 粗加工区和暂存区视觉对准时降低升降轴，避免机构遮挡相机。
    constexpr float LIFT_STATION_VISION = 900.0F;
    constexpr float LIFT_TURNTABLE = 410.0F;
    // 机械臂在载物盘处始终使用这一组固定交接坐标。
    constexpr float LIFT_TRAY_TRANSFER = 235.0F;
    constexpr float MATERIAL_HEIGHT = 700.0F;

    // 实车收纳位置：伸缩轴在1500时完全收回。
    constexpr float EXTENSION_HOME = 1510.0F;
    // 从载物盘取放物料时的伸缩位置。
    constexpr float EXTENSION_TRAY_TRANSFER = 1500.0F;
    constexpr float EXTENSION_TURNTABLE = 1250.0F;

    // 工位动作完成后，底座转到180°车内收纳方向。
    constexpr float BASE_HOME = 1800.0F;
    // 从载物盘取放物料时的底座方向。
    constexpr float BASE_TRAY_TRANSFER = 2770.0F;
    constexpr float BASE_TURNTABLE = 1800.0F;

    /**
     * @brief 从底盘统一基准位置到某个圆环的机械臂点位。
     *
     * base、lift和extension分别对应底座、升降和伸缩轴的绝对目标值。
     * 粗加工区视觉只负责把整车对准统一基准，随后不再移动底盘。
     */
    struct RingPose
    {
        float base;
        float lift;
        float extension;
    };

    /*
     * 粗加工区圆环点位表。
     * 下标0是无效占位；下标1~3直接对应任务码中的1~3号圆环。
     * 当前先沿用原有初值，现场标定时只修改对应圆环的一行。
     */
    constexpr RingPose ROUGH_RING_POSES[4] = {
        {BASE_TRAY_TRANSFER, 0.0F, 0.0F},
        {1320.0F, 1185.0F, 1034.0F}, // 1号圆环
        {1800.0F, 1185.0F, 1510.0F}, // 2号圆环
        {2267.0F, 1185.0F, 710.0F}   // 3号圆环
    };

    /*
     * 暂存区保持独立标定，避免今后调整粗加工区时影响码垛。
     * 当前数值与原逻辑一致，后续同样按圆环编号逐行修改。
     */
    constexpr RingPose FINAL_STORAGE_RING_POSES[4] = {
        {BASE_TRAY_TRANSFER, 0.0F, 0.0F},
        {1320.0F, 1185.0F, 1034.0F}, // 1号圆环
        {1800.0F, 1185.0F, 1510.0F}, // 2号圆环
        {2267.0F, 1185.0F, 710.0F}   // 3号圆环
    };

    // -------------------- 非阻塞执行保护 --------------------
    constexpr uint32_t STEPPER_POLL_MS = 25;
    constexpr uint32_t STEPPER_TIMEOUT_MS = 15000;
    constexpr uint32_t SERVO_POLL_MS = 60;
    // 仅用于检测掉线或卡死，不参与正常动作完成判定。
    constexpr uint32_t SERVO_TIMEOUT_MS = 5000;
} // namespace mechanism_config
