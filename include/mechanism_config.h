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
    constexpr uint8_t LIFT_ACCELERATION = 253;
    // 升降坐标变小为向上，上行动作单独使用该档位。
    constexpr uint16_t LIFT_UP_SPEED = 5000;
    constexpr uint8_t LIFT_UP_ACCELERATION = 255;
    constexpr bool LIFT_CW = false;
    constexpr float LIFT_CONVERT_K = 120.0F;
    constexpr uint16_t LIFT_SUBSTEP = 16;

    constexpr uint16_t EXTENSION_SPEED = 170;
    constexpr uint8_t EXTENSION_ACCELERATION = 200;
    // 原料视觉追踪采用旧项目接近的快速档，其他机构动作仍保持120RPM。
    constexpr uint16_t PICKUP_EXTENSION_TRACK_SPEED = 220;
    constexpr uint8_t PICKUP_EXTENSION_TRACK_ACCELERATION = 220;
    constexpr bool EXTENSION_CW = true;
    constexpr float EXTENSION_CONVERT_K = 1131.0F;
    constexpr uint16_t EXTENSION_SUBSTEP = 16;

    constexpr uint16_t BASE_SPEED = 180;
    constexpr uint8_t BASE_ACCELERATION = 200;
    constexpr bool BASE_CW = true;
    constexpr float BASE_CONVERT_K = 900.0F;
    constexpr uint16_t BASE_SUBSTEP = 16;

    // 储料盘到粗加工圆环及暂存区第一层时，Base统一使用该快速档。
    constexpr uint16_t TRAY_TO_ROUGH_RING_BASE_SPEED = 460;
    constexpr uint8_t TRAY_TO_ROUGH_RING_BASE_ACCELERATION = 247;

    /*
     * 粗加工圆环直接回到储料盘2770时使用的三组速度。
     * 参数保持实车当前值，使三个圆环的直接回程时间接近升降时间。
     */
    constexpr uint16_t ROUGH_RING_TO_TRAY_SPEED[4] = {
        BASE_SPEED, 200, 200, 250};
    constexpr uint8_t ROUGH_RING_TO_TRAY_ACCELERATION[4] = {
        BASE_ACCELERATION, 180, 220, 240};

    // -------------------- 舵机参数（以new_project为准） --------------------
    constexpr float GRIPPER_OPEN_ANGLE = 80.0F;
    constexpr float GRIPPER_CLOSE_ANGLE = 105.0F;
    constexpr float GRIPPER_OPEN_MAX_ANGLE = 50.0F;
    constexpr uint16_t GRIPPER_MAX_POWER = 700;
    /*
     * 直接使用FSUS_Servo的非阻塞定时角度指令，不依赖FSGP夹爪
     * 封装中未实现的负载检测。
     */
    // 张开尽量快；闭合略慢，避免夹爪接触物料时将其碰飞。
    constexpr uint16_t GRIPPER_OPEN_INTERVAL_MS = 100;
    constexpr uint16_t GRIPPER_CLOSE_INTERVAL_MS = 100;
    constexpr uint16_t GRIPPER_SETTLE_MS = 50;
    constexpr float STORAGE_SPEED_DPS = 600.0F;
    // 载物盘最大转角约195°，600°/s下留足转动及机械稳定时间。
    constexpr uint16_t STORAGE_SERVO_SETTLE_MS = 450;

    // 步进驱动偶尔会在到位瞬间短暂置位堵转标志，连续确认后才报错。
    constexpr uint8_t STEPPER_FAULT_CONFIRM_COUNT = 3;
    // Lift Home和环切安全高度均使用两次独立到位帧。
    constexpr uint8_t LIFT_CRITICAL_CONFIRM_COUNT = 2;

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
    constexpr float LIFT_INITIAL = 20.0F;
    constexpr float EXTENSION_INITIAL = 900.0F;
    constexpr float BASE_INITIAL = 0.0F;

    constexpr float LIFT_HOME = 20.0F;
    // 原料盘搜索和视觉追踪专用高度；对准完成后再下降到夹取高度。
    constexpr float LIFT_MATERIAL_VISION = 150.0F;
    // 粗加工区和暂存区视觉对准时降低升降轴，避免机构遮挡相机。
    constexpr float LIFT_STATION_VISION = 900.0F;
    constexpr float LIFT_TURNTABLE = 500.0F;
    // 机械臂在载物盘处始终使用这一组固定交接坐标。
    constexpr float LIFT_TRAY_TRANSFER = 240.0F;
    // 从圆环夹紧物料后先竖直上抬，脱离物料后才允许返回储料盘。
    constexpr float RING_PICKUP_RELEASE_LIFT = 100.0F;
    // 粗加工第三件放完后直切第一个圆环时的安全高度。
    constexpr float LIFT_RING_SWITCH_CLEARANCE = 900.0F;
    // 任意圆环放料松爪后先上抬该距离，再允许机构返回或切环。
    constexpr float RING_PLACEMENT_RELEASE_LIFT = 100.0F;
    // 物料高度
    constexpr float MATERIAL_HEIGHT = 600.0F;

    // 实车收纳位置：伸缩轴在1500时完全收回。
    constexpr float EXTENSION_HOME = 1500.0F;
    // 从载物盘取放物料时的伸缩位置。
    constexpr float EXTENSION_TRAY_TRANSFER = 1485.0F;
    constexpr float EXTENSION_TURNTABLE = 1450.0F;

    // 工位动作完成后，底座转到180°车内收纳方向。
    constexpr float BASE_HOME = 1800.0F;
    // 从载物盘取放物料时的底座方向。
    constexpr float BASE_TRAY_TRANSFER = 2795.0F;
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
        {2296.0F, 1180.0F, 803.0F},  // 1号圆环
        {1805.0F, 1180.0F, 1600.0F}, // 2号圆环
        {1305.0F, 1180.0F, 1035.0F}, // 3号圆环
    };

    /*
     * 暂存区保持独立标定，避免今后调整粗加工区时影响码垛。
     * 当前数值与原逻辑一致，后续同样按圆环编号逐行修改。
     */
    constexpr RingPose FINAL_STORAGE_RING_POSES[4] = {
        {BASE_TRAY_TRANSFER, 0.0F, 0.0F},
        {2296.0F, 1175.0F, 803.0F},  // 1号圆环
        {1805.0F, 1175.0F, 1600.0F}, // 2号圆环
        {1305.0F, 1175.0F, 1035.0F}, // 3号圆环
    };

    // -------------------- 非阻塞执行保护 --------------------
    constexpr uint32_t STEPPER_POLL_MS = 25;
    // TTL状态查询丢帧后不能永久卡在Ask_State，超时后重发。
    constexpr uint32_t STEPPER_QUERY_RESPONSE_TIMEOUT_MS = 100;
    constexpr uint32_t STEPPER_TIMEOUT_MS = 15000;

} // namespace mechanism_config
