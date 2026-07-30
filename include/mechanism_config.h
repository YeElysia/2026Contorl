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

    constexpr uint16_t EXTENSION_SPEED = 150;
    constexpr uint8_t EXTENSION_ACCELERATION = 250;
    constexpr bool EXTENSION_CW = true;
    constexpr float EXTENSION_CONVERT_K = 1131.0F;
    constexpr uint16_t EXTENSION_SUBSTEP = 16;

    constexpr uint16_t BASE_SPEED = 250;
    constexpr uint8_t BASE_ACCELERATION = 250;
    constexpr bool BASE_CW = true;
    constexpr float BASE_CONVERT_K = 900.0F;
    constexpr uint16_t BASE_SUBSTEP = 16;

    // -------------------- 舵机参数（以new_project为准） --------------------
    constexpr float GRIPPER_OPEN_ANGLE = 38.0F;
    constexpr float GRIPPER_CLOSE_ANGLE = 105.0F;
    constexpr float GRIPPER_OPEN_MAX_ANGLE = 23.0F;
    constexpr uint16_t GRIPPER_MAX_POWER = 700;
    constexpr uint16_t GRIPPER_MOVE_MS = 450;
    constexpr uint16_t STORAGE_MOVE_MS = 650;

    /*
     * 载物盘只有三个物料槽。任务可能出现四种颜色中的任意三种，因此
     * 槽位按“本批抓取顺序”分配，而不是把四种颜色固定映射到三个槽。
     */
    constexpr float STORAGE_ANGLE[4] = {
        103.0F, // 0：收纳/出发位
        -83.0F, // 1：本批第一个物料
        7.0F,   // 2：本批第二个物料
        97.0F   // 3：本批第三个物料
    };

    // -------------------- 待实车微调的位置参数 --------------------
    // 上电初始化位置，与比赛动作完成后的收纳位置相互独立。
    constexpr float LIFT_INITIAL = 0.0F;
    constexpr float EXTENSION_INITIAL = 900.0F;
    constexpr float BASE_INITIAL = 0.0F;

    constexpr float LIFT_HOME = 0.0F;
    constexpr float LIFT_TURNTABLE = 410.0F;
    constexpr float LIFT_GROUND = 1200.0F;
    constexpr float LIFT_STORAGE = 240.0F;
    constexpr float MATERIAL_HEIGHT = 700.0F;

    // 实车收纳位置：伸缩轴在1500时完全收回。
    constexpr float EXTENSION_HOME = 1500.0F;
    // 从载物盘取放物料时的伸缩位置。
    constexpr float EXTENSION_STORAGE = 1480.0F;
    constexpr float EXTENSION_TURNTABLE = 800.0F;

    // 工位动作完成后，底座转到180°车内收纳方向。
    constexpr float BASE_HOME = 1800.0F;
    // 从载物盘取放物料时的底座方向。
    constexpr float BASE_STORAGE = 2730.0F;
    constexpr float BASE_TURNTABLE = 1800.0F;

    /*
     * 下标0不用；下标1~3分别对应场地1~3号圆环。
     * 这里沿用旧项目的三组初值，但语义已从“颜色”改成“圆环编号”。
     */
    constexpr float RING_EXTENSION[4] = {
        0.0F,
        1200.0F,
        1200.0F,
        1200.0F};
    constexpr float RING_BASE_ANGLE[4] = {
        BASE_STORAGE,
        1800.189F,
        1800.0F,
        1800.721F};

    // -------------------- 非阻塞执行保护 --------------------
    constexpr uint32_t STEPPER_POLL_MS = 25;
    constexpr uint32_t STEPPER_TIMEOUT_MS = 15000;
} // namespace mechanism_config
