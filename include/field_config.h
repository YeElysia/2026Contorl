#pragma once

#include "start_zone.h"

namespace field_config
{
    /*
     * 场地统一使用底盘几何中心作为定位点，单位均为毫米。
     *
     * +x指向小车出发时的左侧，+y指向出发时的车头方向。
     * 航向角为0度时车头朝向+y。
     */
    struct StartPose
    {
        float xMm;
        float yMm;
        float yawDeg;
    };

    constexpr StartPose LOWER_RIGHT_START = {
        185.0F, 150.0F, 0.0F};
    constexpr StartPose UPPER_RIGHT_START = {
        185.0F, 2250.0F, 0.0F};

    constexpr StartPose startPose(StartZone zone)
    {
        return zone == StartZone::UpperRight
                   ? UPPER_RIGHT_START
                   : LOWER_RIGHT_START;
    }

    // 兼容已有的单启停区测试代码，默认仍使用右下方启停区。
    constexpr float START_X_MM = LOWER_RIGHT_START.xMm;
    constexpr float START_Y_MM = LOWER_RIGHT_START.yMm;
    constexpr float START_YAW_DEG = LOWER_RIGHT_START.yawDeg;

    // 所有改变航向的动作只能在场地中心完成。
    constexpr float TURN_CENTER_X_MM = 1200.0F;
    constexpr float TURN_CENTER_Y_MM = 1200.0F;

    // 底盘外形尺寸，用于点位规划和后续边界检查。
    constexpr float CHASSIS_WIDTH_MM = 230.0F;
    constexpr float CHASSIS_LENGTH_MM = 300.0F;
    constexpr float FIELD_WIDTH_MM = 2400.0F;
    constexpr float FIELD_LENGTH_MM = 2400.0F;
    constexpr float ROTATION_CLEARANCE_RADIUS_MM =
        CHASSIS_LENGTH_MM * 0.5F;

    constexpr bool hasRotationClearance(float xMm, float yMm)
    {
        return
            xMm >= ROTATION_CLEARANCE_RADIUS_MM &&
            xMm <= FIELD_WIDTH_MM - ROTATION_CLEARANCE_RADIUS_MM &&
            yMm >= ROTATION_CLEARANCE_RADIUS_MM &&
            yMm <= FIELD_LENGTH_MM - ROTATION_CLEARANCE_RADIUS_MM;
    }

    static_assert(
        LOWER_RIGHT_START.xMm >= CHASSIS_WIDTH_MM * 0.5F &&
            UPPER_RIGHT_START.xMm >= CHASSIS_WIDTH_MM * 0.5F,
        "start x places chassis outside field boundary");
    static_assert(
        LOWER_RIGHT_START.yMm >= CHASSIS_LENGTH_MM * 0.5F &&
            UPPER_RIGHT_START.yMm <=
                FIELD_LENGTH_MM - CHASSIS_LENGTH_MM * 0.5F,
        "start y places chassis outside field boundary");
    static_assert(
        hasRotationClearance(TURN_CENTER_X_MM, TURN_CENTER_Y_MM),
        "turn center has insufficient chassis rotation clearance");
} // namespace field_config
