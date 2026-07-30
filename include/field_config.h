#pragma once

namespace field_config
{
    /*
     * 场地统一使用底盘几何中心作为定位点，单位均为毫米。
     *
     * +x指向小车出发时的左侧，+y指向出发时的车头方向。
     * 航向角为0度时车头朝向+y。
     */
    constexpr float START_X_MM = 185.0F;
    constexpr float START_Y_MM = 150.0F;
    constexpr float START_YAW_DEG = 0.0F;

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
        START_X_MM >= CHASSIS_WIDTH_MM * 0.5F,
        "start x places chassis outside lower field boundary");
    static_assert(
        START_Y_MM >= CHASSIS_LENGTH_MM * 0.5F,
        "start y places chassis outside lower field boundary");
    static_assert(
        hasRotationClearance(TURN_CENTER_X_MM, TURN_CENTER_Y_MM),
        "turn center has insufficient chassis rotation clearance");
} // namespace field_config
