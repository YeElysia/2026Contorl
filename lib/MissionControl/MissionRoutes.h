#pragma once

#include "RouteExecutor.h"
#include "field_config.h"

namespace mission_routes
{
/**
 * @brief 场地中的世界坐标点，单位为毫米。
 *
 * 坐标记录底盘几何中心：+x指向出发时小车左侧，+y指向车头。
 * 所有比赛点位集中在本文件，实车标定时不修改状态机。
 */
struct FieldPoint
{
    float xMm;
    float yMm;
};

struct FieldPose
{
    FieldPoint position;
    float yawDeg;
};

constexpr RouteAction fastTo(const FieldPoint &point)
{
    return routeMoveTo(point.xMm, point.yMm);
}

constexpr RouteAction preciseTo(const FieldPoint &point)
{
    return routePreciseMoveTo(point.xMm, point.yMm);
}

constexpr RouteAction preciseTo(const FieldPose &pose)
{
    return preciseTo(pose.position);
}

/*
 * 区域基准点。
 *
 * 离站路线首先以精确档返回当前区域基准位置。基准航向在上一次
 * 中心转向时已经确定，作业期间由IMU保持，不在区域内原地旋转。
 */
constexpr FieldPose HOME_ANCHOR = {
    {field_config::START_X_MM, field_config::START_Y_MM},
    -180.0F};
constexpr FieldPose SCAN_ANCHOR = {
    {field_config::TURN_CENTER_X_MM, field_config::TURN_CENTER_Y_MM},
    0.0F};
constexpr FieldPose MATERIAL_ANCHOR = {{1200.0F, 2140.0F}, 90.0F};
constexpr FieldPose ROUGH_ANCHOR = {{1200.0F, 310.0F}, -90.0F};
constexpr FieldPose STORAGE_ANCHOR = {{2100.0F, 1200.0F}, -180.0F};

static_assert(
    MATERIAL_ANCHOR.position.xMm ==
            field_config::TURN_CENTER_X_MM &&
        ROUGH_ANCHOR.position.xMm ==
            field_config::TURN_CENTER_X_MM &&
        STORAGE_ANCHOR.position.yMm ==
            field_config::TURN_CENTER_Y_MM,
    "fixed station anchors must remain on the 1200 mm center axes");

// 场地唯一允许改变航向的位置。
constexpr FieldPoint TURN_CENTER = {
    field_config::TURN_CENTER_X_MM,
    field_config::TURN_CENTER_Y_MM};

/*
 * 各区域的直线进出点。
 *
 * 先到进出点再到基准点，避免麦轮把两轴差值合成为斜向路线，
 * 同时为机械臂和场地边界留出明确的安全距离。
 */
constexpr FieldPoint HOME_CENTERLINE = {
    field_config::START_X_MM,
    field_config::TURN_CENTER_Y_MM};
constexpr FieldPoint CENTER_FROM_HOME = {1150.0F, 1200.0F};
constexpr FieldPoint CENTER_FROM_MATERIAL = {1200.0F, 1250.0F};
constexpr FieldPoint CENTER_FROM_ROUGH = {1200.0F, 1150.0F};
constexpr FieldPoint CENTER_FROM_STORAGE = {1250.0F, 1200.0F};
constexpr FieldPoint MATERIAL_APPROACH = {1200.0F, 2090.0F};
constexpr FieldPoint ROUGH_APPROACH = {1200.0F, 360.0F};
constexpr FieldPoint STORAGE_APPROACH = {2050.0F, 1200.0F};
constexpr FieldPoint HOME_APPROACH = {
    field_config::START_X_MM,
    field_config::START_Y_MM + 50.0F};

// 启停区 -> 扫码区，扫码区与中心转向点重合。
constexpr RouteAction TO_SCAN[] = {
    fastTo(HOME_CENTERLINE),
    fastTo(CENTER_FROM_HOME),
    preciseTo(TURN_CENTER),
};

// 扫码区 -> 原料区：确认位于中心后才转向。
constexpr RouteAction SCAN_TO_MATERIAL[] = {
    preciseTo(TURN_CENTER),
    routeRotateTo(MATERIAL_ANCHOR.yawDeg),
    fastTo(MATERIAL_APPROACH),
    preciseTo(MATERIAL_ANCHOR),
};

// 原料区 -> 粗加工区，两轮使用同一条全局路线。
constexpr RouteAction MATERIAL_TO_ROUGH[] = {
    preciseTo(MATERIAL_ANCHOR),
    preciseTo(MATERIAL_APPROACH),
    fastTo(CENTER_FROM_MATERIAL),
    preciseTo(TURN_CENTER),
    routeRotateTo(ROUGH_ANCHOR.yawDeg),
    fastTo(ROUGH_APPROACH),
    preciseTo(ROUGH_ANCHOR),
};

// 粗加工区 -> 暂存区，两轮使用同一条全局路线。
constexpr RouteAction ROUGH_TO_STORAGE[] = {
    preciseTo(ROUGH_ANCHOR),
    preciseTo(ROUGH_APPROACH),
    fastTo(CENTER_FROM_ROUGH),
    preciseTo(TURN_CENTER),
    routeRotateTo(STORAGE_ANCHOR.yawDeg),
    fastTo(STORAGE_APPROACH),
    preciseTo(STORAGE_ANCHOR),
};

// 暂存区 -> 原料区（第二轮）。
constexpr RouteAction STORAGE_TO_MATERIAL_SECOND[] = {
    preciseTo(STORAGE_ANCHOR),
    preciseTo(STORAGE_APPROACH),
    fastTo(CENTER_FROM_STORAGE),
    preciseTo(TURN_CENTER),
    routeRotateTo(MATERIAL_ANCHOR.yawDeg),
    fastTo(MATERIAL_APPROACH),
    preciseTo(MATERIAL_ANCHOR),
};

// 暂存区 -> 启停区，保持-180度航向，不在终点旋转。
constexpr RouteAction STORAGE_TO_HOME[] = {
    preciseTo(STORAGE_ANCHOR),
    preciseTo(STORAGE_APPROACH),
    fastTo(CENTER_FROM_STORAGE),
    preciseTo(TURN_CENTER),
    fastTo(HOME_CENTERLINE),
    fastTo(HOME_APPROACH),
    preciseTo(HOME_ANCHOR),
};

constexpr RouteDefinition ROUTE_TO_SCAN = routeDefinition(TO_SCAN);
constexpr RouteDefinition ROUTE_SCAN_TO_MATERIAL =
    routeDefinition(SCAN_TO_MATERIAL);
constexpr RouteDefinition ROUTE_MATERIAL_TO_ROUGH[BATCH_COUNT] = {
    routeDefinition(MATERIAL_TO_ROUGH),
    routeDefinition(MATERIAL_TO_ROUGH)};
constexpr RouteDefinition ROUTE_ROUGH_TO_STORAGE[BATCH_COUNT] = {
    routeDefinition(ROUGH_TO_STORAGE),
    routeDefinition(ROUGH_TO_STORAGE)};
constexpr RouteDefinition ROUTE_STORAGE_TO_MATERIAL_SECOND =
    routeDefinition(STORAGE_TO_MATERIAL_SECOND);
constexpr RouteDefinition ROUTE_STORAGE_TO_HOME =
    routeDefinition(STORAGE_TO_HOME);
} // namespace mission_routes
