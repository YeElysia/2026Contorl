#pragma once

#include "RouteExecutor.h"

namespace mission_routes
{
/**
 * @brief 场地中的世界坐标点，单位为毫米。
 *
 * 原点为启动位置，+x指向小车初始左侧，+y指向初始车头。
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

constexpr RouteAction face(const FieldPose &pose)
{
    return routeRotateTo(pose.yawDeg);
}

/*
 * 区域基准点。
 *
 * 每个工位任务完成后，离站路线首先以精确档返回对应基准点，
 * 清除视觉微调造成的坐标偏移，然后才执行固定长距离路线。
 */
constexpr FieldPose SCAN_ANCHOR = {{1015.0F, 1050.0F}, 0.0F};
constexpr FieldPose MATERIAL_ANCHOR = {{1015.0F, 1990.0F}, 90.0F};
constexpr FieldPose ROUGH_ANCHOR = {{1045.0F, 160.0F}, -90.0F};
constexpr FieldPose STORAGE_ANCHOR = {{1915.0F, 1090.0F}, -180.0F};
constexpr FieldPose HOME_ANCHOR = {{0.0F, 0.0F}, -180.0F};

// 启停区 -> 扫码区
constexpr FieldPoint START_FORWARD_WAYPOINT = {0.0F, 1050.0F};
constexpr RouteAction TO_SCAN[] = {
    fastTo(START_FORWARD_WAYPOINT),
    fastTo(SCAN_ANCHOR.position),
};

// 扫码区 -> 原料区（第一轮）
constexpr RouteAction SCAN_TO_MATERIAL[] = {
    preciseTo(SCAN_ANCHOR),
    face(SCAN_ANCHOR),
    routeRotateTo(90.0F),
    fastTo(MATERIAL_ANCHOR.position),
};

// 原料区 -> 粗加工区（第一轮）
constexpr FieldPoint MATERIAL_FIRST_CORNER = {1015.0F, 1010.0F};
constexpr FieldPoint ROUGH_FIRST_APPROACH = {1015.0F, 160.0F};
constexpr RouteAction MATERIAL_TO_ROUGH_FIRST[] = {
    preciseTo(MATERIAL_ANCHOR),
    face(MATERIAL_ANCHOR),
    fastTo(MATERIAL_FIRST_CORNER),
    routeRotateTo(-90.0F),
    fastTo(ROUGH_FIRST_APPROACH),
    preciseTo(ROUGH_ANCHOR),
};

// 粗加工区 -> 暂存区（第一轮）
constexpr FieldPoint ROUGH_FIRST_DEPARTURE = {1015.0F, 160.0F};
constexpr FieldPoint STORAGE_FIRST_CORNER = {1015.0F, 1040.0F};
constexpr FieldPoint STORAGE_FIRST_APPROACH = {1915.0F, 1040.0F};
constexpr RouteAction ROUGH_TO_STORAGE_FIRST[] = {
    preciseTo(ROUGH_ANCHOR),
    face(ROUGH_ANCHOR),
    preciseTo(ROUGH_FIRST_DEPARTURE),
    fastTo(STORAGE_FIRST_CORNER),
    routeRotateTo(-180.0F),
    fastTo(STORAGE_FIRST_APPROACH),
    preciseTo(STORAGE_ANCHOR),
};

// 暂存区 -> 原料区（第二轮）
constexpr FieldPoint STORAGE_SECOND_DEPARTURE = {1915.0F, 1040.0F};
constexpr FieldPoint MATERIAL_SECOND_CORNER = {1035.0F, 1040.0F};
constexpr FieldPoint MATERIAL_SECOND_APPROACH = {1035.0F, 1990.0F};
constexpr RouteAction STORAGE_TO_MATERIAL_SECOND[] = {
    preciseTo(STORAGE_ANCHOR),
    face(STORAGE_ANCHOR),
    preciseTo(STORAGE_SECOND_DEPARTURE),
    fastTo(MATERIAL_SECOND_CORNER),
    routeRotateTo(90.0F),
    fastTo(MATERIAL_SECOND_APPROACH),
    preciseTo(MATERIAL_ANCHOR),
};

// 原料区 -> 粗加工区（第二轮）
constexpr FieldPoint MATERIAL_SECOND_DEPARTURE = {1045.0F, 1990.0F};
constexpr FieldPoint ROUGH_SECOND_APPROACH = {1045.0F, 150.0F};
constexpr RouteAction MATERIAL_TO_ROUGH_SECOND[] = {
    preciseTo(MATERIAL_ANCHOR),
    face(MATERIAL_ANCHOR),
    preciseTo(MATERIAL_SECOND_DEPARTURE),
    fastTo(ROUGH_SECOND_APPROACH),
    routeRotateTo(-90.0F),
    preciseTo(ROUGH_ANCHOR),
};

// 粗加工区 -> 暂存区（第二轮）
constexpr FieldPoint ROUGH_SECOND_CORNER = {1045.0F, 1060.0F};
constexpr FieldPoint STORAGE_SECOND_APPROACH_RIGHT = {1945.0F, 1060.0F};
constexpr FieldPoint STORAGE_SECOND_APPROACH = {1915.0F, 1060.0F};
constexpr RouteAction ROUGH_TO_STORAGE_SECOND[] = {
    preciseTo(ROUGH_ANCHOR),
    face(ROUGH_ANCHOR),
    fastTo(ROUGH_SECOND_CORNER),
    routeRotateTo(-180.0F),
    fastTo(STORAGE_SECOND_APPROACH_RIGHT),
    preciseTo(STORAGE_SECOND_APPROACH),
    preciseTo(STORAGE_ANCHOR),
};

// 暂存区 -> 启停区
constexpr FieldPoint HOME_DEPARTURE = {1915.0F, 1070.0F};
constexpr FieldPoint HOME_FIRST_CORNER = {1915.0F, 220.0F};
constexpr FieldPoint HOME_SECOND_CORNER = {-35.0F, 220.0F};
constexpr FieldPoint HOME_APPROACH = {-35.0F, -20.0F};
constexpr RouteAction STORAGE_TO_HOME[] = {
    preciseTo(STORAGE_ANCHOR),
    face(STORAGE_ANCHOR),
    preciseTo(HOME_DEPARTURE),
    fastTo(HOME_FIRST_CORNER),
    fastTo(HOME_SECOND_CORNER),
    preciseTo(HOME_APPROACH),
    preciseTo(HOME_ANCHOR),
    face(HOME_ANCHOR),
};

constexpr RouteDefinition ROUTE_TO_SCAN = routeDefinition(TO_SCAN);
constexpr RouteDefinition ROUTE_SCAN_TO_MATERIAL =
    routeDefinition(SCAN_TO_MATERIAL);
constexpr RouteDefinition ROUTE_MATERIAL_TO_ROUGH[BATCH_COUNT] = {
    routeDefinition(MATERIAL_TO_ROUGH_FIRST),
    routeDefinition(MATERIAL_TO_ROUGH_SECOND)};
constexpr RouteDefinition ROUTE_ROUGH_TO_STORAGE[BATCH_COUNT] = {
    routeDefinition(ROUGH_TO_STORAGE_FIRST),
    routeDefinition(ROUGH_TO_STORAGE_SECOND)};
constexpr RouteDefinition ROUTE_STORAGE_TO_MATERIAL_SECOND =
    routeDefinition(STORAGE_TO_MATERIAL_SECOND);
constexpr RouteDefinition ROUTE_STORAGE_TO_HOME =
    routeDefinition(STORAGE_TO_HOME);
} // namespace mission_routes
