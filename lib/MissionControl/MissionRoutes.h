#pragma once

#include "RouteExecutor.h"

namespace mission_routes
{
/*
 * 所有底盘点位集中在本文件。
 *
 * 坐标约定：
 *   routeMove(forward, right)
 *   forward > 0 前进，right > 0 右移。
 *
 * 修改比赛点位时只调整这些数组，不修改MissionController。
 */

// 启停区 -> 扫码区
constexpr RouteAction TO_SCAN[] = {
    routeMove(1050.0f, 0.0f),
    routeMove(0.0f, -1015.0f),
};

// 扫码区 -> 原料区（第一轮）
constexpr RouteAction SCAN_TO_MATERIAL[] = {
    routeRotateTo(90.0f),
    routeMove(0.0f, 940.0f),
};

// 原料区 -> 粗加工区（第一轮）
constexpr RouteAction MATERIAL_TO_ROUGH_FIRST[] = {
    routeMove(0.0f, -980.0f),
    routeRotateTo(-90.0f),
    routeMove(0.0f, 850.0f),
    routePreciseMove(-30.0f, 0.0f),
};

// 粗加工区 -> 暂存区（第一轮）
constexpr RouteAction ROUGH_TO_STORAGE_FIRST[] = {
    routePreciseMove(30.0f, 0.0f),
    routeMove(0.0f, -880.0f),
    routeRotateTo(-180.0f),
    routeMove(0.0f, 900.0f),
    routePreciseMove(-50.0f, 0.0f),
};

// 暂存区 -> 原料区（第二轮）
constexpr RouteAction STORAGE_TO_MATERIAL_SECOND[] = {
    routePreciseMove(50.0f, 0.0f),
    routeMove(0.0f, -880.0f),
    routeRotateTo(90.0f),
    routeMove(0.0f, 950.0f),
    routePreciseMove(-30.0f, 0.0f),
};

// 原料区 -> 粗加工区（第二轮）
constexpr RouteAction MATERIAL_TO_ROUGH_SECOND[] = {
    routePreciseMove(30.0f, 0.0f),
    routeMove(0.0f, -1840.0f),
    routeRotateTo(-90.0f),
};

// 粗加工区 -> 暂存区（第二轮）
constexpr RouteAction ROUGH_TO_STORAGE_SECOND[] = {
    routeMove(0.0f, -900.0f),
    routeRotateTo(-180.0f),
    routeMove(0.0f, 900.0f),
    routePreciseMove(-20.0f, 0.0f),
};

// 暂存区 -> 启停区
constexpr RouteAction STORAGE_TO_HOME[] = {
    routePreciseMove(20.0f, 0.0f),
    routeMove(850.0f, 0.0f),
    routeMove(0.0f, -1950.0f),
    routePreciseMove(240.0f, 0.0f),
};

template <size_t N>
constexpr size_t countOf(const RouteAction (&)[N])
{
    return N;
}
} // namespace mission_routes
