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

    /*
     * 区域基准点。
     *
     * 长距离直线使用快速档，脉冲中断按剩余行程自动减速。
     * 原料区到粗加工区保留最后50mm精确靠近；其他路线直接到工位
     * 基准点，再由工位视觉修正毫米级误差。
     */
    constexpr FieldPose LOWER_HOME_ANCHOR = {
        {field_config::LOWER_RIGHT_START.xMm,
         field_config::LOWER_RIGHT_START.yMm},
        -180.0F};
    constexpr FieldPose UPPER_HOME_ANCHOR = {
        {field_config::UPPER_RIGHT_START.xMm,
         field_config::UPPER_RIGHT_START.yMm},
        -180.0F};
    constexpr FieldPose MATERIAL_ANCHOR = {{1200.0F, 2150.0F}, 90.0F};
    constexpr FieldPose ROUGH_ANCHOR = {{1220.0F, 300.0F}, -90.0F};
    constexpr FieldPose STORAGE_ANCHOR = {{2100.0F, 1220.0F}, -180.0F};

    // 原料区到粗加工区沿世界-y方向行驶，车头需先转向-180度。
    constexpr float MATERIAL_TO_ROUGH_TRAVEL_YAW_DEG = -180.0F;

    // static_assert(
    //     MATERIAL_ANCHOR.position.xMm ==
    //             field_config::TURN_CENTER_X_MM &&
    //         ROUGH_ANCHOR.position.xMm ==
    //             field_config::TURN_CENTER_X_MM &&
    //         STORAGE_ANCHOR.position.yMm ==
    //             field_config::TURN_CENTER_Y_MM,
    //     "fixed station anchors must remain on the 1200 mm center axes");

    // 场地唯一允许改变航向的位置。
    constexpr FieldPoint TURN_CENTER = {
        field_config::TURN_CENTER_X_MM,
        field_config::TURN_CENTER_Y_MM};

    // 仅保留形成安全直角路线所需的右侧中心线点。
    constexpr FieldPoint HOME_CENTERLINE = {
        field_config::LOWER_RIGHT_START.xMm,
        field_config::TURN_CENTER_Y_MM};

    /*
     * 右侧两处地图角点。路线先沿当前工位轴线直行到角点，停车
     * 完成旋转后再沿下一工位轴线进入目标区，避免绕行场地中心。
     */
    constexpr FieldPoint ROUGH_STORAGE_CORNER = {
        STORAGE_ANCHOR.position.xMm,
        ROUGH_ANCHOR.position.yMm};
    constexpr FieldPoint STORAGE_MATERIAL_CORNER = {
        STORAGE_ANCHOR.position.xMm,
        MATERIAL_ANCHOR.position.yMm};

    /*
     * 长距离转场保持在场地轴线上，距工位50 mm处停车。完成作业航向
     * 旋转后，再用精确档靠近固定点，降低高速停车误差和麦轮横滑。
     */
    constexpr float STATION_APPROACH_DISTANCE_MM = 50.0F;
    constexpr FieldPoint ROUGH_APPROACH = {
        MATERIAL_ANCHOR.position.xMm,
        ROUGH_ANCHOR.position.yMm + STATION_APPROACH_DISTANCE_MM};

    static_assert(
        field_config::hasRotationClearance(
            ROUGH_STORAGE_CORNER.xMm,
            ROUGH_STORAGE_CORNER.yMm) &&
            field_config::hasRotationClearance(
                STORAGE_MATERIAL_CORNER.xMm,
                STORAGE_MATERIAL_CORNER.yMm),
        "map corner has insufficient chassis rotation clearance");

    /*
     * 原料区和粗加工区分别保留约100 mm、130 mm的边界余量供整车
     * 原地旋转；点位调整时禁止把旋转中心移入底盘扫掠边界。
     */
    static_assert(
        field_config::hasRotationClearance(
            MATERIAL_ANCHOR.position.xMm,
            MATERIAL_ANCHOR.position.yMm) &&
            field_config::hasRotationClearance(
                ROUGH_ANCHOR.position.xMm,
                ROUGH_ANCHOR.position.yMm),
        "material-to-rough endpoint has insufficient rotation clearance");

    // 启停区 -> 扫码区，扫码区与中心转向点重合。
    constexpr RouteAction TO_SCAN[] = {
        fastTo(HOME_CENTERLINE),
        fastTo(TURN_CENTER),
    };

    // 扫码区 -> 原料区：确认位于中心后才转向。
    constexpr RouteAction SCAN_TO_MATERIAL[] = {
        routeRotateTo(MATERIAL_ANCHOR.yawDeg),
        fastTo(MATERIAL_ANCHOR.position),
    };

    // 原料区先转向，沿x=1200轴线快速直行；转到作业航向后精确靠近。
    constexpr RouteAction MATERIAL_TO_ROUGH[] = {
        fastTo(MATERIAL_ANCHOR.position),
        routeRotateTo(MATERIAL_TO_ROUGH_TRAVEL_YAW_DEG),
        fastTo(ROUGH_APPROACH),
        routeRotateTo(ROUGH_ANCHOR.yawDeg),
        preciseTo(ROUGH_ANCHOR.position),
    };

    // 粗加工区 -> 右下地图角点旋转 -> 暂存区。
    constexpr RouteAction ROUGH_TO_STORAGE[] = {
        fastTo(ROUGH_STORAGE_CORNER),
        routeRotateTo(STORAGE_ANCHOR.yawDeg),
        fastTo(STORAGE_ANCHOR.position),
    };

    // 暂存区 -> 右上地图角点旋转 -> 原料区（第二轮）。
    constexpr RouteAction STORAGE_TO_MATERIAL_SECOND[] = {
        fastTo(STORAGE_MATERIAL_CORNER),
        routeRotateTo(MATERIAL_ANCHOR.yawDeg),
        fastTo(MATERIAL_ANCHOR.position),
    };

    // 暂存区 -> 右下启停区，保持-180度航向，不在终点旋转。
    constexpr RouteAction STORAGE_TO_LOWER_HOME[] = {
        fastTo(TURN_CENTER),
        fastTo(HOME_CENTERLINE),
        fastTo(LOWER_HOME_ANCHOR.position),
    };

    // 暂存区 -> 右上启停区，与右下路线共用右侧中心线。
    constexpr RouteAction STORAGE_TO_UPPER_HOME[] = {
        fastTo(TURN_CENTER),
        fastTo(HOME_CENTERLINE),
        fastTo(UPPER_HOME_ANCHOR.position),
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
    constexpr RouteDefinition ROUTE_STORAGE_TO_HOME[] = {
        routeDefinition(STORAGE_TO_LOWER_HOME),
        routeDefinition(STORAGE_TO_UPPER_HOME)};

    constexpr RouteDefinition storageToHomeRoute(StartZone zone)
    {
        return ROUTE_STORAGE_TO_HOME[zone == StartZone::UpperRight ? 1 : 0];
    }
} // namespace mission_routes
