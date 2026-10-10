#pragma once

#include "RouteExecutor.h"
#include "field_config.h"
#include "vision_config.h"

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

    // 途经点，后面必须紧跟fastTo。
    constexpr RouteAction throughTo(const FieldPoint &point)
    {
        return routeMoveThrough(point.xMm, point.yMm);
    }

    constexpr RouteAction driftTo(const FieldPose &pose)
    {
        return routeMoveToPose(
            pose.position.xMm,
            pose.position.yMm,
            pose.yawDeg);
    }

    /*
     * 区域基准点。
     *
     * 长距离直线使用快速档，脉冲中断按剩余行程自动减速。
     * 路线直接到工位基准点，再由工位视觉修正毫米级误差。
     */
    constexpr FieldPose LOWER_HOME_ANCHOR = {
        {field_config::LOWER_RIGHT_START.xMm,
         field_config::LOWER_RIGHT_START.yMm},
        -180.0F};
    constexpr FieldPose UPPER_HOME_ANCHOR = {
        {field_config::UPPER_RIGHT_START.xMm,
         field_config::UPPER_RIGHT_START.yMm},
        -180.0F};
    // 回启停区停车点的标定偏移：只移动路线终点，不改变启停区基准点。
    // 世界坐标：+x为初始左侧，+y为初始车头方向。
    constexpr float LOWER_HOME_STOP_OFFSET_X_MM = -50.0F;
    constexpr float LOWER_HOME_STOP_OFFSET_Y_MM = -10.0F;
    constexpr float UPPER_HOME_STOP_OFFSET_X_MM = -50.0F;
    constexpr float UPPER_HOME_STOP_OFFSET_Y_MM = 10.0F;
    constexpr FieldPoint LOWER_HOME_STOP = {
        LOWER_HOME_ANCHOR.position.xMm + LOWER_HOME_STOP_OFFSET_X_MM,
        LOWER_HOME_ANCHOR.position.yMm + LOWER_HOME_STOP_OFFSET_Y_MM};
    constexpr FieldPoint UPPER_HOME_STOP = {
        UPPER_HOME_ANCHOR.position.xMm + UPPER_HOME_STOP_OFFSET_X_MM,
        UPPER_HOME_ANCHOR.position.yMm + UPPER_HOME_STOP_OFFSET_Y_MM};
    constexpr FieldPose MATERIAL_ANCHOR = {{1200.0F, 2100.0F}, 90.0F};
    // 取完料后的归位转向点：x与原料区轴线一致，y比取料锚点小以避开物料盘。
    constexpr FieldPoint MATERIAL_TURN_POINT = {
        MATERIAL_ANCHOR.position.xMm, 2100.0F};
    // 粗加工区直接使用视觉实测作业点，到站和离站都不再产生对准差值。
    // 进站偏移：只移动漂移到位点，不改变对准成功后重写的世界坐标。
    constexpr float ROUGH_APPROACH_OFFSET_X_MM = 90.0F;
    constexpr float ROUGH_APPROACH_OFFSET_Y_MM = 50.0F;
    constexpr FieldPose ROUGH_ANCHOR = {
        {vision_config::ROUGH_ALIGNED_WORLD_X_MM +
             ROUGH_APPROACH_OFFSET_X_MM,
         vision_config::ROUGH_ALIGNED_WORLD_Y_MM +
             ROUGH_APPROACH_OFFSET_Y_MM},
        -90.0F};
    constexpr FieldPose STORAGE_ANCHOR = {{2040.0F, 1220.0F}, -180.0F};
    // 粗加工区->暂存区漂移到位点的标定偏移：只移动该漂移终点，不改变
    // STORAGE_ANCHOR（其他路线与测试共用）和对准成功后重写的世界坐标，
    // 也不影响原料区->粗加工区路线。
    constexpr float STORAGE_APPROACH_OFFSET_X_MM = 0.0F;
    constexpr float STORAGE_APPROACH_OFFSET_Y_MM = 0.0F;
    constexpr FieldPose STORAGE_DRIFT_TARGET = {
        {STORAGE_ANCHOR.position.xMm + STORAGE_APPROACH_OFFSET_X_MM,
         STORAGE_ANCHOR.position.yMm + STORAGE_APPROACH_OFFSET_Y_MM},
        STORAGE_ANCHOR.yawDeg};

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
        STORAGE_DRIFT_TARGET.position.xMm,
        ROUGH_ANCHOR.position.yMm};
    constexpr FieldPoint STORAGE_MATERIAL_CORNER = {
        STORAGE_ANCHOR.position.xMm,
        MATERIAL_ANCHOR.position.yMm};

    // 原料区轴线与粗加工区作业点y的交点，界定边走边转的扫掠范围。
    constexpr FieldPoint ROUGH_AXIS_END = {
        MATERIAL_ANCHOR.position.xMm,
        ROUGH_ANCHOR.position.yMm};

    static_assert(
        field_config::hasRotationClearance(
            ROUGH_STORAGE_CORNER.xMm,
            ROUGH_STORAGE_CORNER.yMm) &&
            field_config::hasRotationClearance(
                STORAGE_DRIFT_TARGET.position.xMm,
                STORAGE_DRIFT_TARGET.position.yMm) &&
            field_config::hasRotationClearance(
                STORAGE_MATERIAL_CORNER.xMm,
                STORAGE_MATERIAL_CORNER.yMm),
        "map corner has insufficient chassis rotation clearance");

    /*
     * 原料区和粗加工区保留边界余量供整车旋转。原料区到粗加工区边走
     * 边转，路径落在MATERIAL_ANCHOR、ROUGH_AXIS_END、ROUGH_ANCHOR围成的
     * 三角形内；余量区域是矩形，三个顶点都满足即整段满足，与转向窗口
     * 长度无关。
     */
    static_assert(
        field_config::hasRotationClearance(
            MATERIAL_ANCHOR.position.xMm,
            MATERIAL_ANCHOR.position.yMm) &&
            field_config::hasRotationClearance(
                MATERIAL_TURN_POINT.xMm,
                MATERIAL_TURN_POINT.yMm) &&
            field_config::hasRotationClearance(
                ROUGH_AXIS_END.xMm,
                ROUGH_AXIS_END.yMm) &&
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
        routeRotateTo(MATERIAL_ANCHOR.yawDeg),
    };

    // 原料区原地转-180度后一条指令到粗加工区：沿原料区x轴线快速直行，
    // 距作业点POSE_MOVE_TURN_WINDOW_MM处边转-90度边并入x偏差，中途不停车。
    constexpr RouteAction MATERIAL_TO_ROUGH[] = {
        fastTo(MATERIAL_TURN_POINT),
        routeRotateTo(MATERIAL_TO_ROUGH_TRAVEL_YAW_DEG),
        driftTo(ROUGH_ANCHOR),
        routeRotateTo(ROUGH_ANCHOR.yawDeg),
    };

    // 粗加工区沿x轴线直行到地图角点，停车原地转到暂存区航向，
    // 再沿暂存区轴线直行进站。
    constexpr RouteAction ROUGH_TO_STORAGE[] = {
        fastTo(ROUGH_STORAGE_CORNER),
        routeRotateTo(STORAGE_ANCHOR.yawDeg),
        fastTo(STORAGE_DRIFT_TARGET.position),
        routeRotateTo(STORAGE_ANCHOR.yawDeg),
    };

    constexpr RouteAction STORAGE_TO_MATERIAL_SECOND[] = {
        fastTo(STORAGE_MATERIAL_CORNER),
        routeRotateTo(MATERIAL_ANCHOR.yawDeg),
        fastTo(MATERIAL_ANCHOR.position),
        routeRotateTo(MATERIAL_ANCHOR.yawDeg),
    };

    /*
     * 暂存区 -> 右下启停区，保持-180度航向，一条连续平移：沿中心线
     * 直行，途经右侧中心线点不停车圆滑拐入启停区。暂存区与中心线
     * y只差20mm，不再单独经过TURN_CENTER。
     */
    constexpr RouteAction STORAGE_TO_LOWER_HOME[] = {
        throughTo(HOME_CENTERLINE),
        fastTo(LOWER_HOME_STOP),
    };

    // 暂存区 -> 右上启停区，与右下路线共用右侧中心线拐点。
    constexpr RouteAction STORAGE_TO_UPPER_HOME[] = {
        throughTo(HOME_CENTERLINE),
        fastTo(UPPER_HOME_STOP),
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
