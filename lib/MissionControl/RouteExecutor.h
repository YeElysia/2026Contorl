#pragma once

#include <Arduino.h>

#include "ChassisMotionPort.h"
#include "MissionPorts.h"

enum class RouteActionType : uint8_t
{
    MoveTo,
    // 途经点，必须紧跟一条MoveTo，两者作为一条连续平移下发。
    MoveThrough,
    RotateTo,
    // 一条指令平移并转向到xMm/yMm/yawDeg，旋转余量需覆盖整段路径。
    MoveToPose
};

enum class MotionProfile : uint8_t
{
    Fast,
    Precise
};

/**
 * @brief 一条不可变的底盘动作。
 *
 * MoveTo/MoveThrough使用世界绝对坐标xMm/yMm；RotateTo只使用世界yawDeg；
 * MoveToPose三者都用。
 */
struct RouteAction
{
    RouteActionType type;
    float xMm;
    float yMm;
    float yawDeg;
    MotionProfile profile;
};

struct RouteDefinition
{
    const RouteAction *actions;
    size_t count;
};

template <size_t N>
constexpr RouteDefinition routeDefinition(
    const RouteAction (&actions)[N])
{
    return {actions, N};
}

constexpr RouteAction routeMoveTo(float xMm, float yMm)
{
    return {
        RouteActionType::MoveTo,
        xMm,
        yMm,
        0.0f,
        MotionProfile::Fast};
}

constexpr RouteAction routePreciseMoveTo(
    float xMm,
    float yMm)
{
    return {
        RouteActionType::MoveTo,
        xMm,
        yMm,
        0.0f,
        MotionProfile::Precise};
}

constexpr RouteAction routeMoveThrough(float xMm, float yMm)
{
    return {
        RouteActionType::MoveThrough,
        xMm,
        yMm,
        0.0f,
        MotionProfile::Fast};
}

constexpr RouteAction routeMoveToPose(
    float xMm,
    float yMm,
    float yawDeg)
{
    return {
        RouteActionType::MoveToPose,
        xMm,
        yMm,
        yawDeg,
        MotionProfile::Fast};
}

constexpr RouteAction routeRotateTo(float yawDeg)
{
    return {
        RouteActionType::RotateTo,
        0.0f,
        0.0f,
        yawDeg,
        MotionProfile::Precise};
}

/**
 * @brief 非阻塞底盘路线执行器。
 *
 * 每次只向底盘下发一个动作（途经点与其后的MoveTo合为一个），
 * 确认底盘空闲后才进入下一步。
 * 路线结束、失败或取消时释放底盘所有权。
 * 它不知道任务区、二维码或机械臂，只负责可靠执行一段路线。
 */
class RouteExecutor
{
public:
    explicit RouteExecutor(ChassisMotionPort &chassis);

    bool start(RouteDefinition route);
    void update();
    void cancel();

    AsyncResult result() const;
    const char *faultMessage() const;

private:
    ChassisMotionPort &_chassis;
    const RouteAction *_actions = nullptr;
    size_t _count = 0;
    size_t _index = 0;
    AsyncResult _result = AsyncResult::Idle;

    void finish(AsyncResult result);
};
