#pragma once

#include <Arduino.h>

#include "ChassisControl.h"
#include "MissionPorts.h"

enum class RouteActionType : uint8_t
{
    MoveTo,
    RotateTo
};

enum class MotionProfile : uint8_t
{
    Fast,
    Precise
};

/**
 * @brief 一条不可变的底盘动作。
 *
 * MoveTo使用世界绝对坐标xMm/yMm；RotateTo只使用世界yawDeg。
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
 * 每次只向ChassisControl下发一个动作，确认底盘空闲后才进入下一步。
 * 它不知道任务区、二维码或机械臂，只负责可靠执行一段路线。
 */
class RouteExecutor
{
public:
    explicit RouteExecutor(ChassisControl &chassis);

    bool start(RouteDefinition route);
    void update();
    void cancel();

    AsyncResult result() const;

private:
    ChassisControl &_chassis;
    const RouteAction *_actions = nullptr;
    size_t _count = 0;
    size_t _index = 0;
    AsyncResult _result = AsyncResult::Idle;
};
