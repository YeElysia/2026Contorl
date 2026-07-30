#pragma once

#include <Arduino.h>

#include "ChassisControl.h"
#include "MissionPorts.h"

enum class RouteActionType : uint8_t
{
    Move,
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
 * Move使用forwardMm/rightMm；RotateTo只使用yawDeg。
 */
struct RouteAction
{
    RouteActionType type;
    float forwardMm;
    float rightMm;
    float yawDeg;
    MotionProfile profile;
};

constexpr RouteAction routeMove(float forwardMm, float rightMm)
{
    return {
        RouteActionType::Move,
        forwardMm,
        rightMm,
        0.0f,
        MotionProfile::Fast};
}

constexpr RouteAction routePreciseMove(
    float forwardMm,
    float rightMm)
{
    return {
        RouteActionType::Move,
        forwardMm,
        rightMm,
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

    bool start(const RouteAction *actions, size_t count);
    void update();
    void cancel();

    AsyncResult result() const;
    size_t actionIndex() const;

private:
    ChassisControl &_chassis;
    const RouteAction *_actions = nullptr;
    size_t _count = 0;
    size_t _index = 0;
    AsyncResult _result = AsyncResult::Idle;
};
