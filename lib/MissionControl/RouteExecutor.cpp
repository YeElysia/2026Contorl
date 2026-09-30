#include "RouteExecutor.h"

#include "chassis_config.h"

RouteExecutor::RouteExecutor(ChassisMotionPort &chassis)
    : _chassis(chassis)
{
}

bool RouteExecutor::start(RouteDefinition route)
{
    if (_result == AsyncResult::Running ||
        _chassis.busy() ||
        _chassis.faulted() ||
        (route.actions == nullptr && route.count != 0))
    {
        return false;
    }

    _actions = route.actions;
    _count = route.count;
    _index = 0;
    _result =
        route.count == 0
            ? AsyncResult::Succeeded
            : AsyncResult::Running;
    return true;
}

void RouteExecutor::update()
{
    if (_result != AsyncResult::Running)
        return;

    if (_chassis.faulted())
    {
        finish(AsyncResult::Failed);
        return;
    }

    if (_chassis.busy())
        return;

    if (_index >= _count)
    {
        finish(AsyncResult::Succeeded);
        return;
    }

    const RouteAction &action = _actions[_index];
    bool accepted = false;

    const bool precise = action.profile == MotionProfile::Precise;
    const float maximumRpm =
        precise ? chassis_config::PRECISE_DRIVE_RPM
                : chassis_config::DRIVE_RPM;
    const float accelerationRpmPerS =
        precise ? chassis_config::PRECISE_DRIVE_ACCEL_RPM_PER_S
                : chassis_config::DRIVE_ACCEL_RPM_PER_S;

    switch (action.type)
    {
    case RouteActionType::MoveTo:
        accepted = _chassis.moveWorldTo(
            action.xMm,
            action.yMm,
            maximumRpm,
            accelerationRpmPerS);
        break;

    case RouteActionType::MoveThrough:
        if (_index + 1 < _count &&
            _actions[_index + 1].type == RouteActionType::MoveTo)
        {
            const RouteAction &next = _actions[_index + 1];
            accepted = _chassis.moveWorldThrough(
                action.xMm,
                action.yMm,
                next.xMm,
                next.yMm,
                maximumRpm,
                accelerationRpmPerS);
        }
        break;

    case RouteActionType::RotateTo:
        accepted = _chassis.rotateWorldTo(action.yawDeg);
        break;

    case RouteActionType::MoveToPose:
        accepted = _chassis.moveWorldToPose(
            action.xMm,
            action.yMm,
            action.yawDeg,
            maximumRpm,
            accelerationRpmPerS);
        break;
    }

    if (accepted)
        _index += action.type == RouteActionType::MoveThrough ? 2 : 1;
    else
        finish(AsyncResult::Failed);
}

void RouteExecutor::cancel()
{
    if (_result == AsyncResult::Running)
        _chassis.stop();
    _chassis.release();

    _actions = nullptr;
    _count = 0;
    _index = 0;
    _result = AsyncResult::Idle;
}

AsyncResult RouteExecutor::result() const
{
    return _result;
}

const char *RouteExecutor::faultMessage() const
{
    return _chassis.faultMessage();
}

void RouteExecutor::finish(AsyncResult result)
{
    _chassis.release();
    _result = result;
}
