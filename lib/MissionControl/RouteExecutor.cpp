#include "RouteExecutor.h"

#include "chassis_config.h"

RouteExecutor::RouteExecutor(ChassisControl &chassis)
    : _chassis(chassis)
{
}

bool RouteExecutor::start(
    const RouteAction *actions,
    size_t count)
{
    if (_result == AsyncResult::Running ||
        _chassis.busy() ||
        _chassis.state() == ChassisControl::State::Fault ||
        (actions == nullptr && count != 0))
    {
        return false;
    }

    _actions = actions;
    _count = count;
    _index = 0;
    _result =
        count == 0 ? AsyncResult::Succeeded : AsyncResult::Running;
    return true;
}

void RouteExecutor::update()
{
    if (_result != AsyncResult::Running)
        return;

    if (_chassis.state() == ChassisControl::State::Fault)
    {
        _result = AsyncResult::Failed;
        return;
    }

    if (_chassis.busy())
        return;

    if (_index >= _count)
    {
        _result = AsyncResult::Succeeded;
        return;
    }

    const RouteAction &action = _actions[_index];
    bool accepted = false;

    switch (action.type)
    {
    case RouteActionType::Move:
    {
        float maximumRpm = chassis_config::DRIVE_RPM;
        float accelerationRpmPerS =
            chassis_config::DRIVE_ACCEL_RPM_PER_S;

        if (action.profile == MotionProfile::Precise)
        {
            maximumRpm = chassis_config::PRECISE_DRIVE_RPM;
            accelerationRpmPerS =
                chassis_config::PRECISE_DRIVE_ACCEL_RPM_PER_S;
        }

        accepted = _chassis.moveRelative(
            action.forwardMm,
            action.rightMm,
            maximumRpm,
            accelerationRpmPerS);
        break;
    }

    case RouteActionType::RotateTo:
        accepted = _chassis.rotateTo(action.yawDeg);
        break;
    }

    if (accepted)
        ++_index;
    else
        _result = AsyncResult::Failed;
}

void RouteExecutor::cancel()
{
    if (_result == AsyncResult::Running)
        _chassis.stop();

    _actions = nullptr;
    _count = 0;
    _index = 0;
    _result = AsyncResult::Idle;
}

AsyncResult RouteExecutor::result() const
{
    return _result;
}

size_t RouteExecutor::actionIndex() const
{
    return _index;
}
