#include "GraspForwardPositioner.h"

#include "chassis_config.h"

GraspForwardPositioner::GraspForwardPositioner(
    ChassisControl &chassis)
    : _chassis(chassis)
{
}

bool GraspForwardPositioner::moveBodyRelative(
    float forwardMm,
    float rightMm)
{
    return _chassis.moveBodyRelativeTracking(
        forwardMm,
        rightMm,
        chassis_config::GRASP_TRACK_DRIVE_RPM,
        chassis_config::GRASP_TRACK_ACCEL_RPM_PER_S);
}

bool GraspForwardPositioner::busy() const
{
    return _chassis.busy();
}

bool GraspForwardPositioner::faulted() const
{
    return _chassis.state() == ChassisControl::State::Fault;
}

void GraspForwardPositioner::stop()
{
    if (_chassis.busy())
        _chassis.stop();
}
