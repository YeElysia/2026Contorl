#include "GraspForwardPositioner.h"

#include "chassis_config.h"

GraspForwardPositioner::GraspForwardPositioner(
    ChassisControl &chassis)
    : _chassis(chassis)
{
}

bool GraspForwardPositioner::moveForward(float distanceMm)
{
    return _chassis.moveBodyRelative(
        distanceMm,
        0.0F,
        chassis_config::PRECISE_DRIVE_RPM,
        chassis_config::PRECISE_DRIVE_ACCEL_RPM_PER_S);
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
