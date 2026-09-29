#include "ChassisMotionPort.h"

#include <stdio.h>

ChassisMotionPort::ChassisMotionPort(
    ChassisControl &chassis,
    ChassisOwner owner)
    : _chassis(chassis),
      _owner(owner)
{
}

bool ChassisMotionPort::moveWorldTo(
    float targetWorldXMm,
    float targetWorldYMm,
    float maxRpm,
    float accelerationRpmPerS)
{
    return acquire() &&
           _chassis.moveWorldTo(
               targetWorldXMm,
               targetWorldYMm,
               maxRpm,
               accelerationRpmPerS);
}

bool ChassisMotionPort::rotateWorldTo(float worldYawDeg)
{
    return acquire() && _chassis.rotateWorldTo(worldYawDeg);
}

bool ChassisMotionPort::moveBodyRelative(
    float forwardMm,
    float rightMm,
    float maxRpm,
    float accelerationRpmPerS)
{
    return acquire() &&
           _chassis.moveBodyRelative(
               forwardMm,
               rightMm,
               maxRpm,
               accelerationRpmPerS);
}

bool ChassisMotionPort::moveBodyRelativeTracking(
    float forwardMm,
    float rightMm,
    float maxRpm,
    float accelerationRpmPerS)
{
    return acquire() &&
           _chassis.moveBodyRelativeTracking(
               forwardMm,
               rightMm,
               maxRpm,
               accelerationRpmPerS);
}

bool ChassisMotionPort::correctWorldPosition(
    float worldXmm,
    float worldYmm)
{
    return acquire() &&
           _chassis.correctWorldPosition(worldXmm, worldYmm);
}

void ChassisMotionPort::stop()
{
    if (_chassis.owner() == _owner && _chassis.busy())
        _chassis.stop();
}

void ChassisMotionPort::release()
{
    _chassis.release(_owner);
}

bool ChassisMotionPort::busy() const
{
    return _chassis.busy();
}

bool ChassisMotionPort::faulted() const
{
    return _chassis.state() == ChassisControl::State::Fault;
}

ChassisControl::Pose2D ChassisMotionPort::worldPose() const
{
    return _chassis.worldPose();
}

const char *ChassisMotionPort::faultMessage() const
{
    return _denied[0] != '\0' ? _denied : _chassis.faultMessage();
}

bool ChassisMotionPort::acquire()
{
    if (_chassis.acquire(_owner))
    {
        _denied[0] = '\0';
        return true;
    }

    snprintf(
        _denied,
        sizeof(_denied),
        "chassis owned by %s",
        chassisOwnerName(_chassis.owner()));
    return false;
}
