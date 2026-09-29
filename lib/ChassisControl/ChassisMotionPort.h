#pragma once

#include "ChassisControl.h"

/**
 * @brief 以固定身份访问底盘的受限接口。
 *
 * 每条运动命令先申请底盘所有权；底盘被其他模块占用时命令被拒绝，
 * faultMessage()会给出当前拥有者。stop()只停止自己下发的运动。
 * 模块完成、失败或取消时必须调用release()。
 */
class ChassisMotionPort
{
public:
    ChassisMotionPort(ChassisControl &chassis, ChassisOwner owner);

    bool moveWorldTo(
        float targetWorldXMm,
        float targetWorldYMm,
        float maxRpm,
        float accelerationRpmPerS);
    bool rotateWorldTo(float worldYawDeg);
    bool moveBodyRelative(
        float forwardMm,
        float rightMm,
        float maxRpm,
        float accelerationRpmPerS);
    bool moveBodyRelativeTracking(
        float forwardMm,
        float rightMm,
        float maxRpm,
        float accelerationRpmPerS);
    bool correctWorldPosition(float worldXmm, float worldYmm);

    void stop();
    void release();

    bool busy() const;
    bool faulted() const;
    ChassisControl::Pose2D worldPose() const;
    const char *faultMessage() const;

private:
    ChassisControl &_chassis;
    ChassisOwner _owner;
    char _denied[48] = {};

    bool acquire();
};
