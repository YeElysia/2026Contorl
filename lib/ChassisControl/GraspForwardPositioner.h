#pragma once

#include "ChassisMotionPort.h"
#include "GraspMotionPorts.h"

/**
 * @brief 原料区车体坐标系微调执行器。
 */
class GraspForwardPositioner : public IGraspForwardPositioner
{
public:
    explicit GraspForwardPositioner(ChassisMotionPort &chassis);

    bool moveBodyRelative(
        float forwardMm,
        float rightMm) override;
    bool busy() const override;
    bool faulted() const override;
    void stop() override;
    void release() override;

private:
    ChassisMotionPort &_chassis;
};
