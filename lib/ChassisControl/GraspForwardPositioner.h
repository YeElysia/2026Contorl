#pragma once

#include "ChassisControl.h"
#include "GraspMotionPorts.h"

/**
 * @brief 原料区车体坐标系微调执行器。
 */
class GraspForwardPositioner : public IGraspForwardPositioner
{
public:
    explicit GraspForwardPositioner(ChassisControl &chassis);

    bool moveBodyRelative(
        float forwardMm,
        float rightMm) override;
    bool busy() const override;
    bool faulted() const override;
    void stop() override;

private:
    ChassisControl &_chassis;
};
