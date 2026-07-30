#pragma once

#include "ChassisControl.h"
#include "GraspMotionPorts.h"

/**
 * @brief 将底盘约束为原料区前后微调执行器。
 *
 * 适配器内部始终把rightMm设为0，从接口和实现两层禁止车辆
 * 在原料区横向靠近转盘。
 */
class GraspForwardPositioner : public IGraspForwardPositioner
{
public:
    explicit GraspForwardPositioner(ChassisControl &chassis);

    bool moveForward(float distanceMm) override;
    bool busy() const override;
    bool faulted() const override;
    void stop() override;

private:
    ChassisControl &_chassis;
};
