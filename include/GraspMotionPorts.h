#pragma once

/**
 * @brief 原料区抓取允许使用的底盘运动接口。
 *
 * 这里只暴露车体前后方向，机构模块无法提交左右横移命令。
 * 朝向原料转盘的距离误差必须由机械臂伸缩轴补偿。
 */
class IGraspForwardPositioner
{
public:
    virtual ~IGraspForwardPositioner() = default;
    virtual bool moveForward(float distanceMm) = 0;
    virtual bool busy() const = 0;
    virtual bool faulted() const = 0;
    virtual void stop() = 0;
};
