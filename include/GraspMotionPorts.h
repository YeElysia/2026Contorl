#pragma once

/**
 * @brief 原料区抓取允许使用的底盘运动接口。
 *
 * 原料区视觉对齐同时暴露车体前后和左右方向。
 * rightMm为正表示右移，为负表示左移。
 */
class IGraspForwardPositioner
{
public:
    virtual ~IGraspForwardPositioner() = default;
    virtual bool moveBodyRelative(
        float forwardMm,
        float rightMm) = 0;
    virtual bool busy() const = 0;
    virtual bool faulted() const = 0;
    virtual void stop() = 0;
    // 本工位任务结束（成功、失败或取消）后交还底盘。
    virtual void release() = 0;
};
