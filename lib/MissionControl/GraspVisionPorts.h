#pragma once

#include <Arduino.h>

/**
 * @brief 单帧抓取目标观测结果。
 *
 * dx、dy沿用MaixPro图像坐标定义，机械臂模块只使用偏差，
 * 不依赖具体相机协议或图像分辨率。
 */
struct GraspObservation
{
    bool found = false;
    int16_t dx = 0;
    int16_t dy = 0;
    uint8_t quality = 0;
    uint32_t receivedMs = 0;
};

/**
 * @brief 抓取视觉接口。
 *
 * 通信解析留在实现类中，机械臂只负责选择颜色和消费观测结果。
 */
class IGraspVisionProvider
{
public:
    virtual ~IGraspVisionProvider() = default;
    virtual void begin() = 0;
    virtual bool startTracking(uint8_t color) = 0;
    virtual void update() = 0;
    virtual bool takeObservation(GraspObservation &observation) = 0;
    virtual void stop() = 0;
    virtual bool faulted() const = 0;
};
