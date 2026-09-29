#pragma once

#include "GraspVisionPorts.h"
#include "MaixVisionService.h"

/**
 * @brief MaixPro抓取视觉适配器。
 *
 * 将共享视觉服务的检测帧转换成机械臂使用的通用观测接口。
 */
class MaixProGraspVision : public IGraspVisionProvider
{
public:
    explicit MaixProGraspVision(maixcam::MaixVisionService &vision);

    void begin() override;
    bool startTracking(uint8_t color) override;
    void update() override;
    bool takeObservation(GraspObservation &observation) override;
    void stop() override;
    bool faulted() const override;

private:
    maixcam::MaixVisionService &_vision;
    maixcam::MaixVisionService::Token _token = 0;
    uint32_t _cursor = 0;
    GraspObservation _observation = {};
    uint8_t _targetColor = 0;
    bool _observationReady = false;
};
