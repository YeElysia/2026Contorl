#pragma once

#include "GraspVisionPorts.h"
#include "MaixCamV2.h"

/**
 * @brief MaixPro抓取视觉适配器。
 *
 * 将MaixCAM V2协议转换成机械臂使用的通用观测接口。
 */
class MaixProGraspVision : public IGraspVisionProvider
{
public:
    explicit MaixProGraspVision(maixcam::MaixCamV2 &camera);

    void begin() override;
    bool startTracking(uint8_t color) override;
    void update() override;
    bool takeObservation(GraspObservation &observation) override;
    void stop() override;
    bool faulted() const override;

private:
    maixcam::MaixCamV2 &_camera;
    GraspObservation _observation = {};
    uint8_t _targetColor = 0;
    bool _observationReady = false;
    bool _active = false;
    bool _faulted = false;
    bool _begun = false;
};
