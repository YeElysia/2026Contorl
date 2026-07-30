#pragma once

#include "ChassisControl.h"
#include "MaixCamV2.h"
#include "MissionPorts.h"

/**
 * @brief 粗加工区圆环整车对准器。
 *
 * 本类只在RoughProcessing工位启用MaixPro圆环模式，根据圆心偏差
 * 让底盘做小范围二维平移。每次移动完成后才读取新图像，避免旧帧
 * 反复累加；原料区和暂存区不改变现有流程，直接返回对准成功。
 */
class MaixProRingAlignment : public IAlignmentProvider
{
public:
    MaixProRingAlignment(
        maixcam::MaixCamV2 &camera,
        ChassisControl &chassis);

    bool start(Station station, uint8_t round) override;
    void update() override;
    AsyncResult result() const override;
    void cancel() override;

private:
    maixcam::MaixCamV2 &_camera;
    ChassisControl &_chassis;

    AsyncResult _result = AsyncResult::Idle;
    bool _movePending = false;
    uint8_t _stableFrames = 0;
    uint32_t _startedMs = 0;
    uint32_t _lastObservationMs = 0;

    void fail();
    static float clampMagnitude(float value, float limit);
};
