#pragma once

#include "ChassisControl.h"
#include "GraspVisionPorts.h"
#include "MaixCamV2.h"
#include "MissionPorts.h"

/**
 * @brief 粗加工区和暂存区整车视觉对准器。
 *
 * 粗加工区、第一轮暂存区使用圆环模式；第二轮暂存区使用第一轮
 * 2号位物料的颜色模式。三种情况共用同一底盘闭环，原料区整车
 * 对准保持直通，因为原料抓取有独立的逐物料视觉流程。
 */
class MaixProRingAlignment : public IAlignmentProvider
{
public:
    struct DebugState
    {
        int16_t dx = 0;
        int16_t dy = 0;
        uint8_t quality = 0;
        uint8_t stableFrames = 0;
        float forwardMm = 0.0F;
        float rightMm = 0.0F;
        bool found = false;
        bool hasObservation = false;
        bool movePending = false;
        uint8_t targetMode = maixcam::MODE_IDLE;
        uint8_t targetSelector = 0;
    };

    MaixProRingAlignment(
        maixcam::MaixCamV2 &camera,
        IGraspVisionProvider &graspVision,
        ChassisControl &chassis);

    bool start(const AlignmentRequest &request) override;
    void update() override;
    AsyncResult result() const override;
    void cancel() override;
    const DebugState &debugState() const;

private:
    maixcam::MaixCamV2 &_camera;
    IGraspVisionProvider &_graspVision;
    ChassisControl &_chassis;

    AsyncResult _result = AsyncResult::Idle;
    bool _useGraspVision = false;
    bool _movePending = false;
    uint8_t _stableFrames = 0;
    uint32_t _startedMs = 0;
    uint32_t _lastObservationMs = 0;
    uint8_t _targetMode = maixcam::MODE_IDLE;
    uint8_t _targetSelector = 0;
    DebugState _debug;

    void stopVision();
    void fail();
    static float clampMagnitude(float value, float limit);
};
