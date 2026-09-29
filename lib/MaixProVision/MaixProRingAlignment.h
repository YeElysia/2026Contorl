#pragma once

#include "ChassisMotionPort.h"
#include "MaixVisionService.h"
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
        bool worldPoseCorrected = false;
        float worldCorrectionMm = 0.0F;
        uint8_t targetMode = maixcam::MODE_IDLE;
        uint8_t targetSelector = 0;
        uint8_t lastMode = maixcam::MODE_IDLE;
        uint8_t lastTargetId = 0;
        uint32_t observations = 0;
        uint32_t ignoredObservations = 0;
    };

    MaixProRingAlignment(
        maixcam::MaixVisionService &vision,
        ChassisMotionPort &chassis);

    bool start(const AlignmentRequest &request) override;
    void update() override;
    AsyncResult result() const override;
    void cancel() override;
    const DebugState &debugState() const;

private:
    maixcam::MaixVisionService &_vision;
    ChassisMotionPort &_chassis;
    maixcam::MaixVisionService::Token _visionToken = 0;
    uint32_t _visionCursor = 0;

    AsyncResult _result = AsyncResult::Idle;
    bool _movePending = false;
    bool _allowRingFallback = false;
    bool _ringFallbackActive = false;
    bool _targetSeen = false;
    uint8_t _stableFrames = 0;
    uint32_t _startedMs = 0;
    uint32_t _lastObservationMs = 0;
    uint8_t _targetMode = maixcam::MODE_IDLE;
    uint8_t _targetSelector = 0;
    Station _station = Station::Material;
    DebugState _debug;

    void requestVision();
    void stopVision();
    void fail();
    void activateStorageRingFallback();
    bool correctWorldPositionFromLandmark();
    static float clampMagnitude(float value, float limit);
};
