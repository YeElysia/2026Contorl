#include "MaixProRingAlignment.h"

#include "chassis_config.h"
#include "vision_config.h"

#include <math.h>

MaixProRingAlignment::MaixProRingAlignment(
    maixcam::MaixVisionService &vision,
    ChassisMotionPort &chassis)
    : _vision(vision),
      _chassis(chassis)
{
}

bool MaixProRingAlignment::start(
    const AlignmentRequest &request)
{
    if (_result == AsyncResult::Running)
        return false;

    _allowRingFallback = false;
    _ringFallbackActive = false;
    _targetSeen = false;

    // 原料区由机械臂逐物料视觉对准，不执行额外整车对准。
    if (request.station == Station::Material)
    {
        _targetMode = maixcam::MODE_IDLE;
        _targetSelector = 0;
        _debug = {};
        _result = AsyncResult::Succeeded;
        return true;
    }

    if (request.station == Station::RoughProcessing)
    {
        _targetMode = maixcam::MODE_RING;
        _targetSelector = vision_config::ROUGH_RING_ID;
    }
    else if (request.station == Station::Storage &&
             request.round == 0)
    {
        _targetMode = maixcam::MODE_RING;
        _targetSelector =
            vision_config::STORAGE_REFERENCE_RING_ID;
    }
    else if (request.station == Station::Storage &&
             request.round == 1 &&
             request.referenceColor >=
                 static_cast<uint8_t>(MaterialColor::Red) &&
             request.referenceColor <=
                 static_cast<uint8_t>(MaterialColor::Green))
    {
        _allowRingFallback = true;
        _targetMode = maixcam::MODE_GRAB;
        _targetSelector = request.referenceColor;
    }
    else
    {
        return false;
    }

    if (_chassis.busy() || _chassis.faulted())
    {
        return false;
    }

    const uint32_t now = millis();
    _station = request.station;
    _movePending = false;
    _stableFrames = 0;
    _startedMs = now;
    _lastObservationMs = 0;
    _debug = {};
    _debug.targetMode = _targetMode;
    _debug.targetSelector = _targetSelector;
    _result = AsyncResult::Running;
    requestVision();
    return true;
}

void MaixProRingAlignment::update()
{
    using namespace chassis_config;
    using namespace vision_config;

    if (_result != AsyncResult::Running)
        return;

    const maixcam::MaixVisionService::Status visionStatus =
        _vision.status(_visionToken);
    if (visionStatus != maixcam::MaixVisionService::Status::Pending &&
        visionStatus != maixcam::MaixVisionService::Status::Active)
    {
        fail();
        return;
    }

    const uint32_t now = millis();
    if (now - _startedMs >= RING_ALIGNMENT_TIMEOUT_MS)
    {
        fail();
        return;
    }

    if (_chassis.faulted())
    {
        fail();
        return;
    }

    if (_movePending)
    {
        if (_chassis.busy())
            return;

        // 跳过运动期间的最后一帧，下一轮只接受停车后的新图像。
        maixcam::MaixVisionService::Observation stale;
        _vision.readNew(_visionToken, _visionCursor, stale);
        _movePending = false;
        _debug.movePending = false;
        _debug.hasObservation = false;
        _debug.found = false;
        _debug.forwardMm = 0.0F;
        _debug.rightMm = 0.0F;
        _lastObservationMs = 0;
        return;
    }

    maixcam::MaixVisionService::Observation frame;
    const bool hasDetection =
        _vision.readNew(_visionToken, _visionCursor, frame);
    const maixcam::Detection &detection = frame.detection;

    if (!hasDetection)
    {
        if (_allowRingFallback &&
            !_ringFallbackActive &&
            !_targetSeen &&
            now - _startedMs >= STORAGE_COLOR_FALLBACK_MS)
        {
            activateStorageRingFallback();
            return;
        }

        if (_lastObservationMs != 0 &&
            now - _lastObservationMs > RING_TARGET_STALE_MS)
        {
            _stableFrames = 0;
            _debug.hasObservation = false;
            _debug.found = false;
            _debug.stableFrames = 0;
            _debug.forwardMm = 0.0F;
            _debug.rightMm = 0.0F;
        }
        return;
    }

    if (detection.mode != _targetMode ||
        detection.targetId != _targetSelector)
    {
        _debug.lastMode = detection.mode;
        _debug.lastTargetId = detection.targetId;
        ++_debug.observations;
        ++_debug.ignoredObservations;
        return;
    }

    _lastObservationMs = now;
    _debug.hasObservation = true;
    _debug.found = detection.found;
    _debug.lastMode = detection.mode;
    _debug.lastTargetId = detection.targetId;
    ++_debug.observations;
    _debug.dx = detection.dx;
    _debug.dy = detection.dy;
    _debug.quality = detection.quality;
    if (!detection.found ||
        detection.quality < RING_MIN_QUALITY)
    {
        if (_allowRingFallback &&
            !_ringFallbackActive &&
            !_targetSeen &&
            now - _startedMs >= STORAGE_COLOR_FALLBACK_MS)
        {
            activateStorageRingFallback();
            return;
        }

        _stableFrames = 0;
        _debug.stableFrames = 0;
        return;
    }
    _targetSeen = true;

    const int16_t errorDx =
        detection.dx - RING_TARGET_DX_PX;
    const int16_t errorDy =
        detection.dy - RING_TARGET_DY_PX;
    const bool centered =
        abs(errorDx) <= RING_CENTER_TOLERANCE_PX &&
        abs(errorDy) <= RING_CENTER_TOLERANCE_PX;

    if (centered)
    {
        _debug.forwardMm = 0.0F;
        _debug.rightMm = 0.0F;
        if (_stableFrames < RING_REQUIRED_STABLE_FRAMES)
            ++_stableFrames;
        _debug.stableFrames = _stableFrames;

        if (_stableFrames >= RING_REQUIRED_STABLE_FRAMES)
        {
            if (!correctWorldPositionFromLandmark())
            {
                fail();
                return;
            }
            stopVision();
            _chassis.release();
            _result = AsyncResult::Succeeded;
        }
        return;
    }

    _stableFrames = 0;
    _debug.stableFrames = 0;
    const bool fineAlignment =
        abs(errorDx) <= RING_FINE_ALIGNMENT_ZONE_PX &&
        abs(errorDy) <= RING_FINE_ALIGNMENT_ZONE_PX;
    const float moveLimit =
        fineAlignment
            ? RING_FINE_MAX_MOVE_MM
            : RING_COARSE_MAX_MOVE_MM;

    const float forwardMm = clampMagnitude(
        errorDx * RING_FORWARD_MM_PER_DX_PX,
        moveLimit);
    const float rightMm = clampMagnitude(
        errorDy * RING_RIGHT_MM_PER_DY_PX,
        moveLimit);
    _debug.forwardMm = forwardMm;
    _debug.rightMm = rightMm;

    if (!_chassis.moveBodyRelative(
            forwardMm,
            rightMm,
            PRECISE_DRIVE_RPM,
            PRECISE_DRIVE_ACCEL_RPM_PER_S))
    {
        fail();
        return;
    }

    _movePending = true;
    _debug.movePending = true;
}

AsyncResult MaixProRingAlignment::result() const
{
    return _result;
}

void MaixProRingAlignment::cancel()
{
    if (_result == AsyncResult::Running)
        _chassis.stop();
    _chassis.release();

    stopVision();
    _movePending = false;
    _debug.movePending = false;
    _result = AsyncResult::Idle;
}

const MaixProRingAlignment::DebugState &
MaixProRingAlignment::debugState() const
{
    return _debug;
}

void MaixProRingAlignment::fail()
{
    _chassis.stop();
    _chassis.release();

    stopVision();
    _movePending = false;
    _debug.movePending = false;
    _result = AsyncResult::Failed;
}

void MaixProRingAlignment::requestVision()
{
    _visionToken = _vision.request(_targetMode, _targetSelector);
    _visionCursor = 0;
}

void MaixProRingAlignment::stopVision()
{
    _vision.release(_visionToken);
    _visionToken = 0;
}

void MaixProRingAlignment::activateStorageRingFallback()
{
    using namespace vision_config;

    _ringFallbackActive = true;
    _targetMode = maixcam::MODE_RING;
    _targetSelector = STORAGE_REFERENCE_RING_ID;
    _stableFrames = 0;
    _lastObservationMs = 0;
    _startedMs = millis();
    _debug.hasObservation = false;
    _debug.found = false;
    _debug.stableFrames = 0;
    _debug.targetMode = _targetMode;
    _debug.targetSelector = _targetSelector;
    requestVision();
}

bool MaixProRingAlignment::correctWorldPositionFromLandmark()
{
    using namespace vision_config;

    float targetX = 0.0F;
    float targetY = 0.0F;
    if (_station == Station::RoughProcessing)
    {
        targetX = ROUGH_ALIGNED_WORLD_X_MM;
        targetY = ROUGH_ALIGNED_WORLD_Y_MM;
    }
    else if (_station == Station::Storage)
    {
        targetX = STORAGE_ALIGNED_WORLD_X_MM;
        targetY = STORAGE_ALIGNED_WORLD_Y_MM;
    }
    else
    {
        return true;
    }

    const ChassisControl::Pose2D pose = _chassis.worldPose();
    const float errorX = targetX - pose.xMm;
    const float errorY = targetY - pose.yMm;
    const float correctionSquared =
        errorX * errorX + errorY * errorY;
    const float maximumSquared =
        RING_WORLD_CORRECTION_MAX_ERROR_MM *
        RING_WORLD_CORRECTION_MAX_ERROR_MM;
    if (correctionSquared > maximumSquared)
        return false;

    _debug.worldCorrectionMm = sqrtf(correctionSquared);
    if (!_chassis.correctWorldPosition(targetX, targetY))
        return false;

    _debug.worldPoseCorrected = true;
    return true;
}

float MaixProRingAlignment::clampMagnitude(
    float value,
    float limit)
{
    if (value > limit)
        return limit;
    if (value < -limit)
        return -limit;
    return value;
}
