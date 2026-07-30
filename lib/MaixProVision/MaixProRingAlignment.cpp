#include "MaixProRingAlignment.h"

#include "chassis_config.h"
#include "vision_config.h"

MaixProRingAlignment::MaixProRingAlignment(
    maixcam::MaixCamV2 &camera,
    IGraspVisionProvider &graspVision,
    ChassisControl &chassis)
    : _camera(camera),
      _graspVision(graspVision),
      _chassis(chassis)
{
}

bool MaixProRingAlignment::start(
    const AlignmentRequest &request)
{
    if (_result == AsyncResult::Running)
        return false;

    // 原料区由机械臂逐物料视觉对准，不执行额外整车对准。
    if (request.station == Station::Material)
    {
        _useGraspVision = false;
        _targetMode = maixcam::MODE_IDLE;
        _targetSelector = 0;
        _debug = {};
        _result = AsyncResult::Succeeded;
        return true;
    }

    if (request.station == Station::RoughProcessing)
    {
        _useGraspVision = false;
        _targetMode = maixcam::MODE_RING;
        _targetSelector = vision_config::ROUGH_RING_ID;
    }
    else if (request.station == Station::Storage &&
             request.round == 0)
    {
        _useGraspVision = false;
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
        _useGraspVision = true;
        _targetMode = maixcam::MODE_GRAB;
        _targetSelector = request.referenceColor;
    }
    else
    {
        return false;
    }

    if (_chassis.busy() ||
        _chassis.state() == ChassisControl::State::Fault)
    {
        return false;
    }

    const uint32_t now = millis();
    _movePending = false;
    _stableFrames = 0;
    _startedMs = now;
    _lastObservationMs = 0;
    _debug = {};
    _debug.targetMode = _targetMode;
    _debug.targetSelector = _targetSelector;
    _result = AsyncResult::Running;
    if (_useGraspVision)
    {
        if (!_graspVision.startTracking(_targetSelector))
        {
            _result = AsyncResult::Idle;
            return false;
        }
    }
    else
    {
        _camera.setTarget(_targetMode, _targetSelector);
    }
    return true;
}

void MaixProRingAlignment::update()
{
    using namespace chassis_config;
    using namespace vision_config;

    if (_result != AsyncResult::Running)
        return;

    if (_useGraspVision)
    {
        _graspVision.update();
        if (_graspVision.faulted())
        {
            fail();
            return;
        }
    }
    else
    {
        _camera.poll();

        uint8_t command = 0;
        uint8_t commandResult = 0;
        uint8_t mode = 0;
        uint8_t selector = 0;
        if (_camera.takeAck(
                command,
                commandResult,
                mode,
                selector) &&
            command == maixcam::CMD_SET_TARGET &&
            commandResult != 0)
        {
            fail();
            return;
        }
    }

    const uint32_t now = millis();
    if (now - _startedMs >= RING_ALIGNMENT_TIMEOUT_MS)
    {
        fail();
        return;
    }

    if (_chassis.state() == ChassisControl::State::Fault)
    {
        fail();
        return;
    }

    if (_movePending)
    {
        if (_chassis.busy())
            return;

        // 清除运动期间缓存的最后一帧，下一轮只接受停车后的新图像。
        if (_useGraspVision)
        {
            GraspObservation staleObservation;
            _graspVision.takeObservation(staleObservation);
        }
        else
        {
            maixcam::Detection staleDetection;
            _camera.takeDetection(staleDetection);
        }
        _movePending = false;
        _debug.movePending = false;
        _debug.hasObservation = false;
        _debug.found = false;
        _debug.forwardMm = 0.0F;
        _debug.rightMm = 0.0F;
        _lastObservationMs = 0;
        return;
    }

    maixcam::Detection detection;
    bool hasDetection = false;
    if (_useGraspVision)
    {
        GraspObservation observation;
        if (_graspVision.takeObservation(observation))
        {
            detection.mode = maixcam::MODE_GRAB;
            detection.targetId = _targetSelector;
            detection.found = observation.found;
            detection.dx = observation.dx;
            detection.dy = observation.dy;
            detection.quality = observation.quality;
            hasDetection = true;
        }
    }
    else
    {
        hasDetection = _camera.takeDetection(detection);
    }

    if (!hasDetection)
    {
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
        return;
    }

    _lastObservationMs = now;
    _debug.hasObservation = true;
    _debug.found = detection.found;
    _debug.dx = detection.dx;
    _debug.dy = detection.dy;
    _debug.quality = detection.quality;
    if (!detection.found ||
        detection.quality < RING_MIN_QUALITY)
    {
        _stableFrames = 0;
        _debug.stableFrames = 0;
        return;
    }

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
            stopVision();
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
    if (_result == AsyncResult::Running && _chassis.busy())
        _chassis.stop();

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
    if (_chassis.busy())
        _chassis.stop();

    stopVision();
    _movePending = false;
    _debug.movePending = false;
    _result = AsyncResult::Failed;
}

void MaixProRingAlignment::stopVision()
{
    if (_useGraspVision)
        _graspVision.stop();
    else
        _camera.reset();
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
