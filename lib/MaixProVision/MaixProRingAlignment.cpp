#include "MaixProRingAlignment.h"

#include "chassis_config.h"
#include "vision_config.h"

MaixProRingAlignment::MaixProRingAlignment(
    maixcam::MaixCamV2 &camera,
    ChassisControl &chassis)
    : _camera(camera),
      _chassis(chassis)
{
}

bool MaixProRingAlignment::start(
    Station station,
    uint8_t)
{
    if (_result == AsyncResult::Running)
        return false;

    // 当前只为粗加工区启用圆环闭环，其他工位保持原有行为。
    if (station != Station::RoughProcessing)
    {
        _result = AsyncResult::Succeeded;
        return true;
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
    _result = AsyncResult::Running;
    _camera.setTarget(
        maixcam::MODE_RING,
        vision_config::ROUGH_RING_ID);
    return true;
}

void MaixProRingAlignment::update()
{
    using namespace chassis_config;
    using namespace vision_config;

    if (_result != AsyncResult::Running)
        return;

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
        maixcam::Detection staleDetection;
        _camera.takeDetection(staleDetection);
        _movePending = false;
        _lastObservationMs = 0;
        return;
    }

    maixcam::Detection detection;
    if (!_camera.takeDetection(detection))
    {
        if (_lastObservationMs != 0 &&
            now - _lastObservationMs > RING_TARGET_STALE_MS)
        {
            _stableFrames = 0;
        }
        return;
    }

    if (detection.mode != maixcam::MODE_RING ||
        detection.targetId != ROUGH_RING_ID)
    {
        return;
    }

    _lastObservationMs = now;
    if (!detection.found ||
        detection.quality < RING_MIN_QUALITY)
    {
        _stableFrames = 0;
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
        if (_stableFrames < RING_REQUIRED_STABLE_FRAMES)
            ++_stableFrames;

        if (_stableFrames >= RING_REQUIRED_STABLE_FRAMES)
        {
            _camera.reset();
            _result = AsyncResult::Succeeded;
        }
        return;
    }

    _stableFrames = 0;
    const bool fineAlignment =
        abs(errorDx) <= RING_FINE_ALIGNMENT_ZONE_PX &&
        abs(errorDy) <= RING_FINE_ALIGNMENT_ZONE_PX;
    const float moveLimit =
        fineAlignment
            ? RING_FINE_MAX_MOVE_MM
            : RING_COARSE_MAX_MOVE_MM;

    const float forwardMm = clampMagnitude(
        errorDy * RING_FORWARD_MM_PER_DY_PX,
        moveLimit);
    const float rightMm = clampMagnitude(
        errorDx * RING_RIGHT_MM_PER_DX_PX,
        moveLimit);

    if (!_chassis.moveRelative(
            forwardMm,
            rightMm,
            PRECISE_DRIVE_RPM,
            PRECISE_DRIVE_ACCEL_RPM_PER_S))
    {
        fail();
        return;
    }

    _movePending = true;
}

AsyncResult MaixProRingAlignment::result() const
{
    return _result;
}

void MaixProRingAlignment::cancel()
{
    if (_result == AsyncResult::Running && _chassis.busy())
        _chassis.stop();

    _camera.reset();
    _movePending = false;
    _result = AsyncResult::Idle;
}

void MaixProRingAlignment::fail()
{
    if (_chassis.busy())
        _chassis.stop();

    _camera.reset();
    _movePending = false;
    _result = AsyncResult::Failed;
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
