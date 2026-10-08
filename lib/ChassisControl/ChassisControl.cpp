#include "ChassisControl.h"

#include "ChassisEmm42TtlFeedback.h"
#include "DebugLog.h"
#include "ChassisTracking.h"
#include "chassis_config.h"
#include <math.h>
#include <string.h>

using namespace chassis_config;

ChassisControl::ChassisControl(
    HardwareSerial *imuSerial,
    ChassisEmm42TtlFeedback *motorFeedback)
    : _imuSerial(imuSerial),
      _motorFeedback(motorFeedback)
{
}

void ChassisControl::begin()
{
    pinMode(ENABLE_PIN, OUTPUT);
    digitalWrite(ENABLE_PIN, LOW);

    _driver.begin();
    _driver.positions(_lastOdometrySteps);

    if (_imuSerial != nullptr)
        _imuSerial->begin(IMU_BAUD);
    if (_motorFeedback != nullptr)
        _motorFeedback->begin();

    _worldPose = {0.0F, 0.0F, 0.0F};
    _worldYawOffsetDeg = 0.0F;
    _worldYawReady = false;
    _worldYawOffsetReady = false;
    _state = State::Idle;
}

void ChassisControl::update()
{
    updateImu();
    if (_motorFeedback != nullptr)
    {
        _motorFeedback->update();
        if (_state != State::Fault &&
            !_motorFeedback->healthy())
        {
            setFault(_motorFeedback->faultMessage());
            return;
        }
    }
    updateOdometry();

    if (_driver.takeWatchdogTrip() && _state != State::Fault)
    {
        setFault("control loop stalled");
        return;
    }

    if (_state != State::Idle &&
        _state != State::Fault &&
        millis() - _motionStartMs > _motionTimeoutMs)
    {
        setFault(
            _state == State::Stopping
                ? "chassis stop timeout"
                : "chassis motion timeout");
        return;
    }

    switch (_state)
    {
    case State::Translating:
        updateTranslation();
        break;
    case State::Rotating:
        updateRotation();
        break;
    case State::Stopping:
        updateStopping();
        break;
    default:
        break;
    }
}

bool ChassisControl::moveBodyRelative(
    float forwardMm,
    float rightMm,
    float maxRpm,
    float accelerationRpmPerS)
{
    return moveBodyRelativeInternal(
        forwardMm,
        rightMm,
        maxRpm,
        accelerationRpmPerS,
        true);
}

bool ChassisControl::moveBodyRelativeTracking(
    float forwardMm,
    float rightMm,
    float maxRpm,
    float accelerationRpmPerS)
{
    return moveBodyRelativeInternal(
        forwardMm,
        rightMm,
        maxRpm,
        accelerationRpmPerS,
        false);
}

bool ChassisControl::moveBodyRelativeInternal(
    float forwardMm,
    float rightMm,
    float maxRpm,
    float accelerationRpmPerS,
    bool finalHeadingSettle)
{
    // 一次只允许执行一个运动命令。发生故障后需要先调用 clearFault()。
    if (busy() || _state == State::Fault)
        return false;
    if (!isfinite(forwardMm) ||
        !isfinite(rightMm) ||
        fabsf(forwardMm) > MAX_SINGLE_MOVE_COMPONENT_MM ||
        fabsf(rightMm) > MAX_SINGLE_MOVE_COMPONENT_MM)
        return false;

    // 车体方向按锁定航向换算，视觉和抓取给出的前后/左右不随残余偏航歪斜。
    const float holdWorldYawDeg = prepareTranslationHeading();
    float worldXMm = 0.0F;
    float worldYMm = 0.0F;
    bodyToWorld(
        forwardMm,
        rightMm,
        holdWorldYawDeg,
        worldXMm,
        worldYMm);
    return startTranslation(
        _worldPose.xMm + worldXMm,
        _worldPose.yMm + worldYMm,
        holdWorldYawDeg,
        maxRpm,
        accelerationRpmPerS,
        finalHeadingSettle);
}

float ChassisControl::prepareTranslationHeading()
{
    _translationHeadingEnabled =
        ENABLE_HEADING_HOLD && imuReady();
    if (!_translationHeadingEnabled)
        return _worldPose.yawDeg;

    const float referenceErrorDeg =
        wrap180(_headingReferenceDeg - _yawDeg);
    if (!_headingReferenceValid ||
        fabsf(referenceErrorDeg) > HEADING_REFERENCE_MAX_ERROR_DEG)
    {
        if (_headingReferenceValid)
        {
            LOGF_WARNING("Chassis", "HEADING_REFERENCE_RESET ref=%.1f° yaw=%.1f°",
                         _headingReferenceDeg, _yawDeg);
        }
        _headingReferenceDeg = _yawDeg;
        _headingReferenceValid = true;
    }

    _holdYawDeg = _headingReferenceDeg;
    return wrap180(_holdYawDeg - _worldYawOffsetDeg);
}

bool ChassisControl::startTranslation(
    float targetWorldXMm,
    float targetWorldYMm,
    float holdWorldYawDeg,
    float maxRpm,
    float accelerationRpmPerS,
    bool finalHeadingSettle,
    float turnTargetYawDeg)
{
    if (!isfinite(targetWorldXMm) ||
        !isfinite(targetWorldYMm) ||
        !isfinite(maxRpm) ||
        !isfinite(accelerationRpmPerS) ||
        maxRpm <= 0.0f ||
        accelerationRpmPerS <= 0.0f)
        return false;

    const float deltaXMm = targetWorldXMm - _worldPose.xMm;
    const float deltaYMm = targetWorldYMm - _worldPose.yMm;
    const float lengthMm =
        sqrtf(deltaXMm * deltaXMm + deltaYMm * deltaYMm);
    if (lengthMm * STEPS_PER_MM < 0.5F)
        return true;

    _trackHoldWorldYawDeg = holdWorldYawDeg;
    _trackMaxRpm = maxRpm;
    _trackAccelerationRpmPerS = accelerationRpmPerS;
    _trackNextPending = false;
    _trackBlendActive = false;
    _trackAlongSpeedMm = 0.0F;
    const float wheelSteps = setupTrackSegment(
        _worldPose.xMm,
        _worldPose.yMm,
        targetWorldXMm,
        targetWorldYMm,
        turnTargetYawDeg);
    _trackLastControlUs = micros();

    _translationFinalHeadingSettleEnabled = finalHeadingSettle;
    _translationPhase = TranslationPhase::Driving;
    resetYawStability();
    _motionStartMs = millis();
    _motionTimeoutMs = segmentTimeoutMs(wheelSteps);
    _state = State::Translating;

    LOGF_INFO("Chassis", "MOVE_START from=(%.1f,%.1f) to=(%.1f,%.1f) hold=%.1f° turn=%.1f° window=%.0fmm speed=%.0fRPM",
              _trackStartXMm, _trackStartYMm,
              _trackTargetXMm, _trackTargetYMm,
              holdWorldYawDeg,
              _trackYawReached ? 0.0F : _trackYawTotalDeg,
              _trackTurnWindowMm, maxRpm);
    return true;
}

float ChassisControl::configureTrackLimits()
{
    /*
     * maxRpm限制的是轮速。路径方向在车体系中的分量决定每走1mm路径
     * 最快的轮子要走多少mm（斜移时为|f|+|r|），据此换算路径限速，
     * 使最大轮速和轮加速度仍等于路线档位参数。位姿平移的行进轴是
     * 车体轴；窗口内转向时速度已降低，超限由逆解等比缩放兜底。
     */
    float directionForward = 0.0F;
    float directionRight = 0.0F;
    worldToBody(
        _trackDirectionX,
        _trackDirectionY,
        _trackHoldWorldYawDeg,
        directionForward,
        directionRight);
    const float wheelMmPerPathMm =
        fabsf(directionForward) *
            distanceScale(
                directionForward,
                FORWARD_DISTANCE_SCALE,
                BACKWARD_DISTANCE_SCALE) +
        fabsf(directionRight) *
            distanceScale(
                directionRight,
                RIGHT_DISTANCE_SCALE,
                LEFT_DISTANCE_SCALE);
    const float wheelStepsPerPathMm =
        STEPS_PER_MM * wheelMmPerPathMm;
    const float maximumWheelSpeed = rpmToStepsPerSecond(_trackMaxRpm);
    const float wheelAcceleration =
        rpmToStepsPerSecond(_trackAccelerationRpmPerS);

    _trackMaximumSpeedMm = maximumWheelSpeed / wheelStepsPerPathMm;
    _trackAccelerationMm = wheelAcceleration / wheelStepsPerPathMm;
    _trackJerkMm = _trackAccelerationMm / DRIVE_ACCEL_RAMP_S;
    _trackMaximumWheelSpeed = maximumWheelSpeed;
    updateTrackWheelAcceleration();
    return wheelStepsPerPathMm;
}

float ChassisControl::setupTrackSegment(
    float startWorldXMm,
    float startWorldYMm,
    float targetWorldXMm,
    float targetWorldYMm,
    float turnTargetYawDeg)
{
    const float deltaXMm = targetWorldXMm - startWorldXMm;
    const float deltaYMm = targetWorldYMm - startWorldYMm;
    const float lengthMm =
        sqrtf(deltaXMm * deltaXMm + deltaYMm * deltaYMm);

    float directionX = deltaXMm / lengthMm;
    float directionY = deltaYMm / lengthMm;
    const bool poseMove = isfinite(turnTargetYawDeg);
    if (poseMove)
    {
        // 行进轴取起步航向下最接近目标方向的车体轴。
        float forwardX = 0.0F;
        float forwardY = 0.0F;
        float rightX = 0.0F;
        float rightY = 0.0F;
        bodyToWorld(1.0F, 0.0F, _trackHoldWorldYawDeg, forwardX, forwardY);
        bodyToWorld(0.0F, 1.0F, _trackHoldWorldYawDeg, rightX, rightY);
        const float forwardDot = forwardX * deltaXMm + forwardY * deltaYMm;
        const float rightDot = rightX * deltaXMm + rightY * deltaYMm;
        const bool useForward = fabsf(forwardDot) >= fabsf(rightDot);
        const float sign = copysignf(
            1.0F,
            useForward ? forwardDot : rightDot);
        directionX = sign * (useForward ? forwardX : rightX);
        directionY = sign * (useForward ? forwardY : rightY);
    }
    const float alongLengthMm =
        deltaXMm * directionX + deltaYMm * directionY;

    _trackStartXMm = startWorldXMm;
    _trackStartYMm = startWorldYMm;
    _trackTargetXMm = targetWorldXMm;
    _trackTargetYMm = targetWorldYMm;
    _trackDirectionX = directionX;
    _trackDirectionY = directionY;
    _trackTargetCrossMm = directionX * deltaYMm - directionY * deltaXMm;
    _trackTargetYawDeg = poseMove ? turnTargetYawDeg : _holdYawDeg;
    _trackYawTotalDeg =
        fabsf(wrap180(_trackTargetYawDeg - _holdYawDeg));
    _trackTurnStarted = false;
    _trackYawReached = _trackYawTotalDeg <= ROTATE_TOLERANCE_DEG;
    if (_trackYawReached)
        _holdYawDeg = _trackTargetYawDeg;

    // 进度沿行进轴计，与pathError的along一致；横偏在窗口内另行并入。
    _trackLengthMm = alongLengthMm;
    _trackProfile = {};
    const float wheelStepsPerPathMm = configureTrackLimits();
    _trackTurnWindowMm =
        poseMove
            ? min(
                  alongLengthMm,
                  max(
                      POSE_MOVE_TURN_MIN_WINDOW_MM,
                      min(
                          POSE_MOVE_TURN_WINDOW_MM,
                          rotationWindowMm(
                              _trackYawTotalDeg,
                              _trackMaximumSpeedMm))))
            : 0.0F;
    return wheelStepsPerPathMm * lengthMm;
}

uint32_t ChassisControl::segmentTimeoutMs(float wheelSteps) const
{
    uint32_t timeoutMs = estimateTranslationTimeoutMs(
        lroundf(wheelSteps),
        _trackMaximumWheelSpeed,
        rpmToStepsPerSecond(_trackAccelerationRpmPerS));
    if (!_trackYawReached)
        timeoutMs += estimateRotationTimeoutMs(_trackYawTotalDeg);
    return min(timeoutMs, MOTION_TIMEOUT_MAX_MS);
}

void ChassisControl::updateTrackWheelAcceleration()
{
    float acceleration =
        rpmToStepsPerSecond(_trackAccelerationRpmPerS) *
        TRACK_WHEEL_ACCEL_MARGIN;
    // 两段加减速同时作用，拐角处换向的一对轮子需要两倍斜坡。
    if (_trackBlendActive)
        acceleration *= 2.0F;
    // 转向中车体系行进方向在变（斜向轮速为|f|+|r|≤√2倍），且叠加旋转斜坡。
    if (_trackTurnStarted)
    {
        acceleration =
            acceleration * sqrtf(2.0F) +
            rpmToStepsPerSecond(ROTATE_ACCEL_RPM_PER_S);
    }
    _trackWheelAcceleration = acceleration;
}

void ChassisControl::startNextTrackSegment()
{
    _trackNextPending = false;
    _trackBlendActive = true;
    _trackBlendDirectionX = _trackDirectionX;
    _trackBlendDirectionY = _trackDirectionY;
    _trackBlendLengthMm = _trackLengthMm;
    _trackBlendMaximumSpeedMm = _trackMaximumSpeedMm;
    _trackBlendAccelerationMm = _trackAccelerationMm;
    _trackBlendJerkMm = _trackJerkMm;
    _trackBlendProfile = _trackProfile;

    const float wheelSteps = setupTrackSegment(
        _trackTargetXMm,
        _trackTargetYMm,
        _trackNextTargetXMm,
        _trackNextTargetYMm,
        _trackNextYawDeg);

    const uint32_t elapsedMs = millis() - _motionStartMs;
    _motionTimeoutMs = min(
        max(_motionTimeoutMs, elapsedMs + segmentTimeoutMs(wheelSteps)),
        MOTION_TIMEOUT_MAX_MS);

    LOGF_INFO("Chassis", "MOVE_BLEND_START pose=(%.1f,%.1f) via=(%.1f,%.1f) to=(%.1f,%.1f) speed=%.0fmm/s turn=%.1f° window=%.0fmm",
              _worldPose.xMm, _worldPose.yMm,
              _trackStartXMm, _trackStartYMm,
              _trackTargetXMm, _trackTargetYMm,
              _trackBlendProfile.velocity,
              _trackYawReached ? 0.0F : _trackYawTotalDeg,
              _trackTurnWindowMm);
}

void ChassisControl::startTrackTurn(float alongMm)
{
    _trackTurnStarted = true;
    _holdYawDeg = _trackTargetYawDeg;
    // 旋转从零速斜坡起步，与rotateTo一致。
    resetYawStability();
    updateTrackWheelAcceleration();
    LOGF_INFO("Chassis", "MOVE_TURN_START along=%.1fmm speed=%.0fmm/s yaw=%.1f° target=%.1f°",
              alongMm, _trackAlongSpeedMm, _yawDeg, _trackTargetYawDeg);
}

bool ChassisControl::moveRelative(
    float forwardMm,
    float rightMm,
    float maxRpm,
    float accelerationRpmPerS)
{
    return moveBodyRelative(
        forwardMm,
        rightMm,
        maxRpm,
        accelerationRpmPerS);
}

bool ChassisControl::moveWorldRelative(
    float worldXMm,
    float worldYMm,
    float maxRpm,
    float accelerationRpmPerS)
{
    if (!_worldYawReady)
    {
        setFault("world move rejected: pose yaw not ready");
        return false;
    }
    if (busy() || _state == State::Fault)
        return false;
    if (!isfinite(worldXMm) ||
        !isfinite(worldYMm) ||
        fabsf(worldXMm) > MAX_SINGLE_MOVE_COMPONENT_MM ||
        fabsf(worldYMm) > MAX_SINGLE_MOVE_COMPONENT_MM)
        return false;

    const float holdWorldYawDeg = prepareTranslationHeading();
    return startTranslation(
        _worldPose.xMm + worldXMm,
        _worldPose.yMm + worldYMm,
        holdWorldYawDeg,
        maxRpm,
        accelerationRpmPerS,
        true);
}

bool ChassisControl::moveWorldTo(
    float targetWorldXMm,
    float targetWorldYMm,
    float maxRpm,
    float accelerationRpmPerS)
{
    return moveWorldRelative(
        targetWorldXMm - _worldPose.xMm,
        targetWorldYMm - _worldPose.yMm,
        maxRpm,
        accelerationRpmPerS);
}

bool ChassisControl::moveWorldThrough(
    float viaWorldXMm,
    float viaWorldYMm,
    float targetWorldXMm,
    float targetWorldYMm,
    float maxRpm,
    float accelerationRpmPerS)
{
    const float nextXMm = targetWorldXMm - viaWorldXMm;
    const float nextYMm = targetWorldYMm - viaWorldYMm;
    if (!isfinite(nextXMm) ||
        !isfinite(nextYMm) ||
        fabsf(nextXMm) > MAX_SINGLE_MOVE_COMPONENT_MM ||
        fabsf(nextYMm) > MAX_SINGLE_MOVE_COMPONENT_MM)
        return false;

    if (!moveWorldTo(
            viaWorldXMm,
            viaWorldYMm,
            maxRpm,
            accelerationRpmPerS))
        return false;

    // 已在途经点上时第一段不会启动，直接走第二段。
    if (_state != State::Translating)
    {
        return moveWorldTo(
            targetWorldXMm,
            targetWorldYMm,
            maxRpm,
            accelerationRpmPerS);
    }

    const float nextLengthMm =
        sqrtf(nextXMm * nextXMm + nextYMm * nextYMm);
    if (nextLengthMm * STEPS_PER_MM < 0.5F)
        return true;

    _trackNextPending = true;
    _trackNextTargetXMm = targetWorldXMm;
    _trackNextTargetYMm = targetWorldYMm;
    _trackNextYawDeg = NAN;
    _trackBlendMm = min(
        TRACK_BLEND_DISTANCE_MM,
        0.5F * min(_trackLengthMm, nextLengthMm));
    return true;
}

bool ChassisControl::moveWorldToPose(
    float targetWorldXMm,
    float targetWorldYMm,
    float worldYawDeg,
    float maxRpm,
    float accelerationRpmPerS)
{
    if (!_worldYawOffsetReady || !imuReady())
    {
        setFault("pose move rejected: IMU yaw not ready");
        return false;
    }
    if (busy() || _state == State::Fault)
        return false;

    const float deltaXMm = targetWorldXMm - _worldPose.xMm;
    const float deltaYMm = targetWorldYMm - _worldPose.yMm;
    if (!isfinite(deltaXMm) ||
        !isfinite(deltaYMm) ||
        !isfinite(worldYawDeg) ||
        fabsf(deltaXMm) > MAX_SINGLE_MOVE_COMPONENT_MM ||
        fabsf(deltaYMm) > MAX_SINGLE_MOVE_COMPONENT_MM)
        return false;

    const float targetYawDeg =
        wrap180(worldYawDeg + _worldYawOffsetDeg);
    if (sqrtf(deltaXMm * deltaXMm + deltaYMm * deltaYMm) *
            STEPS_PER_MM <
        0.5F)
        return rotateTo(targetYawDeg);

    // 转向前的直线保持起步航向，与普通平移取同一航向参考。
    const float holdWorldYawDeg = prepareTranslationHeading();
    if (!_translationHeadingEnabled)
        _holdYawDeg = _yawDeg;
    _translationHeadingEnabled = true;
    // 与rotateTo一致：转向目标成为后续平移的航向参考。
    _headingReferenceDeg = targetYawDeg;
    _headingReferenceValid = true;

    return startTranslation(
        targetWorldXMm,
        targetWorldYMm,
        holdWorldYawDeg,
        maxRpm,
        accelerationRpmPerS,
        true,
        targetYawDeg);
}

bool ChassisControl::rotateTo(float absoluteYawDeg)
{
    if (busy() || _state == State::Fault)
        return false;
    if (!isfinite(absoluteYawDeg))
        return false;
    if (!imuReady())
    {
        setFault("rotate rejected: IMU not ready");
        return false;
    }

    _rotateTargetDeg = wrap180(absoluteYawDeg);
    _rotateStartSteps = rotationSteps();
    _rotateStartYawDeg = _yawDeg;
    _headingReferenceDeg = _rotateTargetDeg;
    _headingReferenceValid = true;
    resetYawStability();
    _motionStartMs = millis();
    _motionTimeoutMs = estimateRotationTimeoutMs(
        fabsf(wrap180(_rotateTargetDeg - _yawDeg)));
    _state = State::Rotating;
    
    LOGF_INFO("Chassis", "ROTATE_START current=%.1f° target=%.1f° delta=%.1f°",
              _yawDeg, _rotateTargetDeg, wrap180(_rotateTargetDeg - _yawDeg));
    
    return true;
}

bool ChassisControl::rotateWorldTo(float worldYawDeg)
{
    if (!_worldYawOffsetReady)
    {
        setFault("world rotate rejected: IMU yaw not ready");
        return false;
    }

    return rotateTo(
        wrap180(worldYawDeg + _worldYawOffsetDeg));
}

void ChassisControl::stop()
{
    if (_state == State::Fault || _state == State::Idle)
    {
        _driver.halt();
        return;
    }

    /*
     * 不在stop()里阻塞。脉冲中断沿减速曲线停车，update()只检查
     * 是否停稳，IMU、视觉和机构状态机不会被底盘急停卡住。
     */
    _driver.stop();

    _motionStartMs = millis();
    _motionTimeoutMs = STOP_TIMEOUT_MS;
    _state = State::Stopping;
}

void ChassisControl::clearFault()
{
    _driver.halt();
    _driver.takeWatchdogTrip();
    if (_motorFeedback != nullptr)
        _motorFeedback->clearFault();
    _fault[0] = '\0';
    _state = State::Idle;
}

ChassisControl::State ChassisControl::state() const
{
    return _state;
}

bool ChassisControl::busy() const
{
    return _state == State::Translating ||
           _state == State::Rotating ||
           _state == State::Stopping;
}

bool ChassisControl::imuReady() const
{
    return _imuReady &&
           millis() - _lastImuMs <= IMU_STALE_TIMEOUT_MS;
}

float ChassisControl::yawDeg() const
{
    return _yawDeg;
}

ChassisControl::Pose2D ChassisControl::worldPose() const
{
    return _worldPose;
}

bool ChassisControl::resetWorldPose(
    float worldXmm,
    float worldYmm,
    float worldYawDeg)
{
    if (busy())
        return false;
    if (!isfinite(worldXmm) ||
        !isfinite(worldYmm) ||
        !isfinite(worldYawDeg))
        return false;

    _worldPose.xMm = worldXmm;
    _worldPose.yMm = worldYmm;
    _worldPose.yawDeg = wrap180(worldYawDeg);

    if (imuReady())
    {
        _worldYawOffsetDeg =
            wrap180(_yawDeg - _worldPose.yawDeg);
        _worldYawReady = true;
        _worldYawOffsetReady = true;
        _headingReferenceDeg = _yawDeg;
        _headingReferenceValid = true;
    }
    else
    {
        /*
         * 调用方已经明确给出了世界航向，可以立即执行首段平移。
         * IMU零偏仍等待第一帧再建立，旋转动作不会提前放行。
         */
        _worldYawReady = true;
        _worldYawOffsetReady = false;
    }

    _driver.positions(_lastOdometrySteps);
    return true;
}

bool ChassisControl::correctWorldPosition(
    float worldXmm,
    float worldYmm)
{
    if (busy() ||
        !isfinite(worldXmm) ||
        !isfinite(worldYmm))
    {
        return false;
    }

    _worldPose.xMm = worldXmm;
    _worldPose.yMm = worldYmm;
    _driver.positions(_lastOdometrySteps);
    return true;
}

const char *ChassisControl::faultMessage() const
{
    return _fault;
}

const char *chassisOwnerName(ChassisOwner owner)
{
    switch (owner)
    {
    case ChassisOwner::None:
        return "none";
    case ChassisOwner::Route:
        return "route";
    case ChassisOwner::Alignment:
        return "alignment";
    case ChassisOwner::Grasp:
        return "grasp";
    }
    return "unknown";
}

bool ChassisControl::acquire(ChassisOwner owner)
{
    if (owner == ChassisOwner::None)
        return false;
    if (_owner == owner)
        return true;
    if (_owner != ChassisOwner::None)
    {
        LOGF_ERROR("Chassis", "OWNER_DENIED request=%s owner=%s",
                   chassisOwnerName(owner), chassisOwnerName(_owner));
        return false;
    }

    _owner = owner;
    LOGF_DEBUG("Chassis", "OWNER_ACQUIRE %s", chassisOwnerName(owner));
    return true;
}

void ChassisControl::release(ChassisOwner owner)
{
    if (owner == ChassisOwner::None || _owner != owner)
        return;

    _owner = ChassisOwner::None;
    LOGF_DEBUG("Chassis", "OWNER_RELEASE %s", chassisOwnerName(owner));
}

ChassisOwner ChassisControl::owner() const
{
    return _owner;
}

bool ChassisControl::updateImu()
{
    if (_imuSerial == nullptr)
        return false;

    bool updated = false;
    while (_imuSerial->available())
    {
        const uint8_t data = _imuSerial->read();
        if (_imuIndex == 0 && data != 0x55)
            continue;

        _imuFrame[_imuIndex++] = data;
        if (_imuIndex < sizeof(_imuFrame))
            continue;

        _imuIndex = 0;
        uint8_t checksum = 0;
        for (uint8_t i = 0; i < 10; ++i)
            checksum += _imuFrame[i];

        // WIT/JY901 0x53 为角度帧，Yaw 位于字节 6、7。
        if (_imuFrame[1] == 0x53 && checksum == _imuFrame[10])
        {
            const int16_t rawYaw =
                static_cast<int16_t>(
                    static_cast<uint16_t>(_imuFrame[6]) |
                    (static_cast<uint16_t>(_imuFrame[7]) << 8));
            const float yawDeg = rawYaw / 32768.0f * 180.0f;
            _imuFrameRotationSteps = rotationSteps();
            _yawDeg = yawDeg;
            _lastImuMs = millis();
            _imuReady = true;
            ++_imuSequence;
            if (!_headingReferenceValid)
            {
                _headingReferenceDeg = _yawDeg;
                _headingReferenceValid = true;
            }
            if (!_worldYawOffsetReady)
            {
                _worldYawOffsetDeg =
                    wrap180(_yawDeg - _worldPose.yawDeg);
                _worldYawOffsetReady = true;
            }
            _worldYawReady = true;
            _worldPose.yawDeg =
                wrap180(_yawDeg - _worldYawOffsetDeg);
            updated = true;
        }
    }
    return updated;
}

void ChassisControl::updateOdometry()
{
    long current[4] = {};
    _driver.positions(current);

    float wheelMm[4] = {};
    bool moved = false;
    for (uint8_t i = 0; i < 4; ++i)
    {
        const long deltaSteps = current[i] - _lastOdometrySteps[i];
        _lastOdometrySteps[i] = current[i];
        wheelMm[i] =
            static_cast<float>(deltaSteps) /
            (STEPS_PER_MM * MOTOR_SIGN[i]);
        moved = moved || deltaSteps != 0;
    }

    if (!moved)
        return;

    /*
     * 麦轮正解算。四轮共同的旋转分量在这两个平移组合中抵消，
     * 所以旋转由IMU记录，x/y只累计实际产生的平移脉冲。
     */
    const float calibratedForwardMm =
        (wheelMm[0] + wheelMm[1] +
         wheelMm[2] + wheelMm[3]) *
        0.25F;
    const float calibratedRightMm =
        (wheelMm[0] - wheelMm[1] -
         wheelMm[2] + wheelMm[3]) *
        0.25F;
    const float forwardMm =
        calibratedForwardMm /
        distanceScale(
            calibratedForwardMm,
            FORWARD_DISTANCE_SCALE,
            BACKWARD_DISTANCE_SCALE);
    const float rightMm =
        calibratedRightMm /
        distanceScale(
            calibratedRightMm,
            RIGHT_DISTANCE_SCALE,
            LEFT_DISTANCE_SCALE);

    float worldXMm = 0.0F;
    float worldYMm = 0.0F;
    bodyToWorld(
        forwardMm,
        rightMm,
        predictedWorldYawDeg(),
        worldXMm,
        worldYMm);
    _worldPose.xMm += worldXMm;
    _worldPose.yMm += worldYMm;
}

float ChassisControl::predictedYawDeg() const
{
    // 帧间用旋转脉冲外推：脉冲无延迟，不受IMU帧率和角速度估计噪声影响。
    if (!_imuReady)
        return _yawDeg;
    return wrap180(
        _yawDeg +
        static_cast<float>(rotationSteps() - _imuFrameRotationSteps) /
            ROTATION_STEPS_PER_DEG);
}

float ChassisControl::predictedWorldYawDeg() const
{
    if (!_imuReady)
        return _worldPose.yawDeg;
    return wrap180(predictedYawDeg() - _worldYawOffsetDeg);
}

void ChassisControl::updateTranslation()
{
    if (_translationHeadingEnabled && !imuReady())
    {
        setFault("IMU stale while translating");
        return;
    }

    switch (_translationPhase)
    {
    case TranslationPhase::HeadingSettling:
        if (updateYawSettle(
                _holdYawDeg,
                TRANSLATION_FINAL_HEADING_TOLERANCE_DEG))
        {
            _state = State::Idle;
            LOGF_INFO("Chassis", "MOVE_COMPLETE pose=(%.1f, %.1f, %.1f°) duration=%lums",
                      _worldPose.xMm, _worldPose.yMm, _worldPose.yawDeg,
                      millis() - _motionStartMs);
        }
        return;

    case TranslationPhase::Braking:
        // 零速也要每周期刷新，否则速度看门狗会判定主循环卡死。
        driveBodyVelocity(0.0F, 0.0F, 0.0F);
        if (_driver.allStopped())
        {
            _driver.halt();
            finishTranslation();
        }
        return;

    case TranslationPhase::Driving:
        break;

    case TranslationPhase::PoseHeadingSettling:
        if (!updatePoseHeadingSettle(_trackTargetYawDeg))
            return;
        break;
    }

    using namespace chassis_tracking;

    const uint32_t nowUs = micros();
    const float elapsedSeconds = min(
        static_cast<float>(nowUs - _trackLastControlUs) * 1.0e-6F,
        0.05F);
    _trackLastControlUs = nowUs;

    if (_trackNextPending &&
        _trackLengthMm - _trackProfile.position <= _trackBlendMm)
    {
        startNextTrackSegment();
    }

    const Vec2 direction = {_trackDirectionX, _trackDirectionY};
    const PathError error = pathError(
        {_trackStartXMm, _trackStartYMm},
        direction,
        {_trackTargetXMm, _trackTargetYMm},
        {_worldPose.xMm, _worldPose.yMm});

    const float targetYawErrorDeg =
        wrap180(_trackTargetYawDeg - _yawDeg);
    if (!_trackYawReached)
    {
        // 窗口前按到终点的正常曲线行驶，进入窗口才起转。
        if (!_trackTurnStarted &&
            error.along <= _trackTurnWindowMm)
        {
            startTrackTurn(error.along);
        }
        if (_trackTurnStarted &&
            !_trackYawReached &&
            fabsf(targetYawErrorDeg) <= ROTATE_TOLERANCE_DEG)
        {
            _translationPhase = TranslationPhase::PoseHeadingSettling;
            _trackPoseStableSamples = 0;
            _trackPoseStableSinceMs = 0;
            resetYawStability();
            return;
        }
    }

    // 横偏参考在窗口内从0线性过渡到目标横偏，窗口外保持纯轴向运动。
    float crossReferenceMm = 0.0F;
    float crossFeedforwardMm = 0.0F;
    if (_trackTurnWindowMm > 0.0F)
    {
        const float windowRatio = constrain(
            error.along / _trackTurnWindowMm,
            0.0F,
            1.0F);
        crossReferenceMm = _trackTargetCrossMm * (1.0F - windowRatio);
        if (windowRatio > 0.0F && windowRatio < 1.0F)
        {
            crossFeedforwardMm =
                _trackTargetCrossMm / _trackTurnWindowMm *
                _trackAlongSpeedMm;
        }
    }
    /*
     * 上一段参考在途经点前还剩r=-u1·(L1-s1)，速度u1·v1；投影到当前段
     * 的沿程和横偏上叠加，两段合成的参考轨迹连续、圆滑内切拐角。
     */
    float blendAlongMm = 0.0F;
    float blendAlongSpeedMm = 0.0F;
    if (_trackBlendActive)
    {
        jerkStep(
            _trackBlendProfile,
            _trackBlendLengthMm,
            _trackBlendMaximumSpeedMm,
            _trackBlendAccelerationMm,
            _trackBlendJerkMm,
            elapsedSeconds);
        const float alongDot =
            _trackBlendDirectionX * direction.x +
            _trackBlendDirectionY * direction.y;
        const float crossDot =
            direction.x * _trackBlendDirectionY -
            direction.y * _trackBlendDirectionX;
        const float blendRemainingMm =
            _trackBlendLengthMm - _trackBlendProfile.position;
        blendAlongMm = -blendRemainingMm * alongDot;
        blendAlongSpeedMm = _trackBlendProfile.velocity * alongDot;
        crossReferenceMm -= blendRemainingMm * crossDot;
        crossFeedforwardMm += _trackBlendProfile.velocity * crossDot;
        if (jerkFinished(_trackBlendProfile, _trackBlendLengthMm))
        {
            _trackBlendActive = false;
            updateTrackWheelAcceleration();
        }
    }
    const float crossErrorMm = error.cross - crossReferenceMm;

    if (!_trackNextPending &&
        !_trackBlendActive &&
        _trackYawReached &&
        fabsf(error.along) <= TRACK_ALONG_TOLERANCE_MM &&
        fabsf(crossErrorMm) <= TRACK_CROSS_TOLERANCE_MM)
    {
        _trackAlongSpeedMm = 0.0F;
        _translationPhase = TranslationPhase::Braking;
        driveBodyVelocity(0.0F, 0.0F, 0.0F);
        return;
    }

    // 转向中平移只允许跟到实测转向进度，终点前一定先转到位。
    const float remainingMm =
        !_trackTurnStarted || _trackYawReached
            ? error.along
            : yawGatedRemaining(
                  error.along,
                  _trackTurnWindowMm,
                  targetYawErrorDeg,
                  _trackYawTotalDeg);
    /*
     * 发生器沿路径生成加加速度受限的参考，终点为有效剩余距离对应的
     * 进度。里程由脉冲计数得到，参考与实际之差很小，比例修正只消除
     * 累积误差；参考停止后由同一修正加最低速度收尾到容差内。
     */
    const float progressMm = _trackLengthMm - error.along;
    // 普通平移remaining==along，goal恰为全长，发生器可精确吸附到终点。
    const float goalMm = _trackLengthMm - (error.along - remainingMm);
    jerkStep(
        _trackProfile,
        goalMm,
        _trackMaximumSpeedMm,
        _trackAccelerationMm,
        _trackJerkMm,
        elapsedSeconds);
    const float profileErrorMm =
        _trackProfile.position + blendAlongMm - progressMm;
    float alongSpeedMm =
        _trackProfile.velocity + blendAlongSpeedMm +
        TRACK_POSITION_KP_PER_S * profileErrorMm;
    if (jerkFinished(_trackProfile, goalMm) &&
        fabsf(profileErrorMm) > TRACK_ALONG_TOLERANCE_MM &&
        fabsf(alongSpeedMm) < TRACK_MIN_SPEED_MM_PER_S)
    {
        alongSpeedMm = copysignf(TRACK_MIN_SPEED_MM_PER_S, profileErrorMm);
    }
    _trackAlongSpeedMm = clampMagnitude(alongSpeedMm, _trackMaximumSpeedMm);
    const float crossSpeedMm =
        crossCorrection(
            crossErrorMm,
            TRACK_CROSS_KP_PER_S,
            _trackMaximumSpeedMm * TRACK_CROSS_MAX_RATIO,
            TRACK_MIN_SPEED_MM_PER_S,
            TRACK_CROSS_TOLERANCE_MM) +
        crossFeedforwardMm;

    // 沿程速度走u，横偏修正走左法向n=(-u.y, u.x)。
    const float worldVelocityX =
        _trackAlongSpeedMm * direction.x -
        crossSpeedMm * direction.y;
    const float worldVelocityY =
        _trackAlongSpeedMm * direction.y +
        crossSpeedMm * direction.x;
    float forwardMmPerS = 0.0F;
    float rightMmPerS = 0.0F;
    worldToBody(
        worldVelocityX,
        worldVelocityY,
        predictedWorldYawDeg(),
        forwardMmPerS,
        rightMmPerS);

    float rotationStepsPerS = 0.0F;
    if (_trackTurnStarted && !_trackYawReached)
    {
        rotationStepsPerS = rotationProfileSpeed(
            wrap180(_trackTargetYawDeg - predictedYawDeg()));
    }
    else if (_translationHeadingEnabled)
    {
        const float headingErrorDeg = wrap180(_holdYawDeg - _yawDeg);
        const float limit =
            _trackMaximumWheelSpeed * HEADING_MAX_CORRECTION_RATIO;
        rotationStepsPerS = clampMagnitude(
            headingErrorDeg * HEADING_KP_STEPS_PER_S_PER_DEG,
            limit);
        if (fabsf(headingErrorDeg) >
                TRANSLATION_FINAL_HEADING_TOLERANCE_DEG &&
            fabsf(rotationStepsPerS) < ROTATE_MIN_STEPS_PER_S)
        {
            rotationStepsPerS = copysignf(
                min(ROTATE_MIN_STEPS_PER_S, limit),
                headingErrorDeg);
        }
    }

    driveBodyVelocity(forwardMmPerS, rightMmPerS, rotationStepsPerS);
}

void ChassisControl::driveBodyVelocity(
    float forwardMmPerS,
    float rightMmPerS,
    float rotationStepsPerS)
{
    /*
     * 四麦轮逆运动学，电机顺序：左前、右前、左后、右后。
     *
     *   左前 = 前进 + 右移    右前 = 前进 - 右移
     *   左后 = 前进 - 右移    右后 = 前进 + 右移
     *
     * 平移分量经MOTOR_SIGN适配安装方向；旋转分量按当前接线四轮同号。
     */
    const float calibratedForward =
        forwardMmPerS * distanceScale(
                            forwardMmPerS,
                            FORWARD_DISTANCE_SCALE,
                            BACKWARD_DISTANCE_SCALE);
    const float calibratedRight =
        rightMmPerS * distanceScale(
                          rightMmPerS,
                          RIGHT_DISTANCE_SCALE,
                          LEFT_DISTANCE_SCALE);
    const float wheelMmPerS[4] = {
        calibratedForward + calibratedRight,
        calibratedForward - calibratedRight,
        calibratedForward - calibratedRight,
        calibratedForward + calibratedRight};

    float speeds[4] = {};
    float peak = 0.0F;
    for (uint8_t i = 0; i < 4; ++i)
    {
        speeds[i] =
            wheelMmPerS[i] * STEPS_PER_MM * MOTOR_SIGN[i] +
            rotationStepsPerS;
        peak = max(peak, fabsf(speeds[i]));
    }

    // 超过脉冲上限时四轮等比缩放，保持运动方向，而不是单轮削顶。
    if (peak > MAX_STEP_RATE)
    {
        const float scale = MAX_STEP_RATE / peak;
        for (float &speed : speeds)
            speed *= scale;
    }

    _driver.setVelocities(speeds, _trackWheelAcceleration);
}

void ChassisControl::finishTranslation()
{
    if (_translationHeadingEnabled &&
        _translationFinalHeadingSettleEnabled)
    {
        _translationPhase = TranslationPhase::HeadingSettling;
        resetYawStability();
        return;
    }

    _state = State::Idle;
    LOGF_INFO("Chassis", "MOVE_COMPLETE pose=(%.1f, %.1f, %.1f°) duration=%lums",
              _worldPose.xMm, _worldPose.yMm, _worldPose.yawDeg,
              millis() - _motionStartMs);
}

void ChassisControl::updateRotation()
{
    if (!imuReady())
    {
        setFault("IMU stale while rotating");
        return;
    }

    if (updateYawSettle(_rotateTargetDeg, ROTATE_TOLERANCE_DEG))
    {
        _state = State::Idle;
        const float turnedDeg = wrap180(_yawDeg - _rotateStartYawDeg);
        LOGF_INFO("Chassis", "ROTATE_COMPLETE current=%.1f° target=%.1f° duration=%lums stepsPerDeg=%.1f",
                  _yawDeg, _rotateTargetDeg, millis() - _motionStartMs,
                  fabsf(turnedDeg) > 10.0F
                      ? fabsf((rotationSteps() - _rotateStartSteps) / turnedDeg)
                      : 0.0F);
    }
}

void ChassisControl::updateStopping()
{
    if (_driver.allStopped())
    {
        _driver.halt();
        _state = State::Idle;
    }
}

bool ChassisControl::updateYawSettle(
    float targetYawDeg,
    float toleranceDeg)
{
    const float error = wrap180(targetYawDeg - _yawDeg);
    if (fabsf(error) <= toleranceDeg)
    {
        // 进入容差带后停止继续发脉冲；若被惯性带出，再从零平滑纠偏。
        // 每轮都下发零速，同时刷新脉冲中断的速度看门狗。
        _rotateCommandSpeed = 0.0F;
        const float zero[4] = {};
        _driver.setVelocities(
            zero,
            rpmToStepsPerSecond(ROTATE_ACCEL_RPM_PER_S));

        /*
         * loop频率远高于IMU输出频率。只有收到一帧新的有效角度数据
         * 才增加稳定计数，避免在同一帧上空转五次就误判完成。
         */
        if (_lastEvaluatedImuSequence == _imuSequence)
            return false;

        _lastEvaluatedImuSequence = _imuSequence;
        if (_stableSamples == 0)
            _stableSinceMs = millis();
        if (_stableSamples < UINT8_MAX)
            ++_stableSamples;

        if (_stableSamples >= ROTATE_STABLE_SAMPLES &&
            millis() - _stableSinceMs >= ROTATE_STABLE_TIME_MS)
        {
            _driver.halt();
            return true;
        }
        return false;
    }

    if (_lastEvaluatedImuSequence != _imuSequence)
    {
        _lastEvaluatedImuSequence = _imuSequence;
        _stableSamples = 0;
        _stableSinceMs = 0;
    }
    // 判定用实测角度，控制用外推角度，补偿IMU帧间延迟。
    runYawController(wrap180(targetYawDeg - predictedYawDeg()));
    return false;
}

void ChassisControl::runYawController(float errorDeg)
{
    const float speed = rotationProfileSpeed(errorDeg);
    // 当前接线下，四个电机同号脉冲对应原地旋转。
    const float speeds[4] = {speed, speed, speed, speed};
    _driver.setVelocities(
        speeds,
        rpmToStepsPerSecond(ROTATE_ACCEL_RPM_PER_S));
}

bool ChassisControl::updatePoseHeadingSettle(float targetYawDeg)
{
    driveBodyVelocity(0.0F, 0.0F, 0.0F);
    const float error = wrap180(targetYawDeg - _yawDeg);
    if (fabsf(error) > ROTATE_TOLERANCE_DEG)
    {
        _translationPhase = TranslationPhase::Driving;
        _trackTurnStarted = true;
        _trackYawReached = false;
        resetYawStability();
        return true;
    }

    if (_lastEvaluatedImuSequence == _imuSequence)
        return false;
    _lastEvaluatedImuSequence = _imuSequence;
    if (_trackPoseStableSamples == 0)
        _trackPoseStableSinceMs = millis();
    if (_trackPoseStableSamples < UINT8_MAX)
        ++_trackPoseStableSamples;
    if (_trackPoseStableSamples < ROTATE_STABLE_SAMPLES ||
        millis() - _trackPoseStableSinceMs < ROTATE_STABLE_TIME_MS)
        return false;

    _trackYawReached = true;
    _trackTurnStarted = false;
    _holdYawDeg = _trackTargetYawDeg;
    _translationPhase = TranslationPhase::Driving;
    updateTrackWheelAcceleration();
    return true;
}

float ChassisControl::rotationProfileSpeed(float errorDeg)
{
    if (fabsf(errorDeg) <= ROTATE_TOLERANCE_DEG)
        return 0.0F;

    /*
     * 远处按v=√(2a(θ-θl/2))减速，θl=a/K²以内改为v=Kθ，两段在θl处连续。
     * 纯√曲线在零点斜率无穷大，IMU延迟会直接变成过冲和来回震荡。
     */
    const float deceleration =
        rpmToStepsPerSecond(ROTATE_ACCEL_RPM_PER_S) * ROTATE_DECEL_RATIO;
    const float errorSteps = fabsf(errorDeg) * ROTATION_STEPS_PER_DEG;
    const float linearSteps =
        deceleration / (ROTATE_FINAL_GAIN_PER_S * ROTATE_FINAL_GAIN_PER_S);
    const float profileSpeed =
        errorSteps <= linearSteps
            ? ROTATE_FINAL_GAIN_PER_S * errorSteps
            : sqrtf(2.0F * deceleration * (errorSteps - 0.5F * linearSteps));
    const float minimumSpeed =
        fabsf(errorDeg) < ROTATE_SMALL_ANGLE_DEG
            ? ROTATE_SMALL_MIN_STEPS_PER_S
            : ROTATE_MIN_STEPS_PER_S;
    const float speed = constrain(
        profileSpeed,
        minimumSpeed,
        rpmToStepsPerSecond(ROTATE_MAX_RPM));
    return slewRotationCommand(copysignf(speed, errorDeg));
}

long ChassisControl::rotationSteps() const
{
    // 四轮原始脉冲之和中平移分量按MOTOR_SIGN抵消，只剩旋转分量。
    long steps[4] = {};
    _driver.positions(steps);
    return (steps[0] + steps[1] + steps[2] + steps[3]) / 4;
}

float ChassisControl::slewRotationCommand(float targetSpeed)
{
    const float acceleration =
        rpmToStepsPerSecond(ROTATE_ACCEL_RPM_PER_S);
    /*
     * 按真实时间限制每次速度变化量，使ROTATE_ACCEL_RPM_PER_S实际生效。
     * 脉冲中断也按同一加速度限速，这里的斜坡保证输出本身平滑。
     */
    const uint32_t nowUs = micros();
    const float elapsedSeconds = min(
        static_cast<float>(nowUs - _rotateLastControlUs) * 1.0e-6F,
        0.05F);
    _rotateLastControlUs = nowUs;
    const float maximumDelta = acceleration * elapsedSeconds;
    _rotateCommandSpeed += constrain(
        targetSpeed - _rotateCommandSpeed,
        -maximumDelta,
        maximumDelta);
    return _rotateCommandSpeed;
}

void ChassisControl::resetYawStability()
{
    _stableSamples = 0;
    _stableSinceMs = 0;
    _lastEvaluatedImuSequence = _imuSequence;
    _rotateCommandSpeed = 0.0F;
    _rotateLastControlUs = micros();
}

void ChassisControl::setFault(const char *message)
{
    strncpy(_fault, message, sizeof(_fault) - 1);
    _fault[sizeof(_fault) - 1] = '\0';
    _driver.halt();
    _state = State::Fault;

    LOGF_ERROR("Chassis", "FAULT: %s", message);
}

float ChassisControl::wrap180(float angleDeg)
{
    if (!isfinite(angleDeg))
        return 0.0f;

    angleDeg = fmodf(angleDeg + 180.0f, 360.0f);
    if (angleDeg < 0.0f)
        angleDeg += 360.0f;
    return angleDeg - 180.0f;
}

float ChassisControl::rpmToStepsPerSecond(float rpm)
{
    return rpm * STEPS_PER_REV / 60.0f;
}

float ChassisControl::distanceScale(
    float distance,
    float positiveScale,
    float negativeScale)
{
    return distance >= 0.0f ? positiveScale : negativeScale;
}

uint32_t ChassisControl::estimateTranslationTimeoutMs(
    long maximumPulses,
    float maximumSpeed,
    float maximumAcceleration)
{
    const float distance = static_cast<float>(labs(maximumPulses));
    const float accelerationDistance =
        maximumSpeed * maximumSpeed / maximumAcceleration;

    float theoreticalSeconds = 0.0f;
    if (distance <= accelerationDistance)
    {
        theoreticalSeconds =
            2.0f * sqrtf(distance / maximumAcceleration);
    }
    else
    {
        theoreticalSeconds =
            2.0f * maximumSpeed / maximumAcceleration +
            (distance - accelerationDistance) / maximumSpeed;
    }

    const float estimatedMs =
        theoreticalSeconds * 1000.0f * MOTION_TIMEOUT_SCALE +
        static_cast<float>(MOTION_TIMEOUT_MARGIN_MS);
    return static_cast<uint32_t>(constrain(
        estimatedMs,
        static_cast<float>(MOTION_TIMEOUT_MIN_MS),
        static_cast<float>(MOTION_TIMEOUT_MAX_MS)));
}

uint32_t ChassisControl::estimateRotationTimeoutMs(
    float absoluteErrorDeg)
{
    const float estimatedMs =
        static_cast<float>(ROTATION_TIMEOUT_BASE_MS) +
        absoluteErrorDeg * ROTATION_TIMEOUT_PER_DEG_MS;
    return static_cast<uint32_t>(constrain(
        estimatedMs,
        static_cast<float>(MOTION_TIMEOUT_MIN_MS),
        static_cast<float>(MOTION_TIMEOUT_MAX_MS)));
}

float ChassisControl::rotationWindowMm(
    float absoluteErrorDeg,
    float pathSpeedMmPerS)
{
    if (absoluteErrorDeg <= ROTATE_TOLERANCE_DEG ||
        pathSpeedMmPerS <= 0.0F)
        return POSE_MOVE_TURN_MIN_WINDOW_MM;

    const float rotationSeconds =
        static_cast<float>(estimateRotationTimeoutMs(absoluteErrorDeg)) *
        0.001F;
    return pathSpeedMmPerS * rotationSeconds * POSE_MOVE_TURN_MARGIN_RATIO;
}

void ChassisControl::bodyToWorld(
    float forwardMm,
    float rightMm,
    float yawDeg,
    float &worldXMm,
    float &worldYMm)
{
    const float radians = yawDeg * PI / 180.0F;
    const float cosine = cosf(radians);
    const float sine = sinf(radians);
    /*
     * yaw=0时：车体前进对应世界+y，车体向右对应世界-x。
     * yaw正方向沿用JY901和底盘旋转控制的正方向。
     */
    worldXMm = sine * forwardMm - cosine * rightMm;
    worldYMm = cosine * forwardMm + sine * rightMm;
}

void ChassisControl::worldToBody(
    float worldXMm,
    float worldYMm,
    float yawDeg,
    float &forwardMm,
    float &rightMm)
{
    const float radians = yawDeg * PI / 180.0F;
    const float cosine = cosf(radians);
    const float sine = sinf(radians);
    forwardMm = sine * worldXMm + cosine * worldYMm;
    rightMm = -cosine * worldXMm + sine * worldYMm;
}
