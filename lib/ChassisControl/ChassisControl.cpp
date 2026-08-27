#include "ChassisControl.h"

#include "ChassisEmm42TtlFeedback.h"
#include "chassis_config.h"
#include <math.h>
#include <string.h>

using namespace chassis_config;

ChassisControl::ChassisControl(
    HardwareSerial *imuSerial,
    ChassisEmm42TtlFeedback *motorFeedback)
    : _imuSerial(imuSerial),
      _motorFeedback(motorFeedback),
      _motors{
          AccelStepper(AccelStepper::DRIVER, STEP_PINS[0], DIR_PINS[0]),
          AccelStepper(AccelStepper::DRIVER, STEP_PINS[1], DIR_PINS[1]),
          AccelStepper(AccelStepper::DRIVER, STEP_PINS[2], DIR_PINS[2]),
          AccelStepper(AccelStepper::DRIVER, STEP_PINS[3], DIR_PINS[3])}
{
}

void ChassisControl::begin()
{
    pinMode(ENABLE_PIN, OUTPUT);
    digitalWrite(ENABLE_PIN, LOW);

    const float maxSpeed = rpmToStepsPerSecond(DRIVE_RPM);
    const float acceleration =
        rpmToStepsPerSecond(DRIVE_ACCEL_RPM_PER_S);
    for (auto &motor : _motors)
    {
        motor.setMaxSpeed(maxSpeed);
        motor.setAcceleration(acceleration);
        motor.setMinPulseWidth(STEP_PULSE_WIDTH_US);
        motor.setCurrentPosition(0);
    }
    for (uint8_t i = 0; i < 4; ++i)
        _lastOdometrySteps[i] = _motors[i].currentPosition();

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

    // 记录本次run()/runSpeed()实际产生的脉冲，避免位姿落后一轮。
    updateOdometry();
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
        !isfinite(maxRpm) ||
        !isfinite(accelerationRpmPerS) ||
        maxRpm <= 0.0f ||
        accelerationRpmPerS <= 0.0f ||
        fabsf(forwardMm) > MAX_SINGLE_MOVE_COMPONENT_MM ||
        fabsf(rightMm) > MAX_SINGLE_MOVE_COMPONENT_MM)
        return false;

    /*
     * 四麦轮平移逆运动学。
     *
     * 电机顺序：左前、右前、左后、右后。
     * 此处先计算每个轮子的等效行程，随后再通过 MOTOR_SIGN
     * 适配各电机的实际安装方向：
     *
     *   左前 = 前进 + 右移
     *   右前 = 前进 - 右移
     *   左后 = 前进 - 右移
     *   右后 = 前进 + 右移
     */
    const float calibratedForwardMm =
        forwardMm * distanceScale(
                        forwardMm,
                        FORWARD_DISTANCE_SCALE,
                        BACKWARD_DISTANCE_SCALE);
    const float calibratedRightMm =
        rightMm * distanceScale(
                      rightMm,
                      RIGHT_DISTANCE_SCALE,
                      LEFT_DISTANCE_SCALE);
    const float wheelMm[4] = {
        calibratedForwardMm + calibratedRightMm,
        calibratedForwardMm - calibratedRightMm,
        calibratedForwardMm - calibratedRightMm,
        calibratedForwardMm + calibratedRightMm};

    bool hasMotion = false;
    long wheelPulses[4] = {};
    long maximumPulses = 0;
    for (uint8_t i = 0; i < 4; ++i)
    {
        wheelPulses[i] =
            lroundf(wheelMm[i] * STEPS_PER_MM) * MOTOR_SIGN[i];
        maximumPulses = max(maximumPulses, labs(wheelPulses[i]));
        hasMotion = hasMotion || wheelPulses[i] != 0;
    }
    if (!hasMotion)
        return true;

    /*
     * 按行程比例同步四轮的速度和加速度。
     *
     * 若 d[i] = ratio[i] * dMax，同时设置
     * v[i] = ratio[i] * vMax、a[i] = ratio[i] * aMax，
     * 四个轮子的加速、匀速和减速阶段会使用相同时间，因此同时到位。
     */
    const float maximumSpeed = rpmToStepsPerSecond(maxRpm);
    const float maximumAcceleration =
        rpmToStepsPerSecond(accelerationRpmPerS);
    _activeTranslationMaximumSpeed = maximumSpeed;
    for (uint8_t i = 0; i < 4; ++i)
    {
        const float ratio =
            static_cast<float>(labs(wheelPulses[i])) /
            static_cast<float>(maximumPulses);
        _translationSpeed[i] = maximumSpeed * ratio;

        if (wheelPulses[i] != 0)
        {
            _motors[i].setMaxSpeed(_translationSpeed[i]);
            _motors[i].setAcceleration(maximumAcceleration * ratio);
            _motors[i].move(wheelPulses[i]);
        }
        else
        {
            // 45°斜移时可能有两个轮子理论行程为零。
            _motors[i].moveTo(_motors[i].currentPosition());
        }
    }

    // 记录起步航向。updateTranslation() 会在整个移动期间保持该角度。
    _holdYawDeg = _yawDeg;
    _translationHeadingEnabled =
        ENABLE_HEADING_HOLD && imuReady();
    _translationFinalHeadingSettleEnabled =
        finalHeadingSettle;
    _translationPhase = TranslationPhase::Driving;
    resetYawStability();
    _motionStartMs = millis();
    _motionTimeoutMs = estimateTranslationTimeoutMs(
        maximumPulses,
        maximumSpeed,
        maximumAcceleration);
    _state = State::Translating;
    return true;
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

    float forwardMm = 0.0F;
    float rightMm = 0.0F;
    worldToBody(
        worldXMm,
        worldYMm,
        _worldPose.yawDeg,
        forwardMm,
        rightMm);
    return moveBodyRelative(
        forwardMm,
        rightMm,
        maxRpm,
        accelerationRpmPerS);
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
    resetYawStability();
    _motionStartMs = millis();
    _motionTimeoutMs = estimateRotationTimeoutMs(
        fabsf(wrap180(_rotateTargetDeg - _yawDeg)));
    _state = State::Rotating;
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
    if (_state == State::Fault)
    {
        syncTargets();
        return;
    }

    if (_state == State::Idle)
    {
        syncTargets();
        return;
    }

    /*
     * 不在stop()里阻塞一秒。给每个轴生成减速目标，后续由update()
     * 持续产生脉冲，这样IMU、视觉和机构状态机不会被底盘急停卡住。
     */
    for (auto &motor : _motors)
        motor.stop();

    _motionStartMs = millis();
    _motionTimeoutMs = STOP_TIMEOUT_MS;
    _state = State::Stopping;
}

void ChassisControl::clearFault()
{
    syncTargets();
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

    for (uint8_t i = 0; i < 4; ++i)
        _lastOdometrySteps[i] = _motors[i].currentPosition();
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
    for (uint8_t i = 0; i < 4; ++i)
        _lastOdometrySteps[i] = _motors[i].currentPosition();
    return true;
}

const char *ChassisControl::faultMessage() const
{
    return _fault;
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
            _yawDeg = rawYaw / 32768.0f * 180.0f;
            _lastImuMs = millis();
            _imuReady = true;
            ++_imuSequence;
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
    float wheelMm[4] = {};
    bool moved = false;
    for (uint8_t i = 0; i < 4; ++i)
    {
        const long current = _motors[i].currentPosition();
        const long deltaSteps = current - _lastOdometrySteps[i];
        _lastOdometrySteps[i] = current;
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
        _worldPose.yawDeg,
        worldXMm,
        worldYMm);
    _worldPose.xMm += worldXMm;
    _worldPose.yMm += worldYMm;
}

void ChassisControl::updateTranslation()
{
    if (_translationHeadingEnabled && !imuReady())
    {
        setFault("IMU stale while translating");
        return;
    }

    if (_translationPhase == TranslationPhase::HeadingSettling)
    {
        if (updateYawSettle(
                _holdYawDeg,
                TRANSLATION_FINAL_HEADING_TOLERANCE_DEG))
            _state = State::Idle;
        return;
    }

    float correction = 0.0f;
    if (_translationHeadingEnabled)
    {
        const float error = wrap180(_holdYawDeg - _yawDeg);
        float maxCorrection =
            _activeTranslationMaximumSpeed *
            HEADING_MAX_CORRECTION_RATIO;

        /*
         * 修正量不能大于任一运动轮的主要平移速度，否则短行程轮
         * 可能被要求反转，而 run() 的目标位置控制无法执行这种反转。
         */
        for (uint8_t i = 0; i < 4; ++i)
        {
            if (_motors[i].distanceToGo() != 0)
                maxCorrection =
                    min(maxCorrection, _translationSpeed[i] * 0.8f);
        }

        correction = constrain(
            error * HEADING_KP_STEPS_PER_S_PER_DEG,
            -maxCorrection, maxCorrection);
    }

    // 同号的有符号轮速修正产生原地旋转，不改变主要平移组合。
    // AccelStepper 的 run() 根据目标位置确定方向，因此这里根据
    // distanceToGo() 的符号把修正量换算成各轮速度幅值。
    for (uint8_t i = 0; i < 4; ++i)
    {
        const long remaining = _motors[i].distanceToGo();
        if (remaining == 0)
            continue;

        const float direction = remaining > 0 ? 1.0f : -1.0f;
        const float signedSpeed =
            direction * _translationSpeed[i] + correction;
        // 只防止零速度；不能设置较高的固定下限，否则会破坏
        // 很短行程轮与长行程轮之间的同步比例。
        _motors[i].setMaxSpeed(max(1.0f, fabsf(signedSpeed)));
        _motors[i].run();
    }

    if (allMotorsStopped())
    {
        syncTargets();
        if (_translationHeadingEnabled &&
            _translationFinalHeadingSettleEnabled)
        {
            /*
             * 行程完成不等于动作完成。巡航阶段的航向修正受各轮剩余
             * 行程约束，最终再做一次小角度原地收敛，避免误差传给
             * 下一段世界坐标路线或视觉对准。
             */
            _translationPhase = TranslationPhase::HeadingSettling;
            resetYawStability();
        }
        else
        {
            _state = State::Idle;
        }
    }
}

void ChassisControl::updateRotation()
{
    if (!imuReady())
    {
        setFault("IMU stale while rotating");
        return;
    }

    if (updateYawSettle(_rotateTargetDeg, ROTATE_TOLERANCE_DEG))
        _state = State::Idle;
}

void ChassisControl::updateStopping()
{
    for (auto &motor : _motors)
        motor.run();

    if (allMotorsStopped())
    {
        syncTargets();
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
        _rotateCommandSpeed = 0.0F;

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
            syncTargets();
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
    runYawController(error);
    return false;
}

void ChassisControl::runYawController(float errorDeg)
{
    const float limit = rpmToStepsPerSecond(ROTATE_MAX_RPM);
    const float acceleration =
        rpmToStepsPerSecond(ROTATE_ACCEL_RPM_PER_S);

    /*
     * 只在收到新IMU帧时更新角速度，避免主循环在同一帧上重复微分。
     * 一阶低通压制JY901角度量化噪声，PD的D项用于接近目标时提前制动。
     */
    if (_rotateLastImuSequence != _imuSequence)
    {
        const uint32_t sampleMs = _lastImuMs;
        const uint32_t elapsedMs = sampleMs - _rotateLastYawMs;
        if (elapsedMs > 0)
        {
            const float instantaneousYawRate =
                wrap180(_yawDeg - _rotateLastYawDeg) *
                1000.0F / static_cast<float>(elapsedMs);
            _rotateFilteredYawRate +=
                ROTATE_YAW_RATE_FILTER_ALPHA *
                (instantaneousYawRate - _rotateFilteredYawRate);
        }
        _rotateLastYawDeg = _yawDeg;
        _rotateLastYawMs = sampleMs;
        _rotateLastImuSequence = _imuSequence;
    }

    float targetSpeed = constrain(
        errorDeg * ROTATE_KP_STEPS_PER_S_PER_DEG -
            _rotateFilteredYawRate *
                ROTATE_KD_STEPS_PER_S_PER_DEG_PER_S,
        -limit,
        limit);
    if (targetSpeed * errorDeg > 0.0F &&
        fabsf(targetSpeed) < ROTATE_MIN_STEPS_PER_S)
    {
        targetSpeed = copysignf(ROTATE_MIN_STEPS_PER_S, errorDeg);
    }

    /*
     * AccelStepper::runSpeed()本身不使用setAcceleration()。这里按真实
     * 时间限制每次速度变化量，使ROTATE_ACCEL_RPM_PER_S实际生效。
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

    // 当前接线下，四个电机同号脉冲对应原地旋转。
    for (auto &motor : _motors)
    {
        /*
         * 前一次斜移可能给四轮留下不同的maxSpeed/acceleration。
         * 原地旋转前必须统一恢复，否则setSpeed()会被各自的旧上限
         * 裁成不同轮速。
         */
        motor.setMaxSpeed(limit);
        motor.setAcceleration(acceleration);
        motor.setSpeed(_rotateCommandSpeed);
        motor.runSpeed();
    }
}

void ChassisControl::resetYawStability()
{
    _stableSamples = 0;
    _stableSinceMs = 0;
    _lastEvaluatedImuSequence = _imuSequence;
    _rotateCommandSpeed = 0.0F;
    _rotateFilteredYawRate = 0.0F;
    _rotateLastYawDeg = _yawDeg;
    _rotateLastYawMs = _lastImuMs;
    _rotateLastImuSequence = _imuSequence;
    _rotateLastControlUs = micros();
}

void ChassisControl::setFault(const char *message)
{
    strncpy(_fault, message, sizeof(_fault) - 1);
    _fault[sizeof(_fault) - 1] = '\0';
    syncTargets();
    _state = State::Fault;
}

void ChassisControl::syncTargets()
{
    for (auto &motor : _motors)
        motor.moveTo(motor.currentPosition());
}

bool ChassisControl::allMotorsStopped()
{
    for (auto &motor : _motors)
    {
        if (motor.distanceToGo() != 0)
            return false;
    }
    return true;
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
