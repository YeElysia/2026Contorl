#include "ChassisControl.h"

#include "chassis_config.h"
#include <math.h>
#include <string.h>

using namespace chassis_config;

ChassisControl::ChassisControl(HardwareSerial *imuSerial)
    : _imuSerial(imuSerial),
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
        motor.setCurrentPosition(0);
    }

    if (_imuSerial != nullptr)
        _imuSerial->begin(IMU_BAUD);

    _state = State::Idle;
}

void ChassisControl::update()
{
    updateImu();

    if (_state != State::Idle &&
        _state != State::Fault &&
        millis() - _motionStartMs > MOTION_TIMEOUT_MS)
    {
        setFault("motion timeout");
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
    default:
        break;
    }
}

bool ChassisControl::moveRelative(float forwardMm, float rightMm)
{
    return moveRelative(
        forwardMm,
        rightMm,
        DRIVE_RPM,
        DRIVE_ACCEL_RPM_PER_S);
}

bool ChassisControl::moveRelative(
    float forwardMm,
    float rightMm,
    float maxRpm,
    float accelerationRpmPerS)
{
    // 一次只允许执行一个运动命令。发生故障后需要先调用 clearFault()。
    if (busy() || _state == State::Fault)
        return false;
    if (maxRpm <= 0.0f || accelerationRpmPerS <= 0.0f)
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
    const float wheelMm[4] = {
        forwardMm + rightMm,
        forwardMm - rightMm,
        forwardMm - rightMm,
        forwardMm + rightMm};

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
    _motionStartMs = millis();
    _state = State::Translating;
    return true;
}

bool ChassisControl::rotateTo(float absoluteYawDeg)
{
    if (busy() || _state == State::Fault)
        return false;
    if (!_imuReady)
    {
        setFault("rotate rejected: IMU not ready");
        return false;
    }

    _rotateTargetDeg = wrap180(absoluteYawDeg);
    _stableSamples = 0;
    _motionStartMs = millis();
    _state = State::Rotating;
    return true;
}

void ChassisControl::stop()
{
    for (auto &motor : _motors)
        motor.stop();

    const uint32_t start = millis();
    while (!allMotorsStopped() && millis() - start < 1000)
    {
        for (auto &motor : _motors)
            motor.run();
    }
    syncTargets();
    if (_state != State::Fault)
        _state = State::Idle;
}

void ChassisControl::clearFault()
{
    stop();
    _fault[0] = '\0';
    _state = State::Idle;
}

ChassisControl::State ChassisControl::state() const
{
    return _state;
}

bool ChassisControl::busy() const
{
    return _state == State::Translating || _state == State::Rotating;
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
            updated = true;
        }
    }
    return updated;
}

void ChassisControl::updateTranslation()
{
    if (_translationHeadingEnabled && !imuReady())
    {
        setFault("IMU stale while translating");
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
        _state = State::Idle;
    }
}

void ChassisControl::updateRotation()
{
    if (!imuReady())
    {
        setFault("IMU stale while rotating");
        return;
    }

    const float error = wrap180(_rotateTargetDeg - _yawDeg);
    if (fabsf(error) <= ROTATE_TOLERANCE_DEG)
    {
        if (++_stableSamples >= ROTATE_STABLE_SAMPLES)
        {
            syncTargets();
            _state = State::Idle;
        }
        return;
    }
    _stableSamples = 0;

    const float limit = rpmToStepsPerSecond(ROTATE_MAX_RPM);
    const float speed = constrain(
        error * ROTATE_KP_STEPS_PER_S_PER_DEG, -limit, limit);

    // 当前接线下，四个电机同号脉冲对应原地旋转。
    for (auto &motor : _motors)
    {
        motor.setSpeed(speed);
        motor.runSpeed();
    }
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
    while (angleDeg > 180.0f)
        angleDeg -= 360.0f;
    while (angleDeg < -180.0f)
        angleDeg += 360.0f;
    return angleDeg;
}

float ChassisControl::rpmToStepsPerSecond(float rpm)
{
    return rpm * STEPS_PER_REV / 60.0f;
}
