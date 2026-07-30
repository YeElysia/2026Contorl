#pragma once

#include <AccelStepper.h>
#include <Arduino.h>

class ChassisControl
{
public:
    /**
     * @brief 机器人在世界坐标系中的估计位姿。
     *
     * 未校准时以启动位置为(0,0,0)；比赛程序会通过
     * resetWorldPose()写入实际场地起点。x轴正方向指向初始
     * 车体左侧，y轴正方向指向初始车头。
     */
    struct Pose2D
    {
        float xMm;
        float yMm;
        float yawDeg;
    };

    enum class State : uint8_t
    {
        Idle,
        Translating,
        Rotating,
        Fault
    };

    /**
     * @brief 创建底盘控制器。
     *
     * imuSerial 传入 nullptr 时仅启用基础步进电机位置控制，
     * 不初始化任何串口，也不执行航向保持或绝对角度旋转。
     */
    explicit ChassisControl(HardwareSerial *imuSerial = nullptr);

    void begin();
    void update();

    /**
     * @brief 以车体坐标系执行一次相对平移。
     *
     *  forwardMm > 0：向车头方向前进；forwardMm < 0：后退。
     *  rightMm   > 0：向车体右侧横移；rightMm   < 0：向左横移。
     *  两个分量可同时给出，从而执行斜向移动。
     *
     *  运动开始时记录当前 IMU 航向，移动过程中自动抑制偏航。
     *  maxRpm和accelerationRpmPerS分别指定轮子转速与加速度，
     *  供路线执行器选择快速或精确档。
     *  本函数只下发目标，不阻塞等待；必须在 loop() 中持续调用 update()。
     *
     * @return 成功接受命令返回 true；底盘忙碌或处于故障状态返回 false。
     */
    bool moveBodyRelative(
        float forwardMm,
        float rightMm,
        float maxRpm,
        float accelerationRpmPerS);

    /**
     * @brief 兼容现有调用的车体坐标相对移动接口。
     */
    bool moveRelative(
        float forwardMm,
        float rightMm,
        float maxRpm,
        float accelerationRpmPerS);

    /**
     * @brief 按世界坐标增量移动（+x向初始左侧，+y向初始车头）。
     *
     * 本函数在下发时将世界坐标增量转换为当前车体的前后/左右
     * 分量，随后仍由同一套麦轮控制和航向保持逻辑执行。
     */
    bool moveWorldRelative(
        float worldXMm,
        float worldYMm,
        float maxRpm,
        float accelerationRpmPerS);

    /**
     * @brief 移动到世界坐标系中的绝对位置。
     *
     * 执行时根据当前估计位姿计算世界坐标差值，再转换为车体
     * 坐标运动。适合比赛主路线和区域基准点，不用于视觉小步微调。
     */
    bool moveWorldTo(
        float targetWorldXMm,
        float targetWorldYMm,
        float maxRpm,
        float accelerationRpmPerS);

    bool rotateTo(float absoluteYawDeg);
    bool rotateWorldTo(float worldYawDeg);
    void stop();
    void clearFault();

    State state() const;
    bool busy() const;
    bool imuReady() const;
    float yawDeg() const;
    Pose2D worldPose() const;
    bool resetWorldPose(
        float worldXmm = 0.0F,
        float worldYmm = 0.0F,
        float worldYawDeg = 0.0F);
    const char *faultMessage() const;

    static void bodyToWorld(
        float forwardMm,
        float rightMm,
        float yawDeg,
        float &worldXMm,
        float &worldYMm);
    static void worldToBody(
        float worldXMm,
        float worldYMm,
        float yawDeg,
        float &forwardMm,
        float &rightMm);

private:
    HardwareSerial *_imuSerial;
    AccelStepper _motors[4];

    State _state = State::Idle;
    char _fault[80] = {};

    uint8_t _imuFrame[11] = {};
    uint8_t _imuIndex = 0;
    float _yawDeg = 0.0f;
    bool _imuReady = false;
    uint32_t _lastImuMs = 0;

    Pose2D _worldPose = {0.0F, 0.0F, 0.0F};
    float _worldYawOffsetDeg = 0.0F;
    // 世界航向可由resetWorldPose预先给定；IMU零偏必须等首帧后建立。
    bool _worldYawReady = false;
    bool _worldYawOffsetReady = false;
    long _lastOdometrySteps[4] = {};

    float _holdYawDeg = 0.0f;
    // 当前平移中各轮按行程比例分配的巡航速度。
    // 例如轮子行程为 [700, 300, 300, 700]，速度比例就是
    // [1, 3/7, 3/7, 1]，从而保证四轮同时到位。
    float _translationSpeed[4] = {};
    float _activeTranslationMaximumSpeed = 0.0f;
    bool _translationHeadingEnabled = false;
    float _rotateTargetDeg = 0.0f;
    uint8_t _stableSamples = 0;
    uint32_t _motionStartMs = 0;

    bool updateImu();
    void updateOdometry();
    void updateTranslation();
    void updateRotation();
    void setFault(const char *message);
    void syncTargets();
    bool allMotorsStopped();

    static float wrap180(float angleDeg);
    static float rpmToStepsPerSecond(float rpm);
};
