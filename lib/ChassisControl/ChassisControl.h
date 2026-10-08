#pragma once

#include "ChassisStepperDriver.h"
#include "ChassisTracking.h"
#include <Arduino.h>

class ChassisEmm42TtlFeedback;

/**
 * @brief 当前允许下发底盘运动的模块。
 *
 * 同一时刻只有一个模块拥有底盘；其他模块经ChassisMotionPort下发
 * 的命令会被拒绝，避免路线、对准和抓取微调互相覆盖。
 */
enum class ChassisOwner : uint8_t
{
    None,
    Route,
    Alignment,
    Grasp
};

const char *chassisOwnerName(ChassisOwner owner);

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
        Stopping,
        Fault
    };

    /**
     * @brief 创建底盘控制器。
     *
     * imuSerial 传入 nullptr 时不执行航向保持或绝对角度旋转。
     * motorFeedback 传入 nullptr 时不初始化或占用EMM42 TTL串口。
     */
    explicit ChassisControl(
        HardwareSerial *imuSerial = nullptr,
        ChassisEmm42TtlFeedback *motorFeedback = nullptr);

    void begin();
    void update();

    /**
     * @brief 以车体坐标系执行一次相对平移。
     *
     *  forwardMm > 0：向车头方向前进；forwardMm < 0：后退。
     *  rightMm   > 0：向车体右侧横移；rightMm   < 0：向左横移。
     *  两个分量可同时给出，从而执行斜向移动。
     *
     *  车体方向按航向参考（最近一次旋转目标）换算成世界目标点，
     *  移动中闭环跟踪直线路径并锁定该航向。
     *  maxRpm和accelerationRpmPerS是最大轮速与轮加速度，
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
     * @brief 视觉追踪专用短距离平移。
     *
     * 运动中仍保持IMU航向，但到达本段位置后不再额外等待最终航向
     * 静定；下一视觉帧可立即提交新修正。只供工位视觉闭环使用。
     */
    bool moveBodyRelativeTracking(
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
     * 与车体相对移动共用同一套闭环路径跟踪和航向保持逻辑。
     */
    bool moveWorldRelative(
        float worldXMm,
        float worldYMm,
        float maxRpm,
        float accelerationRpmPerS);

    /**
     * @brief 移动到世界坐标系中的绝对位置。
     *
     * 从当前估计位姿沿直线跟踪到目标点。适合比赛主路线和区域
     * 基准点，不用于视觉小步微调。
     */
    bool moveWorldTo(
        float targetWorldXMm,
        float targetWorldYMm,
        float maxRpm,
        float accelerationRpmPerS);

    /**
     * @brief 经途经点连续平移到目标，途经点不停车，全程保持航向。
     *
     * 第一段参考距途经点剩TRACK_BLEND_DISTANCE_MM时第二段起步，
     * 两段参考叠加，拐角被圆滑内切。只在最终目标做航向静定。
     */
    bool moveWorldThrough(
        float viaWorldXMm,
        float viaWorldYMm,
        float targetWorldXMm,
        float targetWorldYMm,
        float maxRpm,
        float accelerationRpmPerS);

    /**
     * @brief 一条指令平移并转向到世界位姿（麦轮漂移入位），中途不停车。
     *
     * 行进方向取起步航向下最接近目标的车体轴，长直线保持纯前后或
     * 纯左右运动，保持起步航向。距终点POSE_MOVE_TURN_WINDOW_MM处
     * 开始旋转，此后平移进度受IMU实测转向进度限制；相对该轴的横向
     * 偏差也只在窗口内并入。转到位后走完剩余平移并做最终航向静定。
     * 调用方需保证窗口内路径都有旋转余量。
     */
    bool moveWorldToPose(
        float targetWorldXMm,
        float targetWorldYMm,
        float worldYawDeg,
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
    /**
     * @brief 使用固定地标修正世界坐标位置，不改变IMU航向基准。
     *
     * 仅允许停车后调用；同步里程计基线可避免修正前的脉冲增量
     * 在下一次update()时被重复累计。
     */
    bool correctWorldPosition(float worldXmm, float worldYmm);
    const char *faultMessage() const;

    // 未被占用或已由owner占用时返回true。
    bool acquire(ChassisOwner owner);
    // 只有当前拥有者才能释放。
    void release(ChassisOwner owner);
    ChassisOwner owner() const;

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
    enum class TranslationPhase : uint8_t
    {
        Driving,
        PoseHeadingSettling,
        Braking,
        HeadingSettling
    };

    HardwareSerial *_imuSerial;
    ChassisEmm42TtlFeedback *_motorFeedback;
    ChassisStepperDriver _driver;

    State _state = State::Idle;
    ChassisOwner _owner = ChassisOwner::None;
    char _fault[80] = {};

    uint8_t _imuFrame[11] = {};
    uint8_t _imuIndex = 0;
    float _yawDeg = 0.0f;
    bool _imuReady = false;
    uint32_t _lastImuMs = 0;
    long _imuFrameRotationSteps = 0;
    uint32_t _imuSequence = 0;
    uint32_t _lastEvaluatedImuSequence = 0;

    Pose2D _worldPose = {0.0F, 0.0F, 0.0F};
    float _worldYawOffsetDeg = 0.0F;
    // 世界航向可由resetWorldPose预先给定；IMU零偏必须等首帧后建立。
    bool _worldYawReady = false;
    bool _worldYawOffsetReady = false;
    long _lastOdometrySteps[4] = {};

    // 平移锁定的IMU航向。旋转命令把它设为旋转目标，平移时不再取起步航向。
    float _headingReferenceDeg = 0.0f;
    bool _headingReferenceValid = false;
    float _holdYawDeg = 0.0f;

    // 当前平移的世界坐标直线路径与速度限制（mm、mm/s、mm/s²）。
    float _trackStartXMm = 0.0f;
    float _trackStartYMm = 0.0f;
    float _trackTargetXMm = 0.0f;
    float _trackTargetYMm = 0.0f;
    float _trackDirectionX = 0.0f;
    float _trackDirectionY = 0.0f;
    // 位姿平移：目标相对行进轴的横偏、转向窗口和IMU系目标航向。
    // 普通平移窗口为0、横偏为0，_trackYawReached恒为true。
    float _trackTargetCrossMm = 0.0f;
    float _trackTurnWindowMm = 0.0f;
    float _trackTargetYawDeg = 0.0f;
    float _trackYawTotalDeg = 0.0f;
    bool _trackTurnStarted = false;
    bool _trackYawReached = true;
    float _trackMaximumSpeedMm = 0.0f;
    float _trackAccelerationMm = 0.0f;
    float _trackJerkMm = 0.0f;
    float _trackLengthMm = 0.0f;
    // 沿程S曲线参考；实际速度=参考速度+位置误差修正。
    chassis_tracking::JerkState _trackProfile = {};
    float _trackAlongSpeedMm = 0.0f;
    float _trackMaximumWheelSpeed = 0.0f;
    float _trackWheelAcceleration = 0.0f;
    uint32_t _trackLastControlUs = 0;
    float _trackHoldWorldYawDeg = 0.0f;
    float _trackMaxRpm = 0.0f;
    float _trackAccelerationRpmPerS = 0.0f;
    // 途经点连续平移：待起步的下一段终点与叠加距离。
    bool _trackNextPending = false;
    float _trackNextTargetXMm = 0.0f;
    float _trackNextTargetYMm = 0.0f;
    float _trackNextYawDeg = 0.0f;
    float _trackBlendMm = 0.0f;
    // 已切到下一段后，上一段尚未走完的参考继续叠加。
    bool _trackBlendActive = false;
    float _trackBlendDirectionX = 0.0f;
    float _trackBlendDirectionY = 0.0f;
    float _trackBlendLengthMm = 0.0f;
    float _trackBlendMaximumSpeedMm = 0.0f;
    float _trackBlendAccelerationMm = 0.0f;
    float _trackBlendJerkMm = 0.0f;
    chassis_tracking::JerkState _trackBlendProfile = {};
    bool _translationHeadingEnabled = false;
    bool _translationFinalHeadingSettleEnabled = true;
    TranslationPhase _translationPhase = TranslationPhase::Driving;
    float _rotateTargetDeg = 0.0f;
    float _rotateCommandSpeed = 0.0f;
    uint32_t _rotateLastControlUs = 0;
    long _rotateStartSteps = 0;
    float _rotateStartYawDeg = 0.0f;
    uint8_t _stableSamples = 0;
    uint32_t _stableSinceMs = 0;
    uint32_t _motionStartMs = 0;
    uint32_t _motionTimeoutMs = 0;
    uint32_t _trackPoseStableSinceMs = 0;
    uint8_t _trackPoseStableSamples = 0;

    bool updateImu();
    float predictedYawDeg() const;
    float predictedWorldYawDeg() const;
    bool moveBodyRelativeInternal(
        float forwardMm,
        float rightMm,
        float maxRpm,
        float accelerationRpmPerS,
        bool finalHeadingSettle);
    float prepareTranslationHeading();
    bool startTranslation(
        float targetWorldXMm,
        float targetWorldYMm,
        float holdWorldYawDeg,
        float maxRpm,
        float accelerationRpmPerS,
        bool finalHeadingSettle,
        float turnTargetYawDeg = NAN);
    void startTrackTurn(float alongMm);
    float configureTrackLimits();
    float setupTrackSegment(
        float startWorldXMm,
        float startWorldYMm,
        float targetWorldXMm,
        float targetWorldYMm,
        float turnTargetYawDeg);
    uint32_t segmentTimeoutMs(float wheelSteps) const;
    void updateTrackWheelAcceleration();
    bool updatePoseHeadingSettle(float targetYawDeg);
    void startNextTrackSegment();
    void driveBodyVelocity(
        float forwardMmPerS,
        float rightMmPerS,
        float rotationStepsPerS);
    void finishTranslation();
    void updateOdometry();
    void updateTranslation();
    void updateRotation();
    void updateStopping();
    bool updateYawSettle(float targetYawDeg, float toleranceDeg);
    void runYawController(float errorDeg);
    float rotationProfileSpeed(float errorDeg);
    float slewRotationCommand(float targetSpeed);
    long rotationSteps() const;
    void resetYawStability();
    void setFault(const char *message);

    static float distanceScale(
        float distance,
        float positiveScale,
        float negativeScale);
    static uint32_t estimateTranslationTimeoutMs(
        long maximumPulses,
        float maximumSpeed,
        float maximumAcceleration);
    static uint32_t estimateRotationTimeoutMs(float absoluteErrorDeg);
    static float rotationWindowMm(float absoluteErrorDeg, float pathSpeedMmPerS);
    static float wrap180(float angleDeg);
    static float rpmToStepsPerSecond(float rpm);
};
