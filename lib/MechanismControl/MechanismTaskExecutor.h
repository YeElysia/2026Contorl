#pragma once

#include <Arduino.h>
#include <FashionStar_UartServo.h>
#include <TTL_STEPPER.h>

#include "GraspMotionPorts.h"
#include "GraspVisionPorts.h"
#include "MissionPorts.h"
#include "mechanism_config.h"

/**
 * @brief 非阻塞机械臂工位任务执行器。
 *
 * 高层只提交“取料/粗加工/码垛”任务。本类把任务展开成基础动作表，
 * 同一动作组中的独立执行器并行运动，动作组之间保持必要的安全顺序。
 * 升降轴禁止与任何其他执行器并行，所在动作组始终只有升降动作。
 * 转换工位姿态时先升到安全高度再移动其他轴；下降到工作高度则
 * 等其他轴定位完成后执行。
 * 每次update只下发命令或轮询状态，不使用delay和阻塞wait。
 */
class MechanismTaskExecutor : public IStationTaskExecutor
{
public:
    /**
     * @brief 舵机动作的只读诊断快照。
     *
     * status是FashionStar协议状态码：0成功，8响应超时。
     * validPolls/polls可区分“舵机未到位”和“完全没有有效反馈”。
     */
    struct ServoDebugState
    {
        float target = 0.0F;
        float actual = 0.0F;
        uint16_t polls = 0;
        uint16_t validPolls = 0;
        uint16_t power = 0;
        uint8_t status = 0xFF;
        bool issued = false;
        bool hasActual = false;
        bool hasPower = false;
    };

    struct GraspDebugState
    {
        int16_t dx = 0;
        int16_t dy = 0;
        uint8_t quality = 0;
        uint8_t item = 0;
        float forwardOffsetMm = 0.0F;
        float extensionTarget = 0.0F;
        bool tracking = false;
        bool found = false;
        bool hasObservation = false;
    };

    MechanismTaskExecutor(
        HardwareSerial &stepperSerial,
        HardwareSerial &baseSerial,
        HardwareSerial &servoSerial,
        IGraspVisionProvider &graspVision,
        IGraspForwardPositioner &forwardPositioner);

    /**
     * @brief 初始化总线并让机构到达上电初始位置。
     *
     * 初始化动作由update异步完成。若期间按下启动键，
     * prepareForTravel()会直接切换到运输收纳动作，与底盘并行。
     */
    void begin();

    bool ready() const override;
    const char *faultMessage() const override;
    const char *debugPhase() const;
    const ServoDebugState &storageServoDebug() const;
    const ServoDebugState &gripperServoDebug() const;
    const GraspDebugState &graspDebug() const;
    bool prepareForTravel(
        TravelDestination destination) override;

    bool start(
        StationTask task,
        uint8_t round,
        const BatchMission &batch) override;
    void update(bool allowBlockingFeedback = true) override;
    AsyncResult result() const override;
    void cancel() override;

private:
    enum class StepKind : uint8_t
    {
        Lift,
        Extend,
        RotateBase,
        RotateStorage,
        OpenGripper,
        OpenGripperMax,
        CloseGripperUnloaded,
        CloseGripper
    };

    struct ActionStep
    {
        StepKind kind;
        float target;
        uint8_t group;
        bool issued;
        bool completed;
        uint32_t startedMs;
        uint32_t lastPollMs;
        uint8_t stableFeedbackCount;
        uint8_t faultFeedbackCount;
        float previousFeedbackAngle;
        bool hasPreviousFeedback;
    };

    enum class TaskPhase : uint8_t
    {
        Initializing,
        PreparingForTravel,
        CollectPreparing,
        CollectAligning,
        CollectDepositing,
        CollectReturningToRoute,
        RoughPlacing,
        RoughRetrieving,
        FinalStoring,
        Idle
    };

    static constexpr uint8_t MAX_ACTION_STEPS = 20;

    HardwareSerial &_stepperSerial;
    HardwareSerial &_baseSerial;
    HardwareSerial &_servoSerial;
    IGraspVisionProvider &_graspVision;
    IGraspForwardPositioner &_forwardPositioner;

    TTL_Protocol _stepperProtocol;
    TTL_Protocol _baseProtocol;
    TTL_Stepper _lift;
    TTL_Stepper _extension;
    TTL_Stepper _base;

    FSUS_Protocol _servoProtocol;
    FSUS_Servo _storageServo;
    FSUS_Servo _gripperServo;
    ServoDebugState _storageServoDebug;
    ServoDebugState _gripperServoDebug;
    GraspDebugState _graspDebug;

    ActionStep _steps[MAX_ACTION_STEPS] = {};
    uint8_t _stepCount = 0;
    uint8_t _stepIndex = 0;
    uint8_t _nextStepGroup = 0;
    TTL_Stepper *_sharedPollOwner = nullptr;
    uint32_t _lastSharedBusPollMs = 0;
    uint32_t _alignmentStartedMs = 0;
    uint32_t _lastObservationMs = 0;
    uint32_t _alignmentObservationAfterMs = 0;
    uint8_t _stableFrames = 0;
    float _alignmentForwardOffset = 0.0F;
    float _alignmentExtensionTarget = 0.0F;
    bool _forwardCommandActive = false;
    bool _allowBlockingServoFeedback = true;

    TaskPhase _phase = TaskPhase::Idle;
    BatchMission _batch = {};
    uint8_t _round = 0;
    uint8_t _itemIndex = 0;
    bool _initialized = false;
    AsyncResult _result = AsyncResult::Idle;
    const char *_fault = "";

    void clearAction();
    void addStep(StepKind kind, float target);
    void addConcurrentStep(StepKind kind, float target);
    void appendStep(
        StepKind kind,
        float target,
        uint8_t group);
    bool lastGroupContains(StepKind kind) const;
    void addSafeRetraction(float baseTarget);
    void addStorageDeposit();
    void loadInitializationAction();
    void loadHomeAction(float liftTarget);
    void loadTurntablePreparationAction(uint8_t traySlot);
    void loadTurntablePickupToStorageAction();
    void loadStorageToRingAction(
        uint8_t traySlot,
        const mechanism_config::RingPose &pose,
        uint8_t stackLevel);
    void loadRingToStorageAction(
        uint8_t traySlot,
        const mechanism_config::RingPose &pose);

    bool updateCurrentStep();
    bool updateActionStep(ActionStep &step);
    void startPickupAlignment();
    void updatePickupAlignment();
    void startReturnToMaterialRouteAnchor();
    void updateReturnToMaterialRouteAnchor();
    bool updateStepperStep(
        TTL_Stepper &motor,
        ActionStep &step,
        bool angle);
    bool pollStepperState(
        TTL_Stepper &motor,
        ActionStep &step,
        uint32_t now);
    void issueServoStep(ActionStep &step);
    bool updateServoStep(ActionStep &step);
    bool queryServoAngle(
        FSUS_Servo &servo,
        ServoDebugState &debug,
        float &angle);
    void onActionCompleted();
    void finishStationTask();
    const char *stepperFaultMessage(
        const TTL_Stepper &motor,
        bool protection) const;
    const char *stepperCommandFaultMessage(
        const TTL_Stepper &motor) const;
    void fail(const char *message);

    static void resetStepperState(TTL_Stepper &motor);
    static bool validRing(uint8_t ring);
    static float clampValue(float value, float minimum, float maximum);
};
