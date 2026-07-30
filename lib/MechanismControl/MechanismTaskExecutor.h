#pragma once

#include <Arduino.h>
#include <FashionStar_UartServo.h>
#include <TTL_STEPPER.h>

#include "GraspVisionPorts.h"
#include "MissionPorts.h"

/**
 * @brief 非阻塞机械臂工位任务执行器。
 *
 * 高层只提交“取料/粗加工/码垛”任务。本类把任务展开成基础动作表，
 * 同一动作组中的独立执行器并行运动，动作组之间保持必要的安全顺序。
 * 每次update只下发命令或轮询状态，不使用delay和阻塞wait。
 */
class MechanismTaskExecutor : public IStationTaskExecutor
{
public:
    MechanismTaskExecutor(
        HardwareSerial &stepperSerial,
        HardwareSerial &baseSerial,
        HardwareSerial &servoSerial,
        IGraspVisionProvider &graspVision);

    /**
     * @brief 初始化总线并让机构到达上电初始位置。
     *
     * 初始化动作由update异步完成；ready()为true后才允许开始比赛。
     * 按下启动键后，机构会在底盘行驶期间转到运输收纳位置。
     */
    void begin();

    bool ready() const override;
    const char *faultMessage() const override;
    bool prepareForTravel() override;

    bool start(
        StationTask task,
        uint8_t round,
        const BatchMission &batch) override;
    void update() override;
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

    TTL_Protocol _stepperProtocol;
    TTL_Protocol _baseProtocol;
    TTL_Stepper _lift;
    TTL_Stepper _extension;
    TTL_Stepper _base;

    FSUS_Protocol _servoProtocol;
    FSUS_Servo _storageServo;
    FSUS_Servo _gripperServo;

    ActionStep _steps[MAX_ACTION_STEPS] = {};
    uint8_t _stepCount = 0;
    uint8_t _stepIndex = 0;
    uint8_t _nextStepGroup = 0;
    TTL_Stepper *_sharedPollOwner = nullptr;
    uint32_t _lastSharedBusPollMs = 0;
    uint32_t _alignmentStartedMs = 0;
    uint32_t _lastObservationMs = 0;
    uint32_t _lastAlignmentCommandMs = 0;
    uint8_t _stableFrames = 0;
    float _alignmentBaseTarget = 0.0F;
    float _alignmentExtensionTarget = 0.0F;

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
    void addSafeRetraction(float baseTarget);
    void addStorageDeposit();
    void loadInitializationAction();
    void loadHomeAction();
    void loadTurntablePreparationAction(uint8_t traySlot);
    void loadTurntablePickupToStorageAction();
    void loadStorageToRingAction(
        uint8_t traySlot,
        uint8_t ring,
        uint8_t stackLevel);
    void loadRingToStorageAction(uint8_t traySlot, uint8_t ring);

    bool updateCurrentStep();
    bool updateActionStep(ActionStep &step);
    void startPickupAlignment();
    void updatePickupAlignment();
    bool updateStepperStep(
        TTL_Stepper &motor,
        ActionStep &step,
        bool angle);
    void pollStepperState(
        TTL_Stepper &motor,
        ActionStep &step,
        uint32_t now);
    void issueServoStep(ActionStep &step);
    bool updateServoStep(ActionStep &step);
    bool queryServoAngle(FSUS_Servo &servo, float &angle);
    void onActionCompleted();
    void finishStationTask();
    const char *stepperFaultMessage(
        const TTL_Stepper &motor,
        bool protection) const;
    void fail(const char *message);

    static void resetStepperState(TTL_Stepper &motor);
    static bool validRing(uint8_t ring);
    static float clampValue(float value, float minimum, float maximum);
};
