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
 * 升降轴默认独占动作组；已验证的储料盘与圆环转换会先竖直脱离
 * 物料，再让升降、底座和伸缩并行。下降到工作高度仍等待其他轴
 * 定位完成后执行。
 * 每次update只下发命令或轮询状态，不使用delay和阻塞wait。
 */
class MechanismTaskExecutor : public IStationTaskExecutor
{
public:
    struct GraspDebugState
    {
        int16_t dx = 0;
        int16_t dy = 0;
        uint8_t quality = 0;
        uint8_t item = 0;
        uint8_t attempt = 0;
        uint8_t collectedMask = 0;
        uint8_t missedMask = 0;
        float forwardOffsetMm = 0.0F;
        float rightOffsetMm = 0.0F;
        float extensionTarget = 0.0F;
        bool tracking = false;
        bool found = false;
        bool hasObservation = false;
    };

    struct MotionDebugState
    {
        float liftTarget = 0.0F;
        float baseTarget = 0.0F;
        float extensionTarget = 0.0F;
        uint32_t liftIssuedMs = 0;
        uint32_t baseIssuedMs = 0;
        uint32_t extensionIssuedMs = 0;
        uint32_t liftHomeConfirmedMs = 0;
        uint32_t roughPlacementIssuedMs = 0;
        uint32_t roughPlacementConfirmedMs = 0;
        uint32_t roughReturnIssuedMs = 0;
        uint32_t roughReturnConfirmedMs = 0;
        uint16_t liftCommandCount = 0;
        uint16_t baseCommandCount = 0;
        uint16_t extensionCommandCount = 0;
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
    void begin(uint8_t initialStorageSlot = 0);

    bool ready() const override;
    const char *faultMessage() const override;
    const char *debugPhase() const;
    const GraspDebugState &graspDebug() const;
    const MotionDebugState &motionDebug() const;
    bool prepareForTravel(
        TravelDestination destination) override;

    bool start(
        StationTask task,
        uint8_t round,
        const BatchMission &batch) override;
    bool startRoughPlacementCalibration(
        uint8_t ring,
        uint16_t speed,
        uint8_t acceleration);
    bool startRoughReturnCalibration(
        uint8_t ring,
        uint16_t speed,
        uint8_t acceleration);
    void update() override;
    AsyncResult result() const override;
    void cancel() override;

private:
    enum class StepKind : uint8_t
    {
        Lift,
        Extend,
        RotateBase,
        RotateBaseRoughCalibration,
        RotateBaseRoughReturn,
        RotateBaseRoughReturnCalibration,
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
        uint32_t queryStartedMs;
        uint8_t faultFeedbackCount;
        uint8_t onPositionFeedbackCount;
    };

    enum class TaskPhase : uint8_t
    {
        Initializing,
        PreparingForTravel,
        CollectPreparing,
        CollectAligning,
        CollectApproaching,
        CollectGrasping,
        CollectDepositing,
        CollectSkipping,
        RoughPlacing,
        RoughRetrieving,
        FinalStoring,
        CalibratingRoughPlacement,
        CalibratingRoughReturn,
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
    GraspDebugState _graspDebug;
    MotionDebugState _motionDebug;

    ActionStep _steps[MAX_ACTION_STEPS] = {};
    uint8_t _stepCount = 0;
    uint8_t _stepIndex = 0;
    uint8_t _nextStepGroup = 0;
    TTL_Stepper *_sharedPollOwner = nullptr;
    uint32_t _lastSharedBusPollMs = 0;
    uint32_t _alignmentStartedMs = 0;
    uint32_t _lastObservationMs = 0;
    uint32_t _alignmentObservationAfterMs = 0;
    uint32_t _pickupSearchWaypointMs = 0;
    uint8_t _stableFrames = 0;
    uint8_t _pickupSearchIndex = 0;
    uint8_t _pickupSearchPass = 0;
    int8_t _primedItemIndex = -1;
    int8_t _preparedStorageSlot = -1;
    float _alignmentForwardOffset = 0.0F;
    float _alignmentRightOffset = 0.0F;
    float _alignmentExtensionTarget = 0.0F;
    float _pickupReferenceForwardOffset = 0.0F;
    float _pickupReferenceRightOffset = 0.0F;
    float _pickupReferenceExtensionTarget = 0.0F;
    bool _forwardCommandActive = false;
    bool _pickupReferenceValid = false;
    bool _pickupTargetSeen = false;
    bool _storageToRoughFastProfile = false;
    bool _roughPlacementCalibrationActive = false;
    uint16_t _roughPlacementCalibrationSpeed = 0;
    uint8_t _roughPlacementCalibrationAcceleration = 0;
    bool _roughReturnCalibrationActive = false;
    uint16_t _roughReturnCalibrationSpeed = 0;
    uint8_t _roughReturnCalibrationAcceleration = 0;

    TaskPhase _phase = TaskPhase::Idle;
    BatchMission _batch = {};
    BatchMission _batchByRound[2] = {};
    uint8_t _round = 0;
    uint8_t _itemIndex = 0;
    uint8_t _initialStorageSlot = 0;
    uint8_t _collectedItemMask[2] = {0, 0};
    uint8_t _missedItemMask[2] = {0, 0};
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
    void addParallelTransfer(
        float baseTarget,
        float extensionTarget = mechanism_config::EXTENSION_HOME);
    void addLiftThenRingTransfer(
        float clearanceLiftTarget,
        float baseTarget,
        float liftTarget,
        float extensionTarget,
        int8_t storageSlotToPrepare = -1);
    void addRoughRingToTrayRetraction(uint8_t ring);
    void addStorageDeposit(bool liftAfterDeposit = true);
    void appendStoragePreparation(
        uint8_t traySlot,
        uint8_t group);
    bool storageSlotPrepared(uint8_t traySlot) const;
    int8_t nextAvailableTraySlot() const;
    void loadInitializationAction();
    void loadTravelAction(
        float liftTarget,
        uint8_t traySlot,
        bool useAlignmentOpenMax);
    void loadTurntablePreparationAction(
        uint8_t traySlot,
        bool liftAlreadyHome = false);
    void loadTurntableApproachAction();
    void loadTurntableGraspAction();
    void loadPickupToStorageAction(
        uint8_t traySlot,
        bool liftAfterDeposit);
    void loadSkippedPickupRecoveryAction();
    void loadStorageToRingAction(
        uint8_t traySlot,
        const mechanism_config::RingPose &pose,
        uint8_t stackLevel,
        bool retractAfterPlacement = true,
        bool pickupPoseAlreadyPrepared = false);
    void loadRingToStorageAction(
        uint8_t traySlot,
        const mechanism_config::RingPose &pose,
        bool directRingSwitch = false);

    bool updateCurrentStep();
    bool updateActionStep(ActionStep &step);
    bool issueStepperCommand(
        TTL_Stepper &motor,
        ActionStep &step,
        bool angle);
    bool primePickupVision(uint8_t itemIndex);
    void startPickupAlignment();
    void updatePickupAlignment();
    bool restorePickupReference();
    void advancePickupSearch();
    void updatePickupApproach();
    void startGripperClosing();
    void acceptPickup();
    void skipCurrentPickup();
    void advanceAfterPickup();
    bool selectFirstAvailableItem();
    bool selectNextAvailableItem();
    bool currentItemAvailable() const;
    bool hasAvailableItemAfterCurrent() const;
    uint8_t currentStorageStackLevel() const;
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
    void onActionCompleted();
    void finishStationTask();
    const char *stepperFaultMessage(
        const TTL_Stepper &motor,
        bool protection) const;
    const char *stepperCommandFaultMessage(
        const TTL_Stepper &motor) const;
    const char *stepperTimeoutMessage(
        const TTL_Stepper &motor) const;
    void fail(const char *message);

    static void resetStepperState(TTL_Stepper &motor);
    static bool validRing(uint8_t ring);
    static int8_t storageSlotFromAngle(float angle);
    static float clampValue(float value, float minimum, float maximum);
};
