#include "MechanismTaskExecutor.h"

#include <string.h>

#include "mechanism_config.h"
#include "vision_config.h"

using namespace mechanism_config;

MechanismTaskExecutor::MechanismTaskExecutor(
    HardwareSerial &stepperSerial,
    HardwareSerial &baseSerial,
    HardwareSerial &servoSerial,
    IGraspVisionProvider &graspVision)
    : _stepperSerial(stepperSerial),
      _baseSerial(baseSerial),
      _servoSerial(servoSerial),
      _graspVision(graspVision),
      _stepperProtocol(&stepperSerial, BUS_BAUD),
      _baseProtocol(&baseSerial, BUS_BAUD),
      _lift(LIFT_STEPPER_ID, &_stepperProtocol),
      _extension(EXTENSION_STEPPER_ID, &_stepperProtocol),
      _base(BASE_STEPPER_ID, &_baseProtocol),
      _storageServo(STORAGE_SERVO_ID, &_servoProtocol),
      _gripperServo(GRIPPER_SERVO_ID, &_servoProtocol)
{
}

void MechanismTaskExecutor::begin()
{
    _graspVision.begin();
    _stepperProtocol.init(&_stepperSerial, BUS_BAUD);
    _baseProtocol.init(&_baseSerial, BUS_BAUD);
    _servoProtocol.init(&_servoSerial, BUS_BAUD);

    _lift.init(LIFT_STEPPER_ID, &_stepperProtocol);
    _lift.set(
        LIFT_SPEED,
        LIFT_ACCELERATION,
        LIFT_CW,
        LIFT_CONVERT_K,
        LIFT_SUBSTEP);
    _extension.init(EXTENSION_STEPPER_ID, &_stepperProtocol);
    _extension.set(
        EXTENSION_SPEED,
        EXTENSION_ACCELERATION,
        EXTENSION_CW,
        EXTENSION_CONVERT_K,
        EXTENSION_SUBSTEP);
    _base.init(BASE_STEPPER_ID, &_baseProtocol);
    _base.set(
        BASE_SPEED,
        BASE_ACCELERATION,
        BASE_CW,
        BASE_CONVERT_K,
        BASE_SUBSTEP);

    // 显式初始化查询状态，避免供应商库默认构造值不确定。
    _lift.recDate_Clear();
    _extension.recDate_Clear();
    _base.recDate_Clear();
    _lift.onPos_state = false;
    _extension.onPos_state = false;
    _base.onPos_state = false;
    _lift.locked_state = false;
    _extension.locked_state = false;
    _base.locked_state = false;

    /*
     * ping用于尽早发现舵机总线接错。仅在setup阶段执行一次，
     * 比赛循环中不使用供应商库的阻塞wait()。
     */
    if (!_storageServo.ping() || !_gripperServo.ping())
    {
        fail("mechanism servo offline");
        return;
    }

    _storageServo.setSpeed(400);
    _initialized = false;
    _result = AsyncResult::Running;
    _phase = TaskPhase::Initializing;
    _itemIndex = 0;
    _fault = "";
    loadInitializationAction();
}

bool MechanismTaskExecutor::ready() const
{
    return _initialized && _result != AsyncResult::Failed;
}

const char *MechanismTaskExecutor::faultMessage() const
{
    return _fault;
}

bool MechanismTaskExecutor::prepareForTravel()
{
    if (!ready() || _result == AsyncResult::Running)
        return false;

    _result = AsyncResult::Running;
    _phase = TaskPhase::PreparingForTravel;
    _fault = "";
    loadHomeAction();
    return true;
}

bool MechanismTaskExecutor::start(
    StationTask task,
    uint8_t round,
    const BatchMission &batch)
{
    if (!ready() || _result == AsyncResult::Running || round > 1)
        return false;

    for (uint8_t i = 0; i < MATERIALS_PER_BATCH; ++i)
    {
        if (!validRing(batch.roughPositions[i]) ||
            !validRing(batch.storagePositions[i]))
        {
            return false;
        }
    }

    _round = round;
    _batch = batch;
    _itemIndex = 0;
    _result = AsyncResult::Running;
    _fault = "";

    switch (task)
    {
    case StationTask::CollectMaterial:
        _phase = TaskPhase::CollectPreparing;
        loadTurntablePreparationAction(1);
        break;

    case StationTask::RoughProcessing:
        _phase = TaskPhase::RoughPlacing;
        loadStorageToRingAction(
            1,
            _batch.roughPositions[0],
            0);
        break;

    case StationTask::StoreFinishedProduct:
        _phase = TaskPhase::FinalStoring;
        loadStorageToRingAction(
            1,
            _batch.storagePositions[0],
            _round);
        break;
    }

    return true;
}

void MechanismTaskExecutor::update()
{
    _graspVision.update();

    if (_result != AsyncResult::Running)
        return;

    if (_phase == TaskPhase::CollectAligning)
    {
        updatePickupAlignment();
        return;
    }

    if (updateCurrentStep())
        onActionCompleted();
}

AsyncResult MechanismTaskExecutor::result() const
{
    return _result;
}

void MechanismTaskExecutor::cancel()
{
    _graspVision.stop();

    if (_result == AsyncResult::Running)
    {
        _stepperProtocol.Emm_V5_Stop_Now(LIFT_STEPPER_ID, false);
        _stepperProtocol.Emm_V5_Stop_Now(EXTENSION_STEPPER_ID, false);
        _baseProtocol.Emm_V5_Stop_Now(BASE_STEPPER_ID, false);
    }

    clearAction();
    _phase = TaskPhase::Idle;
    _result = AsyncResult::Idle;
}

void MechanismTaskExecutor::clearAction()
{
    _stepCount = 0;
    _stepIndex = 0;
    _stepIssued = false;
    _stepStartedMs = 0;
    _lastPollMs = 0;
}

void MechanismTaskExecutor::addStep(
    StepKind kind,
    float target,
    uint16_t waitMs)
{
    if (_stepCount >= MAX_ACTION_STEPS)
    {
        fail("mechanism action table overflow");
        return;
    }
    _steps[_stepCount++] = {kind, target, waitMs};
}

void MechanismTaskExecutor::loadHomeAction()
{
    clearAction();

    /*
     * 载物盘舵机收到命令后会自行转动，因此先发送其目标位置，
     * 后续升降、伸缩和底座步进动作可与载物盘同时进行。最后的
     * WaitStorage只在其他轴先完成时补足剩余舵机运动时间。
     */
    addStep(StepKind::RotateStorage, STORAGE_ANGLE[0], STORAGE_MOVE_MS);
    addStep(StepKind::Lift, LIFT_HOME);
    addStep(StepKind::Extend, EXTENSION_HOME);
    addStep(StepKind::CloseGripper, GRIPPER_CLOSE_ANGLE, GRIPPER_MOVE_MS);
    addStep(StepKind::RotateBase, BASE_HOME);
    addStep(StepKind::WaitStorage, 0.0F);
}

void MechanismTaskExecutor::loadInitializationAction()
{
    clearAction();
    addStep(StepKind::RotateStorage, STORAGE_ANGLE[0], STORAGE_MOVE_MS);
    addStep(StepKind::Lift, LIFT_INITIAL);
    addStep(StepKind::Extend, EXTENSION_INITIAL);
    addStep(StepKind::CloseGripper, GRIPPER_CLOSE_ANGLE, GRIPPER_MOVE_MS);
    addStep(StepKind::RotateBase, BASE_INITIAL);
    addStep(StepKind::WaitStorage, 0.0F);
}

void MechanismTaskExecutor::loadTurntablePreparationAction(uint8_t traySlot)
{
    clearAction();

    /*
     * 先启动载物盘，再转动机械臂底座。载物盘舵机与三个步进轴
     * 相互独立，底座旋转期间载物盘会同步到达本次使用的槽位。
     */
    addStep(StepKind::RotateStorage, STORAGE_ANGLE[traySlot], STORAGE_MOVE_MS);
    addStep(StepKind::Lift, LIFT_HOME);
    addStep(StepKind::RotateBase, BASE_TURNTABLE);
    addStep(StepKind::OpenGripperMax, GRIPPER_OPEN_MAX_ANGLE, GRIPPER_MOVE_MS);
    addStep(StepKind::Extend, EXTENSION_TURNTABLE);
    addStep(StepKind::WaitStorage, 0.0F);
}

void MechanismTaskExecutor::loadTurntablePickupToStorageAction()
{
    clearAction();
    addStep(StepKind::Lift, LIFT_TURNTABLE);
    addStep(StepKind::CloseGripper, GRIPPER_CLOSE_ANGLE, GRIPPER_MOVE_MS);
    addStep(StepKind::Lift, LIFT_HOME);
    addStep(StepKind::Extend, EXTENSION_HOME);
    addStep(StepKind::RotateBase, BASE_STORAGE);
    addStep(StepKind::Extend, EXTENSION_STORAGE);
    addStep(StepKind::WaitStorage, 0.0F);
    addStep(StepKind::Lift, LIFT_STORAGE);
    addStep(StepKind::OpenGripper, GRIPPER_OPEN_ANGLE, GRIPPER_MOVE_MS);
    addStep(StepKind::Lift, LIFT_HOME);
}

void MechanismTaskExecutor::startPickupAlignment()
{
    clearAction();
    _stableFrames = 0;
    _alignmentStartedMs = millis();
    _lastObservationMs = 0;
    _lastAlignmentCommandMs = 0;
    _alignmentBaseTarget = BASE_TURNTABLE;
    _alignmentExtensionTarget = EXTENSION_TURNTABLE;

    if (!_graspVision.startTracking(_batch.colors[_itemIndex]))
    {
        fail("failed to start grasp vision");
        return;
    }

    _phase = TaskPhase::CollectAligning;
}

void MechanismTaskExecutor::updatePickupAlignment()
{
    using namespace vision_config;

    const uint32_t now = millis();
    if (_graspVision.faulted())
    {
        fail("MaixPro rejected grasp target");
        return;
    }

    if (now - _alignmentStartedMs >= TARGET_SEARCH_TIMEOUT_MS)
    {
        fail("grasp target search timeout");
        return;
    }

    GraspObservation observation;
    if (!_graspVision.takeObservation(observation))
    {
        if (_lastObservationMs != 0 &&
            now - _lastObservationMs > TARGET_STALE_MS)
        {
            _stableFrames = 0;
        }
        return;
    }

    _lastObservationMs = observation.receivedMs;
    if (!observation.found || observation.quality < MIN_QUALITY)
    {
        _stableFrames = 0;
        return;
    }

    const int16_t errorDx = observation.dx - TARGET_DX_PX;
    const int16_t errorDy = observation.dy - TARGET_DY_PX;
    const bool centered =
        abs(errorDx) <= CENTER_TOLERANCE_PX &&
        abs(errorDy) <= CENTER_TOLERANCE_PX;

    if (centered)
    {
        if (_stableFrames < REQUIRED_STABLE_FRAMES)
            ++_stableFrames;

        if (_stableFrames >= REQUIRED_STABLE_FRAMES)
        {
            _graspVision.stop();
            _phase = TaskPhase::CollectDepositing;
            loadTurntablePickupToStorageAction();
        }
        return;
    }

    _stableFrames = 0;
    if (now - _lastAlignmentCommandMs < ALIGN_INTERVAL_MS)
        return;
    _lastAlignmentCommandMs = now;

    float baseDelta = -errorDx * BASE_UNITS_PER_PIXEL;
    float extensionDelta = errorDy * EXTENSION_UNITS_PER_PIXEL;
    baseDelta = clampValue(
        baseDelta,
        -BASE_MAX_DELTA,
        BASE_MAX_DELTA);
    extensionDelta = clampValue(
        extensionDelta,
        -EXTENSION_MAX_DELTA,
        EXTENSION_MAX_DELTA);

    if (abs(errorDx) > CENTER_TOLERANCE_PX)
    {
        _alignmentBaseTarget = clampValue(
            _alignmentBaseTarget + baseDelta,
            PICKUP_BASE_MIN,
            PICKUP_BASE_MAX);
        _base.setAngle(
            _alignmentBaseTarget,
            BASE_SPEED,
            10);
    }

    if (abs(errorDy) > CENTER_TOLERANCE_PX)
    {
        _alignmentExtensionTarget = clampValue(
            _alignmentExtensionTarget + extensionDelta,
            PICKUP_EXTENSION_MIN,
            PICKUP_EXTENSION_MAX);
        _extension.runToNewPosition(
            _alignmentExtensionTarget,
            EXTENSION_SPEED,
            10);
    }
}

void MechanismTaskExecutor::loadStorageToRingAction(
    uint8_t traySlot,
    uint8_t ring,
    uint8_t stackLevel)
{
    clearAction();
    // 载物盘先异步旋转，同时准备底座、伸缩轴和夹爪。
    addStep(StepKind::RotateStorage, STORAGE_ANGLE[traySlot], STORAGE_MOVE_MS);
    addStep(StepKind::RotateBase, BASE_STORAGE);
    addStep(StepKind::Extend, EXTENSION_STORAGE);
    addStep(StepKind::OpenGripper, GRIPPER_OPEN_ANGLE, GRIPPER_MOVE_MS);
    addStep(StepKind::WaitStorage, 0.0F);
    addStep(StepKind::Lift, LIFT_STORAGE);
    addStep(StepKind::CloseGripper, GRIPPER_CLOSE_ANGLE, GRIPPER_MOVE_MS);
    addStep(StepKind::Lift, LIFT_HOME);
    addStep(StepKind::RotateBase, RING_BASE_ANGLE[ring]);
    addStep(StepKind::Extend, RING_EXTENSION[ring]);
    addStep(
        StepKind::Lift,
        LIFT_GROUND - stackLevel * MATERIAL_HEIGHT);
    addStep(StepKind::OpenGripper, GRIPPER_OPEN_ANGLE, GRIPPER_MOVE_MS);
    addStep(StepKind::Lift, LIFT_HOME);
    addStep(StepKind::Extend, EXTENSION_HOME);
    addStep(StepKind::RotateBase, BASE_STORAGE);
}

void MechanismTaskExecutor::loadRingToStorageAction(
    uint8_t traySlot,
    uint8_t ring)
{
    clearAction();
    addStep(StepKind::RotateStorage, STORAGE_ANGLE[traySlot], STORAGE_MOVE_MS);
    addStep(StepKind::RotateBase, RING_BASE_ANGLE[ring]);
    addStep(StepKind::OpenGripperMax, GRIPPER_OPEN_MAX_ANGLE, GRIPPER_MOVE_MS);
    addStep(StepKind::Extend, RING_EXTENSION[ring]);
    addStep(StepKind::Lift, LIFT_GROUND);
    addStep(StepKind::CloseGripper, GRIPPER_CLOSE_ANGLE, GRIPPER_MOVE_MS);
    addStep(StepKind::Lift, LIFT_HOME);
    addStep(StepKind::Extend, EXTENSION_HOME);
    addStep(StepKind::RotateBase, BASE_STORAGE);
    addStep(StepKind::Extend, EXTENSION_STORAGE);
    addStep(StepKind::WaitStorage, 0.0F);
    addStep(StepKind::Lift, LIFT_STORAGE);
    addStep(StepKind::OpenGripper, GRIPPER_OPEN_ANGLE, GRIPPER_MOVE_MS);
    addStep(StepKind::Lift, LIFT_HOME);
}

bool MechanismTaskExecutor::updateCurrentStep()
{
    if (_result != AsyncResult::Running)
        return false;
    if (_stepIndex >= _stepCount)
        return true;

    const ActionStep &step = _steps[_stepIndex];
    bool completed = false;

    switch (step.kind)
    {
    case StepKind::Lift:
        completed = updateStepperStep(_lift, step.target, false);
        break;
    case StepKind::Extend:
        completed = updateStepperStep(_extension, step.target, false);
        break;
    case StepKind::RotateBase:
        completed = updateStepperStep(_base, step.target, true);
        break;
    case StepKind::RotateStorage:
        if (!_stepIssued)
            issueServoStep(step);
        // 只负责发送目标角度，不在此处等待舵机到位。
        completed = true;
        break;
    case StepKind::WaitStorage:
        completed =
            static_cast<int32_t>(millis() - _storageReadyMs) >= 0;
        break;
    case StepKind::OpenGripper:
    case StepKind::OpenGripperMax:
    case StepKind::CloseGripper:
        if (!_stepIssued)
            issueServoStep(step);
        completed = millis() - _stepStartedMs >= step.waitMs;
        break;
    }

    if (!completed || _result == AsyncResult::Failed)
        return false;

    ++_stepIndex;
    _stepIssued = false;
    _stepStartedMs = 0;
    _lastPollMs = 0;
    return _stepIndex >= _stepCount;
}

bool MechanismTaskExecutor::updateStepperStep(
    TTL_Stepper &motor,
    float target,
    bool angle)
{
    const uint32_t now = millis();
    if (!_stepIssued)
    {
        motor.recDate_Clear();
        motor.onPos_state = false;
        motor.locked_state = false;
        if (angle)
            motor.setAngle(target);
        else
            motor.runToNewPosition(target);

        _stepIssued = true;
        _stepStartedMs = now;
        _lastPollMs = 0;
        return false;
    }

    if (now - _stepStartedMs >= STEPPER_TIMEOUT_MS)
    {
        fail("mechanism stepper timeout");
        return false;
    }

    if (_lastPollMs == 0 || now - _lastPollMs >= STEPPER_POLL_MS)
    {
        _lastPollMs = now;
        motor.state_update();
    }

    if (motor.locked_state || motor.loPro_state)
    {
        fail("mechanism stepper locked");
        return false;
    }
    return motor.onPos_state;
}

void MechanismTaskExecutor::issueServoStep(const ActionStep &step)
{
    _stepIssued = true;
    _stepStartedMs = millis();

    switch (step.kind)
    {
    case StepKind::RotateStorage:
        _storageServo.setAngle(step.target, step.waitMs);
        _storageReadyMs = _stepStartedMs + step.waitMs;
        break;
    case StepKind::OpenGripper:
    case StepKind::OpenGripperMax:
        _gripperServo.setAngle(step.target, step.waitMs, 0);
        break;
    case StepKind::CloseGripper:
        _gripperServo.setAngle(
            step.target,
            step.waitMs,
            GRIPPER_MAX_POWER);
        break;
    default:
        break;
    }
}

void MechanismTaskExecutor::onActionCompleted()
{
    switch (_phase)
    {
    case TaskPhase::Initializing:
        _initialized = true;
        _phase = TaskPhase::Idle;
        _result = AsyncResult::Succeeded;
        clearAction();
        break;

    case TaskPhase::PreparingForTravel:
        _phase = TaskPhase::Idle;
        _result = AsyncResult::Succeeded;
        clearAction();
        break;

    case TaskPhase::CollectPreparing:
        startPickupAlignment();
        break;

    case TaskPhase::CollectDepositing:
        if (++_itemIndex < MATERIALS_PER_BATCH)
        {
            _phase = TaskPhase::CollectPreparing;
            loadTurntablePreparationAction(_itemIndex + 1);
        }
        else
        {
            finishStationTask();
        }
        break;

    case TaskPhase::CollectAligning:
        fail("unexpected grasp alignment completion");
        break;

    case TaskPhase::RoughPlacing:
        if (++_itemIndex < MATERIALS_PER_BATCH)
        {
            loadStorageToRingAction(
                _itemIndex + 1,
                _batch.roughPositions[_itemIndex],
                0);
        }
        else
        {
            _itemIndex = 0;
            _phase = TaskPhase::RoughRetrieving;
            loadRingToStorageAction(
                1,
                _batch.roughPositions[0]);
        }
        break;

    case TaskPhase::RoughRetrieving:
        if (++_itemIndex < MATERIALS_PER_BATCH)
        {
            loadRingToStorageAction(
                _itemIndex + 1,
                _batch.roughPositions[_itemIndex]);
        }
        else
        {
            finishStationTask();
        }
        break;

    case TaskPhase::FinalStoring:
        if (++_itemIndex < MATERIALS_PER_BATCH)
        {
            loadStorageToRingAction(
                _itemIndex + 1,
                _batch.storagePositions[_itemIndex],
                _round);
        }
        else
        {
            finishStationTask();
        }
        break;

    case TaskPhase::Idle:
        fail("unexpected mechanism action completion");
        break;
    }
}

void MechanismTaskExecutor::finishStationTask()
{
    /*
     * 各动作表在结束前都已将物料放稳并把升降轴抬回安全高度。
     * 此时即可通知底盘启程；完整收纳由prepareForTravel()在行驶
     * 期间完成，避免停车等待底座和载物盘回位。
     */
    _phase = TaskPhase::Idle;
    _result = AsyncResult::Succeeded;
    clearAction();
}

void MechanismTaskExecutor::fail(const char *message)
{
    _graspVision.stop();
    // 故障发生在任务update内部时，主状态机尚未来得及调用cancel。
    // 在这里立即停车，避免超时或堵转后电机继续保持运动命令。
    _stepperProtocol.Emm_V5_Stop_Now(LIFT_STEPPER_ID, false);
    _stepperProtocol.Emm_V5_Stop_Now(EXTENSION_STEPPER_ID, false);
    _baseProtocol.Emm_V5_Stop_Now(BASE_STEPPER_ID, false);

    _fault = message;
    _phase = TaskPhase::Idle;
    _result = AsyncResult::Failed;
    clearAction();
}

bool MechanismTaskExecutor::validRing(uint8_t ring)
{
    return ring >= 1 && ring <= 3;
}

float MechanismTaskExecutor::clampValue(
    float value,
    float minimum,
    float maximum)
{
    if (value < minimum)
        return minimum;
    if (value > maximum)
        return maximum;
    return value;
}
