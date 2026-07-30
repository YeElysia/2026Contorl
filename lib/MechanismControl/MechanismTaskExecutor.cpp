#include "MechanismTaskExecutor.h"

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
    resetStepperState(_lift);
    resetStepperState(_extension);
    resetStepperState(_base);

    /*
     * ping用于尽早发现舵机总线接错。仅在setup阶段执行一次，
     * 比赛循环中不使用供应商库的阻塞wait()。
     */
    if (!_storageServo.ping() || !_gripperServo.ping())
    {
        fail("mechanism servo offline");
        return;
    }

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
    _nextStepGroup = 0;
    _sharedPollOwner = nullptr;
    _lastSharedBusPollMs = 0;
}

void MechanismTaskExecutor::addStep(
    StepKind kind,
    float target)
{
    appendStep(kind, target, _nextStepGroup++);
}

void MechanismTaskExecutor::addConcurrentStep(
    StepKind kind,
    float target)
{
    if (_stepCount == 0)
    {
        addStep(kind, target);
        return;
    }

    appendStep(
        kind,
        target,
        _steps[_stepCount - 1].group);
}

void MechanismTaskExecutor::appendStep(
    StepKind kind,
    float target,
    uint8_t group)
{
    if (_stepCount >= MAX_ACTION_STEPS)
    {
        fail("mechanism action table overflow");
        return;
    }

    _steps[_stepCount++] = {
        kind,
        target,
        group,
        false,
        false,
        0,
        0,
        0,
        0.0F,
        false};
}

void MechanismTaskExecutor::addSafeRetraction(float baseTarget)
{
    /*
     * 用户已确认升降上抬和底座旋转不会互相干涉。保持升降命令
     * 最先下发，随后让底座转向和伸缩回收在同一动作组中并行。
     */
    addStep(StepKind::Lift, LIFT_HOME);
    addConcurrentStep(StepKind::RotateBase, baseTarget);
    addConcurrentStep(StepKind::Extend, EXTENSION_HOME);
}

void MechanismTaskExecutor::addStorageDeposit()
{
    // 底座已经进入车内方向后，按固定安全顺序将物料放回载物盘。
    addStep(StepKind::Extend, EXTENSION_STORAGE);
    addStep(StepKind::Lift, LIFT_STORAGE);
    addStep(StepKind::OpenGripper, GRIPPER_OPEN_ANGLE);
    addStep(StepKind::Lift, LIFT_HOME);
}

void MechanismTaskExecutor::loadHomeAction()
{
    clearAction();

    addSafeRetraction(BASE_HOME);
    addConcurrentStep(StepKind::RotateStorage, STORAGE_ANGLE[0]);
    addConcurrentStep(StepKind::CloseGripper, GRIPPER_CLOSE_ANGLE);
}

void MechanismTaskExecutor::loadInitializationAction()
{
    clearAction();
    addStep(StepKind::Lift, LIFT_INITIAL);
    addConcurrentStep(StepKind::RotateBase, BASE_INITIAL);
    addConcurrentStep(StepKind::Extend, EXTENSION_INITIAL);
    addConcurrentStep(StepKind::RotateStorage, STORAGE_ANGLE[0]);
    addConcurrentStep(StepKind::CloseGripper, GRIPPER_CLOSE_ANGLE);
}

void MechanismTaskExecutor::loadTurntablePreparationAction(uint8_t traySlot)
{
    clearAction();

    /*
     * 先发送升降上升命令，再发送底座旋转命令；载物盘和夹爪
     * 随后启动，四个执行器仍属于同一个并行动作组。
     */
    addStep(StepKind::Lift, LIFT_HOME);
    addConcurrentStep(StepKind::RotateBase, BASE_TURNTABLE);
    addConcurrentStep(StepKind::RotateStorage, STORAGE_ANGLE[traySlot]);
    addConcurrentStep(StepKind::OpenGripperMax, GRIPPER_OPEN_MAX_ANGLE);

    // 底座进入转盘方向后再伸出，避免长臂扫过车体结构。
    addStep(StepKind::Extend, EXTENSION_TURNTABLE);
}

void MechanismTaskExecutor::loadTurntablePickupToStorageAction()
{
    clearAction();
    addStep(StepKind::Lift, LIFT_TURNTABLE);
    addStep(StepKind::CloseGripper, GRIPPER_CLOSE_ANGLE);

    addSafeRetraction(BASE_STORAGE);
    addStorageDeposit();
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
    // 车内取料准备互不干涉，载物盘、底座、伸缩轴和夹爪同时动作。
    addStep(StepKind::RotateStorage, STORAGE_ANGLE[traySlot]);
    addConcurrentStep(StepKind::RotateBase, BASE_STORAGE);
    addConcurrentStep(StepKind::Extend, EXTENSION_STORAGE);
    addConcurrentStep(StepKind::OpenGripper, GRIPPER_OPEN_ANGLE);
    addStep(StepKind::Lift, LIFT_STORAGE);
    addStep(StepKind::CloseGripper, GRIPPER_CLOSE_ANGLE);

    addSafeRetraction(RING_BASE_ANGLE[ring]);

    // 底座到达圆环方向后才允许伸出长臂。
    addStep(StepKind::Extend, RING_EXTENSION[ring]);
    addStep(
        StepKind::Lift,
        LIFT_GROUND - stackLevel * MATERIAL_HEIGHT);
    addStep(StepKind::OpenGripper, GRIPPER_OPEN_ANGLE);

    addSafeRetraction(BASE_STORAGE);
}

void MechanismTaskExecutor::loadRingToStorageAction(
    uint8_t traySlot,
    uint8_t ring)
{
    clearAction();
    addStep(StepKind::RotateStorage, STORAGE_ANGLE[traySlot]);
    addConcurrentStep(StepKind::RotateBase, RING_BASE_ANGLE[ring]);
    addConcurrentStep(StepKind::OpenGripperMax, GRIPPER_OPEN_MAX_ANGLE);

    // 底座到达圆环方向后再伸出。
    addStep(StepKind::Extend, RING_EXTENSION[ring]);
    addStep(StepKind::Lift, LIFT_GROUND);
    addStep(StepKind::CloseGripper, GRIPPER_CLOSE_ANGLE);

    addSafeRetraction(BASE_STORAGE);
    addStorageDeposit();
}

bool MechanismTaskExecutor::updateCurrentStep()
{
    if (_result != AsyncResult::Running)
        return false;
    if (_stepIndex >= _stepCount)
        return true;

    const uint8_t activeGroup = _steps[_stepIndex].group;
    bool groupCompleted = true;

    for (uint8_t i = _stepIndex;
         i < _stepCount && _steps[i].group == activeGroup;
         ++i)
    {
        ActionStep &step = _steps[i];
        if (!step.completed)
            step.completed = updateActionStep(step);
        if (!step.completed)
            groupCompleted = false;
    }

    if (!groupCompleted || _result == AsyncResult::Failed)
        return false;

    do
    {
        ++_stepIndex;
    } while (
        _stepIndex < _stepCount &&
        _steps[_stepIndex].group == activeGroup);

    return _stepIndex >= _stepCount;
}

bool MechanismTaskExecutor::updateActionStep(ActionStep &step)
{
    switch (step.kind)
    {
    case StepKind::Lift:
        return updateStepperStep(_lift, step, false);

    case StepKind::Extend:
        return updateStepperStep(_extension, step, false);

    case StepKind::RotateBase:
        return updateStepperStep(_base, step, true);

    case StepKind::RotateStorage:
    case StepKind::OpenGripper:
    case StepKind::OpenGripperMax:
    case StepKind::CloseGripper:
        return updateServoStep(step);
    }

    return false;
}

bool MechanismTaskExecutor::updateStepperStep(
    TTL_Stepper &motor,
    ActionStep &step,
    bool angle)
{
    const uint32_t now = millis();
    if (!step.issued)
    {
        resetStepperState(motor);
        if (angle)
            motor.setAngle(step.target);
        else
            motor.runToNewPosition(step.target);

        step.issued = true;
        step.startedMs = now;
        step.lastPollMs = 0;
        return false;
    }

    if (now - step.startedMs >= STEPPER_TIMEOUT_MS)
    {
        fail("mechanism stepper timeout");
        return false;
    }

    pollStepperState(motor, step, now);

    if (motor.locked_state)
    {
        fail(stepperFaultMessage(motor, false));
        return false;
    }

    if (motor.loPro_state)
    {
        fail(stepperFaultMessage(motor, true));
        return false;
    }
    return motor.onPos_state;
}

void MechanismTaskExecutor::pollStepperState(
    TTL_Stepper &motor,
    ActionStep &step,
    uint32_t now)
{
    /*
     * 升降和伸缩共用一条TTL串口。供应商库的state_update()在发起
     * 查询时会清空串口缓存，因此必须让一次查询完整收发结束后，
     * 才能查询同总线的另一台电机。运动命令已经同时下发，这里的
     * 轮流操作只影响状态查询频率，不影响两个电机并行运动。
     */
    const bool sharedBusMotor =
        &motor == &_lift || &motor == &_extension;

    if (sharedBusMotor)
    {
        if (_sharedPollOwner != nullptr &&
            _sharedPollOwner != &motor)
        {
            return;
        }

        if (now - _lastSharedBusPollMs < STEPPER_POLL_MS)
            return;

        if (_sharedPollOwner == nullptr)
            _sharedPollOwner = &motor;

        _lastSharedBusPollMs = now;
        motor.state_update();

        // Ask_State清零表示该电机的应答已经完整解析。
        if (!motor.Ask_State)
            _sharedPollOwner = nullptr;
        return;
    }

    if (step.lastPollMs != 0 &&
        now - step.lastPollMs < STEPPER_POLL_MS)
        return;

    step.lastPollMs = now;
    motor.state_update();
}

void MechanismTaskExecutor::issueServoStep(ActionStep &step)
{
    step.issued = true;
    step.startedMs = millis();
    step.lastPollMs = 0;
    step.stableFeedbackCount = 0;
    step.previousFeedbackAngle = 0.0F;
    step.hasPreviousFeedback = false;

    switch (step.kind)
    {
    case StepKind::RotateStorage:
        _storageServo.setRawAngleByVelocity(
            _storageServo.angleReal2Raw(step.target),
            STORAGE_SPEED_DPS,
            0,
            0,
            0);
        break;
    case StepKind::OpenGripper:
    case StepKind::OpenGripperMax:
        _gripperServo.setRawAngleByVelocity(
            _gripperServo.angleReal2Raw(step.target),
            GRIPPER_SPEED_DPS,
            0,
            0,
            0);
        break;
    case StepKind::CloseGripper:
        _gripperServo.setRawAngleByVelocity(
            _gripperServo.angleReal2Raw(step.target),
            GRIPPER_SPEED_DPS,
            0,
            0,
            GRIPPER_MAX_POWER);
        break;
    default:
        break;
    }
}

bool MechanismTaskExecutor::updateServoStep(ActionStep &step)
{
    if (!step.issued)
    {
        issueServoStep(step);
        return false;
    }

    const uint32_t now = millis();
    if (now - step.startedMs >= SERVO_TIMEOUT_MS)
    {
        fail(step.kind == StepKind::RotateStorage
                 ? "storage servo feedback timeout"
                 : "gripper servo feedback timeout");
        return false;
    }

    if (step.lastPollMs != 0 &&
        now - step.lastPollMs < SERVO_POLL_MS)
    {
        return false;
    }
    step.lastPollMs = now;

    FSUS_Servo &servo =
        step.kind == StepKind::RotateStorage
            ? _storageServo
            : _gripperServo;
    float actualAngle = 0.0F;
    if (!queryServoAngle(servo, actualAngle))
    {
        step.stableFeedbackCount = 0;
        return false;
    }

    const float tolerance =
        step.kind == StepKind::RotateStorage
            ? STORAGE_POSITION_TOLERANCE_DEG
            : GRIPPER_POSITION_TOLERANCE_DEG;
    bool reached = abs(actualAngle - step.target) <= tolerance;
    const bool angleStopped =
        step.hasPreviousFeedback &&
        abs(actualAngle - step.previousFeedbackAngle) <=
            GRIPPER_STALL_ANGLE_DELTA_DEG;
    step.previousFeedbackAngle = actualAngle;
    step.hasPreviousFeedback = true;

    if (step.kind == StepKind::CloseGripper && !reached)
    {
        /*
         * 夹住物料后，夹爪本来就不应继续到空载闭合角度。
         * 实际角度确认夹爪已经离开张开端，再用功率判断是否形成
         * 有效夹持；通信失败返回0，不会被误判成夹紧成功。
         */
        const uint16_t power = _gripperServo.queryPower();
        const bool powerValid =
            _servoProtocol.responsePack.recv_status ==
            FSUS_STATUS_SUCCESS;
        reached =
            powerValid &&
            actualAngle >= GRIPPER_LOAD_MIN_ANGLE &&
            angleStopped &&
            power >= GRIPPER_LOAD_POWER_MW;
    }

    if (reached)
    {
        if (step.stableFeedbackCount < SERVO_STABLE_FEEDBACK_COUNT)
            ++step.stableFeedbackCount;
    }
    else
    {
        step.stableFeedbackCount = 0;
    }

    return step.stableFeedbackCount >=
           SERVO_STABLE_FEEDBACK_COUNT;
}

bool MechanismTaskExecutor::queryServoAngle(
    FSUS_Servo &servo,
    float &angle)
{
    angle = servo.queryAngle();
    const uint8_t status = _servoProtocol.responsePack.recv_status;

    /*
     * 供应商协议解析器在校验和异常时仍会解析完整角度帧，
     * 因此允许该状态参与后续的连续两帧确认。其他错误一律重试。
     */
    return status == FSUS_STATUS_SUCCESS ||
           status == FSUS_STATUS_CHECKSUM_ERROR;
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

const char *MechanismTaskExecutor::stepperFaultMessage(
    const TTL_Stepper &motor,
    bool protection) const
{
    if (&motor == &_lift)
        return protection
                   ? "lift stepper protection"
                   : "lift stepper locked";

    if (&motor == &_extension)
        return protection
                   ? "extension stepper protection"
                   : "extension stepper locked";

    return protection
               ? "base stepper protection"
               : "base stepper locked";
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

void MechanismTaskExecutor::resetStepperState(TTL_Stepper &motor)
{
    motor.recDate_Clear();
    motor.onPos_state = false;
    motor.locked_state = false;
    motor.loPro_state = false;
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
