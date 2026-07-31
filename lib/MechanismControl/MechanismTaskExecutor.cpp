#include "MechanismTaskExecutor.h"

#include "mechanism_config.h"
#include "vision_config.h"

using namespace mechanism_config;

MechanismTaskExecutor::MechanismTaskExecutor(
    HardwareSerial &stepperSerial,
    HardwareSerial &baseSerial,
    HardwareSerial &servoSerial,
    IGraspVisionProvider &graspVision,
    IGraspForwardPositioner &forwardPositioner)
    : _stepperSerial(stepperSerial),
      _baseSerial(baseSerial),
      _servoSerial(servoSerial),
      _graspVision(graspVision),
      _forwardPositioner(forwardPositioner),
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

    /*
     * ping用于尽早发现舵机总线接错。仅在setup阶段执行一次，
     * 比赛循环中不使用供应商库的阻塞wait()。
     */
    if (!_storageServo.ping() || !_gripperServo.ping())
    {
        fail("mechanism servo offline");
        return;
    }
    // 与new_project一致：载物盘使用标准角度指令和库内速度换算。
    _storageServo.setSpeed(STORAGE_SPEED_DPS);

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

const char *MechanismTaskExecutor::debugPhase() const
{
    switch (_phase)
    {
    case TaskPhase::Initializing:
        return "INIT";
    case TaskPhase::PreparingForTravel:
        return "TRAVEL";
    case TaskPhase::CollectPreparing:
        return "COLLECT_PREP";
    case TaskPhase::CollectAligning:
        return "COLLECT_ALIGN";
    case TaskPhase::CollectDepositing:
        return "COLLECT_DEPOSIT";
    case TaskPhase::CollectReturningToRoute:
        return "COLLECT_RETURN";
    case TaskPhase::RoughPlacing:
        return "ROUGH_PLACE";
    case TaskPhase::RoughRetrieving:
        return "ROUGH_GET";
    case TaskPhase::FinalStoring:
        return "FINAL_STORE";
    case TaskPhase::Idle:
        return "IDLE";
    }
    return "UNKNOWN";
}

const MechanismTaskExecutor::GraspDebugState &
MechanismTaskExecutor::graspDebug() const
{
    return _graspDebug;
}

bool MechanismTaskExecutor::prepareForTravel(
    TravelDestination destination)
{
    if (_result == AsyncResult::Failed)
        return false;

    const bool interruptingInitialization =
        _result == AsyncResult::Running &&
        _phase == TaskPhase::Initializing;
    if (!interruptingInitialization &&
        (!ready() || _result == AsyncResult::Running))
    {
        return false;
    }

    /*
     * 初始化尚未完成时，loadTravelAction()会清除旧动作表并向各轴
     * 下发运输位目标。步进驱动支持重新设定绝对目标，无需停车等待。
     */
    _result = AsyncResult::Running;
    _phase = TaskPhase::PreparingForTravel;
    _fault = "";
    const float liftTarget =
        destination == TravelDestination::RoughProcessing ||
                destination == TravelDestination::Storage
            ? LIFT_STATION_VISION
            : LIFT_HOME;
    /*
     * 粗加工区和暂存区的第一个动作都从本批第一物料开始。行驶途中
     * 预先将第一物料槽转到机械臂取放位，避免到站后再等待载物盘。
     */
    const uint8_t traySlot =
        destination == TravelDestination::RoughProcessing ||
                destination == TravelDestination::Storage
            ? 1
            : 0;
    loadTravelAction(liftTarget, traySlot);
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
    _alignmentForwardOffset = 0.0F;
    _forwardCommandActive = false;
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
            ROUGH_RING_POSES[_batch.roughPositions[0]],
            0);
        break;

    case StationTask::StoreFinishedProduct:
        _phase = TaskPhase::FinalStoring;
        loadStorageToRingAction(
            1,
            FINAL_STORAGE_RING_POSES[
                _batch.storagePositions[0]],
            _round);
        break;
    }

    return true;
}

void MechanismTaskExecutor::update()
{
    if (_result != AsyncResult::Running)
        return;

    if (_phase == TaskPhase::CollectAligning)
    {
        _graspVision.update();
        updatePickupAlignment();
        return;
    }

    if (_phase == TaskPhase::CollectReturningToRoute)
    {
        updateReturnToMaterialRouteAnchor();
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
    if (_forwardCommandActive)
        _forwardPositioner.stop();

    if (_result == AsyncResult::Running)
    {
        _stepperProtocol.Emm_V5_Stop_Now(LIFT_STEPPER_ID, false);
        _stepperProtocol.Emm_V5_Stop_Now(EXTENSION_STEPPER_ID, false);
        _baseProtocol.Emm_V5_Stop_Now(BASE_STEPPER_ID, false);
    }

    clearAction();
    _forwardCommandActive = false;
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

    /*
     * 升降动作必须独占动作组。无论其他执行器是否存在机械干涉，
     * 都必须等待升降到位后再启动，避免共用电源和结构振动造成误判。
     * 即使动作表误用了并行接口，也自动拆到下一组并保持调用顺序。
     */
    const bool liftMustRunAlone =
        kind == StepKind::Lift ||
        lastGroupContains(StepKind::Lift);
    if (liftMustRunAlone)
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
        0};
}

bool MechanismTaskExecutor::lastGroupContains(StepKind kind) const
{
    if (_stepCount == 0)
        return false;

    const uint8_t lastGroup = _steps[_stepCount - 1].group;
    for (uint8_t i = 0; i < _stepCount; ++i)
    {
        if (_steps[i].group == lastGroup &&
            _steps[i].kind == kind)
        {
            return true;
        }
    }
    return false;
}

void MechanismTaskExecutor::addSafeRetraction(float baseTarget)
{
    /*
     * 先等待升降轴到达安全高度，再允许底座转向。
     * 底座旋转时可同时回收伸缩轴。
     */
    addStep(StepKind::Lift, LIFT_HOME);
    addStep(StepKind::RotateBase, baseTarget);
    addConcurrentStep(StepKind::Extend, EXTENSION_HOME);
}

void MechanismTaskExecutor::addStorageDeposit()
{
    // 底座已经进入车内方向后，按固定安全顺序将物料放回载物盘。
    addStep(StepKind::Extend, EXTENSION_TRAY_TRANSFER);
    addStep(StepKind::Lift, LIFT_TRAY_TRANSFER);
    addStep(StepKind::OpenGripper, GRIPPER_OPEN_ANGLE);
    addStep(StepKind::Lift, LIFT_HOME);
}

void MechanismTaskExecutor::loadTravelAction(
    float liftTarget,
    uint8_t traySlot)
{
    clearAction();

    const bool liftingToSafeHeight =
        liftTarget <= LIFT_HOME;

    /*
     * 升轴先于所有横向动作，先建立安全间隙；降轴则必须等底座、
     * 伸缩、载物盘和夹爪全部到位后再执行。粗加工视觉高度900
     * 属于降轴动作，因此会排在本动作表最后。
     */
    if (liftingToSafeHeight)
        addStep(StepKind::Lift, liftTarget);

    addStep(StepKind::RotateBase, BASE_HOME);
    addConcurrentStep(StepKind::Extend, EXTENSION_HOME);
    addConcurrentStep(
        StepKind::RotateStorage,
        TRAY_SLOT_ANGLE[traySlot]);
    // 运输收纳时保持夹爪张开，避免到粗加工区后遮挡圆环视野。
    addConcurrentStep(StepKind::OpenGripper, GRIPPER_OPEN_ANGLE);

    if (!liftingToSafeHeight)
        addStep(StepKind::Lift, liftTarget);
}

void MechanismTaskExecutor::loadInitializationAction()
{
    clearAction();
    addStep(StepKind::Lift, LIFT_INITIAL);
    addStep(StepKind::RotateBase, BASE_INITIAL);
    addConcurrentStep(StepKind::Extend, EXTENSION_INITIAL);
    addConcurrentStep(StepKind::RotateStorage, TRAY_SLOT_ANGLE[0]);
    // 初始化时夹爪内没有物料，只按角度确认空载闭合。
    addConcurrentStep(
        StepKind::CloseGripperUnloaded,
        GRIPPER_CLOSE_ANGLE);
}

void MechanismTaskExecutor::loadTurntablePreparationAction(uint8_t traySlot)
{
    clearAction();

    /*
     * 升降轴必须先独立到达安全高度。随后底座、载物盘和夹爪
     * 可以并行准备，全部到位后才允许伸出长臂。
     */
    addStep(StepKind::Lift, LIFT_HOME);
    addStep(StepKind::RotateBase, BASE_TURNTABLE);
    addConcurrentStep(
        StepKind::RotateStorage,
        TRAY_SLOT_ANGLE[traySlot]);
    addConcurrentStep(StepKind::OpenGripperMax, GRIPPER_OPEN_MAX_ANGLE);

    // 底座进入转盘方向后再伸出，避免长臂扫过车体结构。
    addStep(StepKind::Extend, EXTENSION_TURNTABLE);
}

void MechanismTaskExecutor::loadTurntablePickupToStorageAction()
{
    clearAction();
    addStep(StepKind::Lift, LIFT_TURNTABLE);
    addStep(StepKind::CloseGripper, GRIPPER_CLOSE_ANGLE);

    addSafeRetraction(BASE_TRAY_TRANSFER);
    addStorageDeposit();
}

void MechanismTaskExecutor::startPickupAlignment()
{
    clearAction();
    _stableFrames = 0;
    _alignmentStartedMs = millis();
    _lastObservationMs = 0;
    _alignmentObservationAfterMs = _alignmentStartedMs;
    _alignmentExtensionTarget = EXTENSION_TURNTABLE;
    _graspDebug = {};
    _graspDebug.item = _itemIndex + 1;
    _graspDebug.forwardOffsetMm = _alignmentForwardOffset;
    _graspDebug.extensionTarget = _alignmentExtensionTarget;
    _graspDebug.tracking = true;

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

    if (_forwardPositioner.faulted())
    {
        fail("material forward positioning failed");
        return;
    }

    if (now - _alignmentStartedMs >= TARGET_SEARCH_TIMEOUT_MS)
    {
        _graspDebug.tracking = false;
        fail("grasp target search timeout");
        return;
    }

    /*
     * 每次根据图像计算出底盘前后和伸缩目标后，先等待二者实际
     * 到位，再使用到位后的新图像继续精调。底盘接口不提供左右
     * 横移能力，朝转盘方向的误差只能由伸缩轴消除。
     */
    if (_stepCount > 0 || _forwardCommandActive)
    {
        const bool armCompleted =
            _stepCount == 0 || updateCurrentStep();
        const bool chassisCompleted =
            !_forwardCommandActive ||
            !_forwardPositioner.busy();
        if (!armCompleted || !chassisCompleted)
            return;

        clearAction();
        _forwardCommandActive = false;
        _alignmentObservationAfterMs = millis();
        _lastObservationMs = 0;
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

    // 丢弃电机运动期间产生的旧图像，只使用到位后的观测。
    if (static_cast<int32_t>(
            observation.receivedMs -
            _alignmentObservationAfterMs) < 0)
    {
        return;
    }

    _lastObservationMs = observation.receivedMs;
    _graspDebug.dx = observation.dx;
    _graspDebug.dy = observation.dy;
    _graspDebug.quality = observation.quality;
    _graspDebug.found = observation.found;
    _graspDebug.hasObservation = true;
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
            _graspDebug.tracking = false;
            _graspVision.stop();
            _phase = TaskPhase::CollectDepositing;
            loadTurntablePickupToStorageAction();
        }
        return;
    }

    _stableFrames = 0;

    float forwardDelta =
        FORWARD_FROM_DX * errorDx +
        FORWARD_FROM_DY * errorDy;
    float extensionDelta =
        EXTENSION_FROM_DX * errorDx +
        EXTENSION_FROM_DY * errorDy;

    const bool fineAlignment =
        abs(errorDx) <= FINE_ALIGNMENT_ZONE_PX &&
        abs(errorDy) <= FINE_ALIGNMENT_ZONE_PX;
    const float forwardLimit =
        fineAlignment
            ? FINE_FORWARD_MAX_DELTA_MM
            : COARSE_FORWARD_MAX_DELTA_MM;
    const float extensionLimit =
        fineAlignment
            ? FINE_EXTENSION_MAX_DELTA
            : COARSE_EXTENSION_MAX_DELTA;

    forwardDelta = clampValue(
        forwardDelta,
        -forwardLimit,
        forwardLimit);
    extensionDelta = clampValue(
        extensionDelta,
        -extensionLimit,
        extensionLimit);

    const float nextForwardOffset = clampValue(
        _alignmentForwardOffset + forwardDelta,
        PICKUP_FORWARD_MIN_OFFSET_MM,
        PICKUP_FORWARD_MAX_OFFSET_MM);
    const float nextExtensionTarget = clampValue(
        _alignmentExtensionTarget + extensionDelta,
        PICKUP_EXTENSION_MIN,
        PICKUP_EXTENSION_MAX);

    const float forwardMove =
        nextForwardOffset - _alignmentForwardOffset;
    if (fabsf(forwardMove) > 0.1F)
    {
        if (!_forwardPositioner.moveForward(forwardMove))
        {
            fail("material forward correction rejected");
            return;
        }
        _alignmentForwardOffset = nextForwardOffset;
        _graspDebug.forwardOffsetMm = _alignmentForwardOffset;
        _forwardCommandActive = true;
    }

    if (fabsf(
            nextExtensionTarget -
            _alignmentExtensionTarget) > 0.1F)
    {
        _alignmentExtensionTarget = nextExtensionTarget;
        _graspDebug.extensionTarget = _alignmentExtensionTarget;
        addStep(
            StepKind::Extend,
            _alignmentExtensionTarget);
    }

    if (!_forwardCommandActive && _stepCount == 0)
    {
        /*
         * 原料转盘仍在旋转。目标颜色位于视野边缘时，底盘和伸缩轴
         * 可能已经到达本工位软限位，此时不能继续追赶，也不应立即
         * 终止整场任务。保持当前位置读取后续新帧，等待目标随转盘
         * 进入可抓取范围；TARGET_SEARCH_TIMEOUT_MS仍负责最终兜底。
         */
        _stableFrames = 0;
        return;
    }
}

void MechanismTaskExecutor::startReturnToMaterialRouteAnchor()
{
    _graspVision.stop();
    clearAction();

    if (fabsf(_alignmentForwardOffset) <= 0.1F)
    {
        _alignmentForwardOffset = 0.0F;
        finishStationTask();
        return;
    }

    if (!_forwardPositioner.moveForward(
            -_alignmentForwardOffset))
    {
        fail("failed to return material route anchor");
        return;
    }

    _forwardCommandActive = true;
    _phase = TaskPhase::CollectReturningToRoute;
}

void MechanismTaskExecutor::updateReturnToMaterialRouteAnchor()
{
    if (_forwardPositioner.faulted())
    {
        fail("material route-anchor return failed");
        return;
    }

    if (_forwardPositioner.busy())
        return;

    _forwardCommandActive = false;
    _alignmentForwardOffset = 0.0F;
    finishStationTask();
}

void MechanismTaskExecutor::loadStorageToRingAction(
    uint8_t traySlot,
    const RingPose &pose,
    uint8_t stackLevel)
{
    clearAction();

    /*
     * 粗加工圆环识别结束时升降轴位于900。必须先升回安全高度，
     * 再允许底座和伸缩轴进入车内取料位置。
     */
    addStep(StepKind::Lift, LIFT_HOME);

    // 升轴完成后，车内取料的其他准备动作可以并行。
    addStep(
        StepKind::RotateStorage,
        TRAY_SLOT_ANGLE[traySlot]);
    addConcurrentStep(
        StepKind::RotateBase,
        BASE_TRAY_TRANSFER);
    addConcurrentStep(
        StepKind::Extend,
        EXTENSION_TRAY_TRANSFER);
    addConcurrentStep(StepKind::OpenGripper, GRIPPER_OPEN_ANGLE);
    addStep(StepKind::Lift, LIFT_TRAY_TRANSFER);
    addStep(StepKind::CloseGripper, GRIPPER_CLOSE_ANGLE);

    addSafeRetraction(pose.base);

    // 底座到达圆环方向后才允许伸出长臂。
    addStep(StepKind::Extend, pose.extension);
    addStep(
        StepKind::Lift,
        pose.lift - stackLevel * MATERIAL_HEIGHT);
    addStep(StepKind::OpenGripper, GRIPPER_OPEN_ANGLE);

    addSafeRetraction(BASE_TRAY_TRANSFER);
}

void MechanismTaskExecutor::loadRingToStorageAction(
    uint8_t traySlot,
    const RingPose &pose)
{
    clearAction();

    // 每次圆环取回都先确认升降轴处于安全高度，再移动其他轴。
    addStep(StepKind::Lift, LIFT_HOME);

    addStep(
        StepKind::RotateStorage,
        TRAY_SLOT_ANGLE[traySlot]);
    addConcurrentStep(StepKind::RotateBase, pose.base);
    addConcurrentStep(StepKind::OpenGripperMax, GRIPPER_OPEN_MAX_ANGLE);

    // 底座到达圆环方向后再伸出。
    addStep(StepKind::Extend, pose.extension);
    addStep(StepKind::Lift, pose.lift);
    addStep(StepKind::CloseGripper, GRIPPER_CLOSE_ANGLE);

    addSafeRetraction(BASE_TRAY_TRANSFER);
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
        if (_result == AsyncResult::Failed)
            return false;
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
    case StepKind::CloseGripperUnloaded:
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
        bool commandAccepted = false;
        if (angle)
            commandAccepted = motor.setAngle(step.target);
        else
            commandAccepted =
                motor.runToNewPosition(step.target);

        if (!commandAccepted)
        {
            fail(stepperCommandFaultMessage(motor));
            return false;
        }

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

    const bool freshFeedback =
        pollStepperState(motor, step, now);
    if (!freshFeedback)
        return false;

    /*
     * 驱动器可能在减速到位瞬间同时返回“到位”和短暂“堵转”。
     * 机械位置已经成立时应优先结束动作，不能把成功误报为故障。
     */
    if (motor.onPos_state)
    {
        step.faultFeedbackCount = 0;
        return true;
    }

    if (motor.locked_state || motor.loPro_state)
    {
        if (step.faultFeedbackCount < STEPPER_FAULT_CONFIRM_COUNT)
            ++step.faultFeedbackCount;

        if (step.faultFeedbackCount >= STEPPER_FAULT_CONFIRM_COUNT)
        {
            fail(stepperFaultMessage(
                motor,
                motor.loPro_state));
        }
        return false;
    }

    step.faultFeedbackCount = 0;
    return false;
}

bool MechanismTaskExecutor::pollStepperState(
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
            return false;
        }

        if (now - _lastSharedBusPollMs < STEPPER_POLL_MS)
            return false;

        if (_sharedPollOwner == nullptr)
            _sharedPollOwner = &motor;

        _lastSharedBusPollMs = now;
        const bool responsePending = motor.Ask_State;
        motor.state_update();

        // Ask_State清零表示该电机的应答已经完整解析。
        if (!motor.Ask_State)
            _sharedPollOwner = nullptr;
        return responsePending && !motor.Ask_State;
    }

    if (step.lastPollMs != 0 &&
        now - step.lastPollMs < STEPPER_POLL_MS)
        return false;

    step.lastPollMs = now;
    const bool responsePending = motor.Ask_State;
    motor.state_update();
    return responsePending && !motor.Ask_State;
}

void MechanismTaskExecutor::issueServoStep(ActionStep &step)
{
    step.issued = true;
    step.startedMs = millis();
    step.lastPollMs = 0;

    switch (step.kind)
    {
    case StepKind::RotateStorage:
        /*
         * 实车舵机固件不执行“按速度控制”扩展指令，但标准角度
         * 指令已经由new_project验证。setAngle只下发命令，不等待到位。
         */
        _storageServo.setAngle(step.target);
        break;
    case StepKind::OpenGripper:
    case StepKind::OpenGripperMax:
        _gripperServo.setAngle(
            step.target,
            GRIPPER_COMMAND_INTERVAL_MS,
            0);
        break;
    case StepKind::CloseGripperUnloaded:
    case StepKind::CloseGripper:
        _gripperServo.setAngle(
            step.target,
            GRIPPER_COMMAND_INTERVAL_MS,
            GRIPPER_MAX_POWER);
        break;
    default:
        break;
    }
}

bool MechanismTaskExecutor::updateServoStep(ActionStep &step)
{
    if (!step.issued)
        issueServoStep(step);

    // 不再读取角度或功率：命令写入串口后即视为舵机步骤完成。
    return true;
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
        // 直接从初始化切入运输动作时，也在收纳完成后建立ready状态。
        _initialized = true;
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
            startReturnToMaterialRouteAnchor();
        }
        break;

    case TaskPhase::CollectAligning:
        fail("unexpected grasp alignment completion");
        break;

    case TaskPhase::CollectReturningToRoute:
        fail("unexpected route-anchor action completion");
        break;

    case TaskPhase::RoughPlacing:
        if (++_itemIndex < MATERIALS_PER_BATCH)
        {
            loadStorageToRingAction(
                _itemIndex + 1,
                ROUGH_RING_POSES[
                    _batch.roughPositions[_itemIndex]],
                0);
        }
        else
        {
            _itemIndex = 0;
            _phase = TaskPhase::RoughRetrieving;
            loadRingToStorageAction(
                1,
                ROUGH_RING_POSES[
                    _batch.roughPositions[0]]);
        }
        break;

    case TaskPhase::RoughRetrieving:
        if (++_itemIndex < MATERIALS_PER_BATCH)
        {
            loadRingToStorageAction(
                _itemIndex + 1,
                ROUGH_RING_POSES[
                    _batch.roughPositions[_itemIndex]]);
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
                FINAL_STORAGE_RING_POSES[
                    _batch.storagePositions[_itemIndex]],
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

const char *MechanismTaskExecutor::stepperCommandFaultMessage(
    const TTL_Stepper &motor) const
{
    if (&motor == &_lift)
        return "lift stepper command failed";
    if (&motor == &_extension)
        return "extension stepper command failed";
    return "base stepper command failed";
}

void MechanismTaskExecutor::fail(const char *message)
{
    _graspVision.stop();
    if (_forwardCommandActive)
        _forwardPositioner.stop();
    // 故障发生在任务update内部时，主状态机尚未来得及调用cancel。
    // 在这里立即停车，避免超时或堵转后电机继续保持运动命令。
    _stepperProtocol.Emm_V5_Stop_Now(LIFT_STEPPER_ID, false);
    _stepperProtocol.Emm_V5_Stop_Now(EXTENSION_STEPPER_ID, false);
    _baseProtocol.Emm_V5_Stop_Now(BASE_STEPPER_ID, false);

    _fault = message;
    _forwardCommandActive = false;
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
