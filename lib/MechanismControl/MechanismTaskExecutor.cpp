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

void MechanismTaskExecutor::begin(uint8_t initialStorageSlot)
{
    _initialStorageSlot =
        initialStorageSlot <= MATERIALS_PER_BATCH
            ? initialStorageSlot
            : 0;
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
     * 使用FSUS底层的完整初始化，同步当前角度并明确设置单圈模式。
     * 不使用FSGP_Gripper：其负载检测未实现，open/wait又会阻塞主循环。
     */
    _storageServo.init(STORAGE_SERVO_ID, &_servoProtocol);
    _gripperServo.init(GRIPPER_SERVO_ID, &_servoProtocol);
    if (!_storageServo.isOnline || !_gripperServo.isOnline)
    {
        fail("mechanism servo offline");
        return;
    }
    _gripperServo.setAngleRange(
        min(GRIPPER_OPEN_ANGLE, GRIPPER_OPEN_MAX_ANGLE),
        max(GRIPPER_CLOSE_ANGLE, GRIPPER_OPEN_MAX_ANGLE));
    // 与new_project一致：载物盘使用标准角度指令和库内速度换算。
    _storageServo.setSpeed(STORAGE_SPEED_DPS);

    _initialized = false;
    _result = AsyncResult::Running;
    _phase = TaskPhase::Initializing;
    _itemIndex = 0;
    _collectedItemMask[0] = 0;
    _collectedItemMask[1] = 0;
    _missedItemMask[0] = 0;
    _missedItemMask[1] = 0;
    _primedItemIndex = -1;
    _preparedStorageSlot = -1;
    _pickupReferenceValid = false;
    _pickupTargetSeen = false;
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
    case TaskPhase::CollectApproaching:
        return "COLLECT_DOWN";
    case TaskPhase::CollectGrasping:
        return "COLLECT_GRAB";
    case TaskPhase::CollectDepositing:
        return "COLLECT_DEPOSIT";
    case TaskPhase::CollectSkipping:
        return "COLLECT_SKIP";
    case TaskPhase::CollectReturningToRoute:
        return "COLLECT_RETURN";
    case TaskPhase::RoughPlacing:
        return "ROUGH_PLACE";
    case TaskPhase::RoughRetrieving:
        return "ROUGH_GET";
    case TaskPhase::FinalStoring:
        return "FINAL_STORE";
    case TaskPhase::CalibratingRoughPlacement:
        return "ROUGH_PLACE_CAL";
    case TaskPhase::CalibratingRoughReturn:
        return "ROUGH_RETURN_CAL";
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

const MechanismTaskExecutor::MotionDebugState &
MechanismTaskExecutor::motionDebug() const
{
    return _motionDebug;
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
    uint8_t traySlot = 0;
    if (destination == TravelDestination::Material)
    {
        traySlot = 1;
    }
    else if (destination == TravelDestination::RoughProcessing ||
             destination == TravelDestination::Storage)
    {
        // 若有原料抓取失败，直接预转第一个实际有料的槽位。
        for (uint8_t index = 0; index < MATERIALS_PER_BATCH; ++index)
        {
            if ((_collectedItemMask[_round] & (1U << index)) != 0)
            {
                traySlot = index + 1;
                break;
            }
        }
    }
    loadTravelAction(
        liftTarget,
        traySlot,
        destination != TravelDestination::Home);
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
    _batchByRound[_round] = batch;
    _itemIndex = 0;
    _alignmentForwardOffset = 0.0F;
    _alignmentRightOffset = 0.0F;
    _pickupReferenceValid = false;
    _pickupTargetSeen = false;
    _forwardCommandActive = false;
    _result = AsyncResult::Running;
    _fault = "";

    switch (task)
    {
    case StationTask::CollectMaterial:
        _collectedItemMask[_round] = 0;
        _missedItemMask[_round] = 0;
        primePickupVision(0);
        _phase = TaskPhase::CollectPreparing;
        loadTurntablePreparationAction(1);
        break;

    case StationTask::RoughProcessing:
        if (!selectFirstAvailableItem())
        {
            finishStationTask();
            break;
        }
        _phase = TaskPhase::RoughPlacing;
        loadStorageToRingAction(
            _itemIndex + 1,
            ROUGH_RING_POSES[
                _batch.roughPositions[_itemIndex]],
            0,
            hasAvailableItemAfterCurrent());
        break;

    case StationTask::StoreFinishedProduct:
        if (!selectFirstAvailableItem())
        {
            finishStationTask();
            break;
        }
        _phase = TaskPhase::FinalStoring;
        loadStorageToRingAction(
            _itemIndex + 1,
            FINAL_STORAGE_RING_POSES[
                _batch.storagePositions[_itemIndex]],
            currentStorageStackLevel(),
            hasAvailableItemAfterCurrent());
        break;
    }

    return true;
}

bool MechanismTaskExecutor::startRoughPlacementCalibration(
    uint8_t ring,
    uint16_t speed,
    uint8_t acceleration)
{
    if (!ready() ||
        _result == AsyncResult::Running ||
        !validRing(ring) ||
        speed == 0 ||
        acceleration == 0)
    {
        return false;
    }

    clearAction();
    _roughPlacementCalibrationActive = true;
    _roughPlacementCalibrationSpeed = speed;
    _roughPlacementCalibrationAcceleration = acceleration;
    _motionDebug.roughPlacementIssuedMs = 0;
    _motionDebug.roughPlacementConfirmedMs = 0;
    _result = AsyncResult::Running;
    _fault = "";
    _phase = TaskPhase::CalibratingRoughPlacement;

    const RingPose &pose = ROUGH_RING_POSES[ring];

    // 固定从储料盘3号槽取料，测试期间不再旋转载物盘。
    addStep(StepKind::Lift, LIFT_HOME);
    addStep(StepKind::RotateBase, BASE_TRAY_TRANSFER);
    addConcurrentStep(StepKind::Extend, EXTENSION_TRAY_TRANSFER);
    addConcurrentStep(StepKind::OpenGripper, GRIPPER_OPEN_ANGLE);
    addStep(StepKind::Lift, LIFT_TRAY_TRANSFER);
    addStep(StepKind::CloseGripper, GRIPPER_CLOSE_ANGLE);

    // 与正式出盘一致：先到Home，再三轴并行到圆环完整点位。
    addStep(StepKind::Lift, LIFT_HOME);
    addStep(StepKind::Lift, pose.lift);
    const uint8_t placementGroup = _steps[_stepCount - 1].group;
    appendStep(
        StepKind::RotateBaseRoughCalibration,
        pose.base,
        placementGroup);
    appendStep(
        StepKind::Extend,
        pose.extension,
        placementGroup);

    // 到圆环后放料并上抬100，便于观察落点和下一轮安全回盘。
    addStep(StepKind::OpenGripper, GRIPPER_OPEN_ANGLE);
    addStep(
        StepKind::Lift,
        max(LIFT_HOME, pose.lift - RING_PLACEMENT_RELEASE_LIFT));
    return true;
}

bool MechanismTaskExecutor::startRoughReturnCalibration(
    uint8_t ring,
    uint16_t speed,
    uint8_t acceleration)
{
    if (!ready() ||
        _result == AsyncResult::Running ||
        !validRing(ring) ||
        speed == 0 ||
        acceleration == 0)
    {
        return false;
    }

    clearAction();
    _roughReturnCalibrationActive = true;
    _roughReturnCalibrationSpeed = speed;
    _roughReturnCalibrationAcceleration = acceleration;
    _motionDebug.roughReturnIssuedMs = 0;
    _motionDebug.roughReturnConfirmedMs = 0;
    _result = AsyncResult::Running;
    _fault = "";
    _phase = TaskPhase::CalibratingRoughReturn;

    const RingPose &pose = ROUGH_RING_POSES[ring];

    /*
     * 物料由测试人员预先放在所选圆环。空爪先按正常参数
     * 从固定的3号料盘位到圆环，这一段不计入回程标定。
     */
    addStep(StepKind::Lift, LIFT_HOME);
    addStep(StepKind::Lift, pose.lift);
    const uint8_t arrivalGroup = _steps[_stepCount - 1].group;
    appendStep(StepKind::RotateBase, pose.base, arrivalGroup);
    appendStep(StepKind::Extend, pose.extension, arrivalGroup);
    appendStep(
        StepKind::OpenGripper,
        GRIPPER_OPEN_ANGLE,
        arrivalGroup);
    addStep(StepKind::CloseGripper, GRIPPER_CLOSE_ANGLE);

    // 先竖直上抬100，再用候选参数三轴并行直接回2770。
    addStep(
        StepKind::Lift,
        max(LIFT_HOME, pose.lift - RING_PICKUP_RELEASE_LIFT));
    addStep(StepKind::Lift, LIFT_HOME);
    const uint8_t returnGroup = _steps[_stepCount - 1].group;
    appendStep(
        StepKind::RotateBaseRoughReturnCalibration,
        BASE_TRAY_TRANSFER,
        returnGroup);
    appendStep(
        StepKind::Extend,
        EXTENSION_HOME,
        returnGroup);

    // 回程到位后放入固定的3号料盘位。
    addStep(StepKind::Extend, EXTENSION_TRAY_TRANSFER);
    addStep(StepKind::Lift, LIFT_TRAY_TRANSFER);
    addStep(StepKind::OpenGripper, GRIPPER_OPEN_ANGLE);
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

    if (_phase == TaskPhase::CollectApproaching)
    {
        _graspVision.update();
        updatePickupApproach();
        return;
    }

    if (_phase == TaskPhase::CollectGrasping)
    {
        /*
         * 闭爪期间继续清空当前物料的相机帧。闭爪稳定后直接升到
         * 安全高度，不再停在中间高度做二次视觉确认。
         */
        _graspVision.update();
        GraspObservation stale;
        _graspVision.takeObservation(stale);
        if (updateCurrentStep())
            onActionCompleted();
        return;
    }

    if (_phase == TaskPhase::CollectReturningToRoute)
    {
        updateReturnToMaterialRouteAnchor();
        return;
    }

    if ((_phase == TaskPhase::CollectPreparing ||
         _phase == TaskPhase::CollectDepositing ||
         _phase == TaskPhase::CollectSkipping) &&
        _primedItemIndex >= 0)
    {
        // 机械动作执行期间并行寻找本件或下一件物料。
        _graspVision.update();
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
    _primedItemIndex = -1;
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
    _storageToRoughFastProfile = false;
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

void MechanismTaskExecutor::addParallelTransfer(
    float baseTarget,
    float extensionTarget)
{
    /*
     * 调用方已经先竖直上抬并脱离物料。随后升降回安全高度、底座
     * 转向目标、伸缩到目标点同时执行。这里有意绕过通用升降独占保护。
     */
    addStep(StepKind::Lift, LIFT_HOME);
    const uint8_t departureGroup = _steps[_stepCount - 1].group;
    appendStep(
        StepKind::RotateBase,
        baseTarget,
        departureGroup);
    appendStep(
        StepKind::Extend,
        extensionTarget,
        departureGroup);
}

void MechanismTaskExecutor::addLiftThenRingTransfer(
    float clearanceLiftTarget,
    float baseTarget,
    float liftTarget,
    float extensionTarget,
    int8_t storageSlotToPrepare)
{
    /*
     * 动作一：只有升降轴抬到指定安全高度。Home点使用
     * 专用的连续两帧到位反馈完成确认。
    */
    addStep(StepKind::Lift, clearanceLiftTarget);

    /*
     * 动作二：安全高度确认后，升降、底座和伸缩三轴同组运行，
     * 一次到达圆环完整点位。
     */
    addStep(StepKind::Lift, liftTarget);
    const uint8_t ringTransferGroup = _steps[_stepCount - 1].group;
    appendStep(
        StepKind::RotateBase,
        baseTarget,
        ringTransferGroup);
    appendStep(
        StepKind::Extend,
        extensionTarget,
        ringTransferGroup);
    if (storageSlotToPrepare >= 0)
    {
        appendStoragePreparation(
            static_cast<uint8_t>(storageSlotToPrepare),
            ringTransferGroup);
    }
}

void MechanismTaskExecutor::addRoughRingToTrayRetraction(uint8_t ring)
{
    if (!validRing(ring))
    {
        fail("invalid rough ring retraction");
        return;
    }

    /*
     * 调用方已经先上抬100并脱离物料。随后升降回120、伸缩回收、
     * 底座按当前圆环慢速档直接到2770，三轴同组并行且不再经2400。
     */
    addStep(StepKind::Lift, LIFT_HOME);
    const uint8_t safeGroup = _steps[_stepCount - 1].group;
    appendStep(
        StepKind::RotateBaseRoughReturn,
        BASE_TRAY_TRANSFER,
        safeGroup);
    appendStep(
        StepKind::Extend,
        EXTENSION_HOME,
        safeGroup);
}

void MechanismTaskExecutor::addStorageDeposit(bool liftAfterDeposit)
{
    // 底座已经进入车内方向后，按固定安全顺序将物料放回载物盘。
    addStep(StepKind::Extend, EXTENSION_TRAY_TRANSFER);
    addStep(StepKind::Lift, LIFT_TRAY_TRANSFER);
    addStep(StepKind::OpenGripper, GRIPPER_OPEN_ANGLE);
    if (liftAfterDeposit)
        addStep(StepKind::Lift, LIFT_HOME);
}

void MechanismTaskExecutor::appendStoragePreparation(
    uint8_t traySlot,
    uint8_t group)
{
    if (traySlot > MATERIALS_PER_BATCH || storageSlotPrepared(traySlot))
        return;

    appendStep(
        StepKind::RotateStorage,
        TRAY_SLOT_ANGLE[traySlot],
        group);
}

bool MechanismTaskExecutor::storageSlotPrepared(uint8_t traySlot) const
{
    return traySlot <= MATERIALS_PER_BATCH &&
           _preparedStorageSlot == static_cast<int8_t>(traySlot);
}

int8_t MechanismTaskExecutor::nextAvailableTraySlot() const
{
    for (uint8_t index = _itemIndex + 1;
         index < MATERIALS_PER_BATCH;
         ++index)
    {
        if ((_collectedItemMask[_round] & (1U << index)) != 0)
            return static_cast<int8_t>(index + 1);
    }
    return -1;
}

void MechanismTaskExecutor::loadTravelAction(
    float liftTarget,
    uint8_t traySlot,
    bool useAlignmentOpenMax)
{
    clearAction();

    /*
     * 工位最后一件放下后底盘已经开始行驶，因此运输动作
     * 始终先单轴抬到Home，再收底座、伸缩和载物盘。需要
     * 工位视觉高度时，收纳完成后再降到该高度。
     */
    addStep(StepKind::Lift, LIFT_HOME);

    addStep(StepKind::RotateBase, BASE_HOME);
    addConcurrentStep(StepKind::Extend, EXTENSION_HOME);
    appendStoragePreparation(
        traySlot,
        _steps[_stepCount - 1].group);
    // 赴工位对齐时使用最大开度；回终点只用普通开度。
    addConcurrentStep(
        useAlignmentOpenMax
            ? StepKind::OpenGripperMax
            : StepKind::OpenGripper,
        useAlignmentOpenMax
            ? GRIPPER_OPEN_MAX_ANGLE
            : GRIPPER_OPEN_ANGLE);

    if (liftTarget > LIFT_HOME)
        addStep(StepKind::Lift, liftTarget);
}

void MechanismTaskExecutor::loadInitializationAction()
{
    clearAction();
    addStep(StepKind::Lift, LIFT_INITIAL);
    addStep(StepKind::RotateBase, BASE_INITIAL);
    addConcurrentStep(StepKind::Extend, EXTENSION_INITIAL);
    addConcurrentStep(
        StepKind::RotateStorage,
        TRAY_SLOT_ANGLE[_initialStorageSlot]);
    // 初始化时夹爪内没有物料，只按角度确认空载闭合。
    addConcurrentStep(
        StepKind::CloseGripperUnloaded,
        GRIPPER_CLOSE_ANGLE);
}

void MechanismTaskExecutor::loadTurntablePreparationAction(
    uint8_t traySlot,
    bool liftAlreadyHome)
{
    clearAction();

    /*
     * 首件从视觉/行驶姿态进入时必须先确认Home。连续抓取时，上一件
     * 的存盘或失败恢复已经等待Home到位，不再重复发命令和查询状态。
     */
    if (!liftAlreadyHome)
        addStep(StepKind::Lift, LIFT_HOME);
    addStep(StepKind::RotateBase, BASE_TURNTABLE);
    appendStoragePreparation(
        traySlot,
        _steps[_stepCount - 1].group);
    addConcurrentStep(
        StepKind::OpenGripperMax,
        GRIPPER_OPEN_MAX_ANGLE);

    // 先到原料抓取初始位，微调时dy优先由伸缩轴补偿。
    addStep(StepKind::Extend, EXTENSION_TURNTABLE);

    /*
     * 该动作完成后才启动原料视觉，因此搜索和追踪全程保持在150；
     * 对准完成后由Approach动作下降到LIFT_TURNTABLE夹取高度。
     */
    addStep(StepKind::Lift, LIFT_MATERIAL_VISION);
}

void MechanismTaskExecutor::loadTurntableApproachAction()
{
    clearAction();
    addStep(StepKind::Lift, LIFT_TURNTABLE);
}

void MechanismTaskExecutor::loadTurntableGraspAction()
{
    clearAction();
    addStep(StepKind::CloseGripper, GRIPPER_CLOSE_ANGLE);
}

void MechanismTaskExecutor::loadPickupToStorageAction(
    uint8_t traySlot,
    bool liftAfterDeposit)
{
    clearAction();

    /*
     * 闭爪后先升到安全高度。随后在底座回车内、伸缩轴回收的同时，
     * 再次明确将当前储料槽转到交接位，避免只依赖抓取准备阶段的
     * 预旋转状态。三个动作全部完成后才允许机械臂下降存料。
     */
    addStep(StepKind::Lift, LIFT_HOME);
    addStep(StepKind::RotateBase, BASE_TRAY_TRANSFER);
    addConcurrentStep(StepKind::Extend, EXTENSION_HOME);
    appendStoragePreparation(
        traySlot,
        _steps[_stepCount - 1].group);

    addStorageDeposit(liftAfterDeposit);
}

void MechanismTaskExecutor::loadSkippedPickupRecoveryAction()
{
    clearAction();

    /*
     * 先小幅松爪并同时竖直抬升，避免一次张开到最大时
     * 将边缘物料推飞或拖走。该并行组仍必须等Home到位。
     */
    addStep(StepKind::Lift, LIFT_HOME);
    const uint8_t releaseAndLiftGroup = _steps[_stepCount - 1].group;
    appendStep(
        StepKind::OpenGripper,
        GRIPPER_OPEN_ANGLE,
        releaseAndLiftGroup);

    // 离开物料平面后再张到最大角度，为下一件对齐做准备。
    addStep(
        StepKind::OpenGripperMax,
        GRIPPER_OPEN_MAX_ANGLE);

    // 最后底座转回车内，伸缩轴同时收回。
    addStep(StepKind::RotateBase, BASE_TRAY_TRANSFER);
    addConcurrentStep(StepKind::Extend, EXTENSION_HOME);
}

bool MechanismTaskExecutor::primePickupVision(uint8_t itemIndex)
{
    if (itemIndex >= MATERIALS_PER_BATCH)
        return false;

    if (_primedItemIndex == static_cast<int8_t>(itemIndex))
        return true;

    if (!_graspVision.startTracking(_batch.colors[itemIndex]))
    {
        _primedItemIndex = -1;
        return false;
    }

    _primedItemIndex = static_cast<int8_t>(itemIndex);
    return true;
}

void MechanismTaskExecutor::startPickupAlignment()
{
    clearAction();
    _stableFrames = 0;
    _alignmentStartedMs = millis();
    _lastObservationMs = 0;
    _pickupSearchWaypointMs = _alignmentStartedMs;
    _pickupSearchIndex = 0;
    _pickupSearchPass = 0;
    // 后续物料从第一次抓取基准开始；首个无目标帧先触发回基准。
    _pickupTargetSeen = _pickupReferenceValid;
    _alignmentObservationAfterMs =
        _alignmentStartedMs > vision_config::
                                  PRIMED_OBSERVATION_MAX_AGE_MS
            ? _alignmentStartedMs -
                  vision_config::
                      PRIMED_OBSERVATION_MAX_AGE_MS
            : 0;
    _alignmentExtensionTarget = EXTENSION_TURNTABLE;
    _graspDebug = {};
    _graspDebug.item = _itemIndex + 1;
    _graspDebug.attempt = 1;
    _graspDebug.collectedMask = _collectedItemMask[_round];
    _graspDebug.missedMask = _missedItemMask[_round];
    _graspDebug.forwardOffsetMm = _alignmentForwardOffset;
    _graspDebug.rightOffsetMm = _alignmentRightOffset;
    _graspDebug.extensionTarget = _alignmentExtensionTarget;
    _graspDebug.tracking = true;

    /*
     * 相机在机构准备和上一件存放期间已经开始找料。这里只在预启动
     * 失败或目标发生变化时重新下发命令，避免每件多等一个相机周期。
     */
    if (!primePickupVision(_itemIndex))
    {
        skipCurrentPickup();
        return;
    }

    _phase = TaskPhase::CollectAligning;
}

void MechanismTaskExecutor::updatePickupAlignment()
{
    using namespace vision_config;

    const uint32_t now = millis();
    if (_forwardPositioner.faulted())
    {
        fail("material forward positioning failed");
        return;
    }

    /*
     * dy优先由伸缩轴补偿。只有伸缩轴已完成到1500的动作，
     * 且新帧仍要求继续收缩时，才向底盘下发左移；视觉闭环
     * 不会下发右移。
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
        const uint32_t completedMs = millis();
        _pickupSearchWaypointMs = completedMs;
        _alignmentObservationAfterMs =
            completedMs >
                    MOVING_OBSERVATION_GRACE_MS
                ? completedMs -
                      MOVING_OBSERVATION_GRACE_MS
                : 0;
        _lastObservationMs = 0;
        return;
    }

    /*
     * 视觉拒绝或搜索超时属于“本件未抓到”，不是整机故障。必须等
     * 已下发的底盘/伸缩修正结束后再执行跳过收臂，避免边走边收臂。
     */
    if (_graspVision.faulted())
    {
        skipCurrentPickup();
        return;
    }

    if (now - _alignmentStartedMs >= TARGET_SEARCH_TIMEOUT_MS)
    {
        _graspDebug.tracking = false;
        skipCurrentPickup();
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
        if (now - _pickupSearchWaypointMs >=
            PICKUP_SEARCH_WAYPOINT_WAIT_MS)
        {
            if (_pickupTargetSeen && restorePickupReference())
                return;
            advancePickupSearch();
        }
        return;
    }

    // 只拒绝早于运动末段复用窗口的旧图像。
    if (static_cast<int32_t>(
            observation.receivedMs -
            _alignmentObservationAfterMs) < 0)
    {
        return;
    }

    _lastObservationMs = observation.receivedMs;
    // 任意新帧都重新开始驻点计时，避免两帧稳定确认之间误触发搜索。
    _pickupSearchWaypointMs = now;
    _graspDebug.dx = observation.dx;
    _graspDebug.dy = observation.dy;
    _graspDebug.quality = observation.quality;
    _graspDebug.found = observation.found;
    _graspDebug.hasObservation = true;
    if (!observation.found)
    {
        _stableFrames = 0;
        if (_pickupTargetSeen && restorePickupReference())
            return;
        advancePickupSearch();
        return;
    }
    if (observation.quality < MIN_QUALITY)
    {
        _stableFrames = 0;
        return;
    }
    _pickupTargetSeen = true;

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
            /*
             * 目标在下降前进入稳定抓取窗口后立即下降。下降开始后
             * 不再根据视觉结果进行确认或撤销，到位直接闭爪。
             */
            _phase = TaskPhase::CollectApproaching;
            loadTurntableApproachAction();
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
    const float leftLimit =
        fineAlignment
            ? FINE_LEFT_MAX_DELTA_MM
            : COARSE_LEFT_MAX_DELTA_MM;

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

    /*
     * 不在“本次刚好到1500”的同一轮就横移。必须先等
     * 伸缩动作到位，再用下一帧确认仍需继续收缩。
     */
    float leftDelta = 0.0F;
    if (_alignmentExtensionTarget >=
            PICKUP_EXTENSION_MAX - 0.1F &&
        extensionDelta > 0.0F)
    {
        leftDelta = clampValue(
            LEFT_FROM_DY * errorDy,
            -leftLimit,
            0.0F);
    }
    const float nextRightOffset = clampValue(
        _alignmentRightOffset + leftDelta,
        PICKUP_RIGHT_MIN_OFFSET_MM,
        PICKUP_RIGHT_MAX_OFFSET_MM);

    const float forwardMove =
        nextForwardOffset - _alignmentForwardOffset;
    const float rightMove =
        nextRightOffset - _alignmentRightOffset;
    if (fabsf(forwardMove) > FORWARD_COMMAND_DEADBAND_MM ||
        fabsf(rightMove) > RIGHT_COMMAND_DEADBAND_MM)
    {
        if (!_forwardPositioner.moveBodyRelative(
                forwardMove,
                rightMove))
        {
            fail("material chassis correction rejected");
            return;
        }
        _alignmentForwardOffset = nextForwardOffset;
        _alignmentRightOffset = nextRightOffset;
        _graspDebug.forwardOffsetMm = _alignmentForwardOffset;
        _graspDebug.rightOffsetMm = _alignmentRightOffset;
        _forwardCommandActive = true;
    }

    const bool mustReachRetractionLimit =
        nextExtensionTarget >=
            PICKUP_EXTENSION_MAX - 0.1F &&
        _alignmentExtensionTarget <
            PICKUP_EXTENSION_MAX - 0.1F;
    if (mustReachRetractionLimit ||
        fabsf(
            nextExtensionTarget -
            _alignmentExtensionTarget) >
            EXTENSION_COMMAND_DEADBAND)
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
         * 原料转盘仍在旋转。目标颜色位于视野边缘时，底盘
         * 可能已经到达本工位软限位，此时不能继续追赶，也不应立即
         * 终止整场任务。保持当前位置读取后续新帧，等待目标随转盘
         * 进入可抓取范围；TARGET_SEARCH_TIMEOUT_MS仍负责最终兜底。
         */
        _stableFrames = 0;
        return;
    }
}

bool MechanismTaskExecutor::restorePickupReference()
{
    using namespace vision_config;

    _pickupTargetSeen = false;
    if (!_pickupReferenceValid)
        return false;

    bool commanded = false;
    const float forwardMove =
        _pickupReferenceForwardOffset -
        _alignmentForwardOffset;
    const float rightMove =
        _pickupReferenceRightOffset -
        _alignmentRightOffset;
    if (fabsf(forwardMove) > FORWARD_COMMAND_DEADBAND_MM ||
        fabsf(rightMove) > RIGHT_COMMAND_DEADBAND_MM)
    {
        if (!_forwardPositioner.moveBodyRelative(
                forwardMove,
                rightMove))
        {
            fail("material reference return rejected");
            return true;
        }
        _alignmentForwardOffset =
            _pickupReferenceForwardOffset;
        _alignmentRightOffset =
            _pickupReferenceRightOffset;
        _graspDebug.forwardOffsetMm =
            _alignmentForwardOffset;
        _graspDebug.rightOffsetMm =
            _alignmentRightOffset;
        _forwardCommandActive = true;
        commanded = true;
    }

    if (fabsf(
            _pickupReferenceExtensionTarget -
            _alignmentExtensionTarget) >
        EXTENSION_COMMAND_DEADBAND)
    {
        _alignmentExtensionTarget =
            _pickupReferenceExtensionTarget;
        _graspDebug.extensionTarget =
            _alignmentExtensionTarget;
        addStep(
            StepKind::Extend,
            _alignmentExtensionTarget);
        commanded = true;
    }

    _pickupSearchWaypointMs = millis();
    return commanded;
}

void MechanismTaskExecutor::advancePickupSearch()
{
    using namespace vision_config;

    if (++_pickupSearchIndex >= PICKUP_SEARCH_POSITION_COUNT)
    {
        _pickupSearchIndex = 0;
        if (++_pickupSearchPass >= PICKUP_SEARCH_MAX_PASSES)
        {
            _graspDebug.tracking = false;
            skipCurrentPickup();
            return;
        }
    }

    const float searchCenter =
        _pickupReferenceValid
            ? _pickupReferenceForwardOffset
            : 0.0F;
    const float targetOffset = clampValue(
        searchCenter +
            PICKUP_SEARCH_OFFSETS_MM[_pickupSearchIndex],
        PICKUP_FORWARD_MIN_OFFSET_MM,
        PICKUP_FORWARD_MAX_OFFSET_MM);
    const float move = targetOffset - _alignmentForwardOffset;
    _pickupSearchWaypointMs = millis();
    _stableFrames = 0;

    if (fabsf(move) <= FORWARD_COMMAND_DEADBAND_MM)
        return;

    if (!_forwardPositioner.moveBodyRelative(move, 0.0F))
    {
        fail("material search move rejected");
        return;
    }

    _alignmentForwardOffset = targetOffset;
    _graspDebug.forwardOffsetMm = _alignmentForwardOffset;
    _forwardCommandActive = true;
}

void MechanismTaskExecutor::updatePickupApproach()
{
    if (updateCurrentStep())
        startGripperClosing();
}

void MechanismTaskExecutor::startGripperClosing()
{
    _phase = TaskPhase::CollectGrasping;
    loadTurntableGraspAction();
}

void MechanismTaskExecutor::acceptPickup()
{
    if (!_pickupReferenceValid)
    {
        _pickupReferenceForwardOffset =
            _alignmentForwardOffset;
        _pickupReferenceRightOffset =
            _alignmentRightOffset;
        _pickupReferenceExtensionTarget =
            _alignmentExtensionTarget;
        _pickupReferenceValid = true;
    }

    _graspVision.stop();
    _primedItemIndex = -1;
    _collectedItemMask[_round] |=
        static_cast<uint8_t>(1U << _itemIndex);
    _graspDebug.collectedMask = _collectedItemMask[_round];
    _graspDebug.tracking = false;
    _phase = TaskPhase::CollectDepositing;
    loadPickupToStorageAction(
        _itemIndex + 1,
        _itemIndex + 1 < MATERIALS_PER_BATCH);

    /*
     * 存放当前物料通常耗时数秒，利用这段机械动作时间提前搜索
     * 下一种颜色，到下一轮对准时直接使用最近的有效观测。
     */
    if (_itemIndex + 1 < MATERIALS_PER_BATCH)
        primePickupVision(_itemIndex + 1);
}

void MechanismTaskExecutor::skipCurrentPickup()
{
    _graspVision.stop();
    _primedItemIndex = -1;
    _missedItemMask[_round] |=
        static_cast<uint8_t>(1U << _itemIndex);
    _graspDebug.item = _itemIndex + 1;
    _graspDebug.attempt = 1;
    _graspDebug.collectedMask = _collectedItemMask[_round];
    _graspDebug.missedMask = _missedItemMask[_round];
    _graspDebug.tracking = false;
    _phase = TaskPhase::CollectSkipping;
    loadSkippedPickupRecoveryAction();

    if (_itemIndex + 1 < MATERIALS_PER_BATCH)
        primePickupVision(_itemIndex + 1);
}

void MechanismTaskExecutor::advanceAfterPickup()
{
    if (++_itemIndex < MATERIALS_PER_BATCH)
    {
        primePickupVision(_itemIndex);
        _phase = TaskPhase::CollectPreparing;
        // 上一件的存盘/失败恢复均已明确等待升降轴回到Home。
        loadTurntablePreparationAction(_itemIndex + 1, true);
        return;
    }

    startReturnToMaterialRouteAnchor();
}

bool MechanismTaskExecutor::selectFirstAvailableItem()
{
    _itemIndex = 0;
    while (_itemIndex < MATERIALS_PER_BATCH &&
           !currentItemAvailable())
    {
        ++_itemIndex;
    }
    return _itemIndex < MATERIALS_PER_BATCH;
}

bool MechanismTaskExecutor::selectNextAvailableItem()
{
    do
    {
        ++_itemIndex;
    } while (
        _itemIndex < MATERIALS_PER_BATCH &&
        !currentItemAvailable());
    return _itemIndex < MATERIALS_PER_BATCH;
}

bool MechanismTaskExecutor::currentItemAvailable() const
{
    return _itemIndex < MATERIALS_PER_BATCH &&
           (_collectedItemMask[_round] &
           static_cast<uint8_t>(1U << _itemIndex)) != 0;
}

bool MechanismTaskExecutor::hasAvailableItemAfterCurrent() const
{
    for (uint8_t index = _itemIndex + 1;
         index < MATERIALS_PER_BATCH;
         ++index)
    {
        if ((_collectedItemMask[_round] &
             static_cast<uint8_t>(1U << index)) != 0)
        {
            return true;
        }
    }
    return false;
}

uint8_t MechanismTaskExecutor::currentStorageStackLevel() const
{
    if (_round == 0 || _itemIndex >= MATERIALS_PER_BATCH)
        return 0;

    const uint8_t targetPosition =
        _batch.storagePositions[_itemIndex];
    for (uint8_t firstIndex = 0;
         firstIndex < MATERIALS_PER_BATCH;
         ++firstIndex)
    {
        if (_batchByRound[0].storagePositions[firstIndex] ==
                targetPosition &&
            (_collectedItemMask[0] &
             static_cast<uint8_t>(1U << firstIndex)) != 0)
        {
            return 1;
        }
    }

    // 第一轮对应位置没有物料时，第二轮自动按第一层高度放置。
    return 0;
}

void MechanismTaskExecutor::startReturnToMaterialRouteAnchor()
{
    _graspVision.stop();
    _primedItemIndex = -1;
    clearAction();

    if (fabsf(_alignmentForwardOffset) <= 0.1F &&
        fabsf(_alignmentRightOffset) <= 0.1F)
    {
        _alignmentForwardOffset = 0.0F;
        _alignmentRightOffset = 0.0F;
        finishStationTask();
        return;
    }

    if (!_forwardPositioner.moveBodyRelative(
            -_alignmentForwardOffset,
            -_alignmentRightOffset))
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
    _alignmentRightOffset = 0.0F;
    finishStationTask();
}

void MechanismTaskExecutor::loadStorageToRingAction(
    uint8_t traySlot,
    const RingPose &pose,
    uint8_t stackLevel,
    bool retractAfterPlacement,
    bool pickupPoseAlreadyPrepared)
{
    clearAction();
    /*
     * 粗加工出盘和暂存区第一层使用同一套快速转盘到圆环参数。
     * 暂存区第二层仍走避障串行路径并使用默认底座参数。
     */
    _storageToRoughFastProfile =
        _phase == TaskPhase::RoughPlacing ||
        (_phase == TaskPhase::FinalStoring && stackLevel == 0);

    if (!pickupPoseAlreadyPrepared)
    {
        /*
         * 首件进入工位时升降轴仍处于视觉/行驶姿态，完整执行一次
         * 回Home、底座回盘、伸缩回收、开爪和料盘定位。
         */
        addStep(StepKind::Lift, LIFT_HOME);
        addStep(StepKind::RotateBase, BASE_TRAY_TRANSFER);
        appendStoragePreparation(
            traySlot,
            _steps[_stepCount - 1].group);
        addConcurrentStep(
            StepKind::Extend,
            EXTENSION_TRAY_TRANSFER);
        addConcurrentStep(StepKind::OpenGripper, GRIPPER_OPEN_ANGLE);
    }
    else if (!storageSlotPrepared(traySlot))
    {
        /*
         * 正常情况下下一槽已在上一件赴圆环时预转完成。若预转状态
         * 丢失，仅补做料盘定位，不能带着错误槽位直接下降取料。
         */
        addStep(StepKind::RotateStorage, TRAY_SLOT_ANGLE[traySlot]);
    }

    // 连续件可从已经确认完成的车内取料姿态直接下降。
    addStep(StepKind::Lift, LIFT_TRAY_TRANSFER);
    addStep(StepKind::CloseGripper, GRIPPER_CLOSE_ANGLE);

    const float placementLiftTarget =
        pose.lift - stackLevel * MATERIAL_HEIGHT;
    if (_phase == TaskPhase::FinalStoring && stackLevel > 0)
    {
        /*
         * 第二层已经存在第一层物料，不能使用三轴并行扫掠。
         * 严格按实车验证顺序：升降到40、伸缩到目标位置、底座
         * 转到目标角度，最后才下降到第二层放置高度。
         */
        addStep(StepKind::Lift, LIFT_HOME);
        addStep(StepKind::Extend, pose.extension);
        addStep(StepKind::RotateBase, pose.base);
        const int8_t nextSlot = nextAvailableTraySlot();
        if (nextSlot >= 0)
        {
            appendStoragePreparation(
                static_cast<uint8_t>(nextSlot),
                _steps[_stepCount - 1].group);
        }
        addStep(StepKind::Lift, placementLiftTarget);
    }
    else
    {
        addLiftThenRingTransfer(
            LIFT_HOME,
            pose.base,
            placementLiftTarget,
            pose.extension,
            nextAvailableTraySlot());
    }

    // 各轴已按本层对应路径到达圆环完整点位，直接放料。
    addStep(StepKind::OpenGripper, GRIPPER_OPEN_ANGLE);

    /*
     * 圆环放料后先竖直脱离物料，再开始旋转或伸缩返回。
     * 升降坐标越小位置越高，因此上抬量从放置目标中减去。
     */
    addStep(
        StepKind::Lift,
        max(
            LIFT_HOME,
            placementLiftTarget -
                RING_PLACEMENT_RELEASE_LIFT));

    /*
     * 粗加工最后一件放下后，下一动作会直接取回第一件。此时不再
     * 先回车内交接位，避免底座和伸缩轴做一次无意义的往返。
     * 进入取回动作后仍会先升至安全高度，再转向第一个圆环。
    */
    if (retractAfterPlacement)
    {
        if (_phase == TaskPhase::RoughPlacing)
        {
            addRoughRingToTrayRetraction(
                _batch.roughPositions[_itemIndex]);
        }
        else if (stackLevel > 0)
        {
            /*
             * 第二层放置后的返程仍避障：先升至Home，再完全收回
             * 伸缩轴，最后旋转底座回储料盘。
             */
            addStep(StepKind::Lift, LIFT_HOME);
            addStep(StepKind::Extend, EXTENSION_HOME);
            addStep(StepKind::RotateBase, BASE_TRAY_TRANSFER);
        }
        else
        {
            // 第一层尚无垛料障碍，恢复原来的三轴并行回盘。
            addParallelTransfer(BASE_TRAY_TRANSFER);
        }
    }
}

void MechanismTaskExecutor::loadRingToStorageAction(
    uint8_t traySlot,
    const RingPose &pose,
    bool directRingSwitch)
{
    clearAction();
    // 普通取回从储料盘出发；第三件放置后的首次取回属于圆环直切。
    _storageToRoughFastProfile = !directRingSwitch;

    uint8_t ringArrivalGroup = 0;
    if (directRingSwitch)
    {
        /*
         * 圆环直切必须在整个横向扫掠期间保持安全高度。
         * 先单轴抬升，再只并行旋转底座和移动伸缩轴；不在
         * 旋转期间下降。
         */
        addStep(StepKind::Lift, LIFT_RING_SWITCH_CLEARANCE);
        addStep(StepKind::RotateBase, pose.base);
        ringArrivalGroup = _steps[_stepCount - 1].group;
        appendStep(
            StepKind::Extend,
            pose.extension,
            ringArrivalGroup);
    }
    else
    {
        // 储料盘到圆环：先单轴到Home，然后三轴并行到点。
        addLiftThenRingTransfer(
            LIFT_HOME,
            pose.base,
            pose.lift,
            pose.extension);
        ringArrivalGroup = _steps[_stepCount - 1].group;
    }

    // 圆环抓取使用普通开度，载物盘预旋转也在赴圆环时完成。
    appendStoragePreparation(traySlot, ringArrivalGroup);
    appendStep(
        StepKind::OpenGripper,
        GRIPPER_OPEN_ANGLE,
        ringArrivalGroup);

    if (directRingSwitch)
    {
        // 底座和伸缩完全到位后，才允许从安全高度下降。
        addStep(StepKind::Lift, pose.lift);
    }

    addStep(StepKind::CloseGripper, GRIPPER_CLOSE_ANGLE);

    // 先从圆环竖直提起物料，再进入慢速并行回盘动作。
    addStep(
        StepKind::Lift,
        max(
            LIFT_HOME,
            pose.lift - RING_PICKUP_RELEASE_LIFT));
    addRoughRingToTrayRetraction(
        _batch.roughPositions[_itemIndex]);
    addStorageDeposit(hasAvailableItemAfterCurrent());
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
    case StepKind::RotateBaseRoughCalibration:
    case StepKind::RotateBaseRoughReturn:
    case StepKind::RotateBaseRoughReturnCalibration:
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

bool MechanismTaskExecutor::issueStepperCommand(
    TTL_Stepper &motor,
    ActionStep &step,
    bool angle)
{
    resetStepperState(motor);
    bool commandAccepted = false;
    if (angle)
    {
        uint16_t velocity = BASE_SPEED;
        uint8_t acceleration = BASE_ACCELERATION;

        if (_storageToRoughFastProfile &&
            _itemIndex < MATERIALS_PER_BATCH)
        {
            const bool finalStorage =
                _phase == TaskPhase::FinalStoring;
            const uint8_t ring =
                finalStorage
                    ? _batch.storagePositions[_itemIndex]
                    : _batch.roughPositions[_itemIndex];
            if (validRing(ring))
            {
                const RingPose &targetPose =
                    finalStorage
                        ? FINAL_STORAGE_RING_POSES[ring]
                        : ROUGH_RING_POSES[ring];
                const bool movingToRing =
                    fabsf(
                        step.target -
                        targetPose.base) < 0.5F;
                if (movingToRing)
                {
                    velocity =
                        TRAY_TO_ROUGH_RING_BASE_SPEED;
                    acceleration =
                        TRAY_TO_ROUGH_RING_BASE_ACCELERATION;
                }
            }
        }

        if (step.kind == StepKind::RotateBaseRoughCalibration &&
            _roughPlacementCalibrationActive)
        {
            velocity = _roughPlacementCalibrationSpeed;
            acceleration =
                _roughPlacementCalibrationAcceleration;
        }
        else if (step.kind ==
                     StepKind::RotateBaseRoughReturnCalibration &&
                 _roughReturnCalibrationActive)
        {
            velocity = _roughReturnCalibrationSpeed;
            acceleration =
                _roughReturnCalibrationAcceleration;
        }
        else if (step.kind == StepKind::RotateBaseRoughReturn &&
                 _itemIndex < MATERIALS_PER_BATCH)
        {
            const uint8_t ring =
                _batch.roughPositions[_itemIndex];
            if (validRing(ring))
            {
                velocity = ROUGH_RING_TO_TRAY_SPEED[ring];
                acceleration =
                    ROUGH_RING_TO_TRAY_ACCELERATION[ring];
            }
        }

        commandAccepted = motor.setAngle(
            step.target,
            velocity,
            acceleration);
    }
    else
    {
        uint16_t velocity = motor.Speed;
        uint8_t acceleration = motor.Acceleration;
        if (&motor == &_lift &&
            _motionDebug.liftCommandCount > 0 &&
            step.target < _motionDebug.liftTarget - 0.5F)
        {
            // 本项目中升降坐标越小位置越高。
            velocity = LIFT_UP_SPEED;
            acceleration = LIFT_UP_ACCELERATION;
        }
        else if (&motor == &_extension &&
            _phase == TaskPhase::CollectAligning)
        {
            velocity = PICKUP_EXTENSION_TRACK_SPEED;
            acceleration =
                PICKUP_EXTENSION_TRACK_ACCELERATION;
        }

        commandAccepted = motor.runToNewPosition(
            step.target,
            velocity,
            acceleration);
    }

    if (!commandAccepted)
    {
        fail(stepperCommandFaultMessage(motor));
        return false;
    }

    step.issued = true;
    step.startedMs = millis();
    step.lastPollMs = 0;

    if (&motor == &_lift)
    {
        _motionDebug.liftTarget = step.target;
        _motionDebug.liftIssuedMs = step.startedMs;
        if (fabsf(step.target - LIFT_HOME) < 0.5F)
            _motionDebug.liftHomeConfirmedMs = 0;
        ++_motionDebug.liftCommandCount;
    }
    else if (&motor == &_extension)
    {
        _motionDebug.extensionTarget = step.target;
        _motionDebug.extensionIssuedMs = step.startedMs;
        ++_motionDebug.extensionCommandCount;
    }
    else
    {
        _motionDebug.baseTarget = step.target;
        _motionDebug.baseIssuedMs = step.startedMs;
        ++_motionDebug.baseCommandCount;
        if (step.kind == StepKind::RotateBaseRoughCalibration)
        {
            _motionDebug.roughPlacementIssuedMs =
                step.startedMs;
            _motionDebug.roughPlacementConfirmedMs = 0;
        }
        else if (step.kind ==
                 StepKind::RotateBaseRoughReturnCalibration)
        {
            _motionDebug.roughReturnIssuedMs =
                step.startedMs;
            _motionDebug.roughReturnConfirmedMs = 0;
        }
    }
    return true;
}

bool MechanismTaskExecutor::updateStepperStep(
    TTL_Stepper &motor,
    ActionStep &step,
    bool angle)
{
    const uint32_t now = millis();
    if (!step.issued)
    {
        issueStepperCommand(motor, step, angle);
        return false;
    }

    if (now - step.startedMs >= STEPPER_TIMEOUT_MS)
    {
        fail(stepperTimeoutMessage(motor));
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
        const bool criticalLiftTarget =
            &motor == &_lift &&
            (fabsf(step.target - LIFT_HOME) < 0.5F ||
             fabsf(
                 step.target -
                 LIFT_RING_SWITCH_CLEARANCE) < 0.5F);
        if (criticalLiftTarget)
        {
            if (step.onPositionFeedbackCount < LIFT_CRITICAL_CONFIRM_COUNT)
                ++step.onPositionFeedbackCount;
            if (step.onPositionFeedbackCount < LIFT_CRITICAL_CONFIRM_COUNT)
                return false;

            if (fabsf(step.target - LIFT_HOME) < 0.5F)
                _motionDebug.liftHomeConfirmedMs = now;
        }
        if (step.kind == StepKind::RotateBaseRoughCalibration &&
            _motionDebug.roughPlacementConfirmedMs == 0)
        {
            _motionDebug.roughPlacementConfirmedMs = now;
        }
        if (step.kind ==
                StepKind::RotateBaseRoughReturnCalibration &&
            _motionDebug.roughReturnConfirmedMs == 0)
        {
            _motionDebug.roughReturnConfirmedMs = now;
        }
        return true;
    }

    step.onPositionFeedbackCount = 0;

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

        /*
         * 应答丢失或只收到残帧时，供应商库会让Ask_State
         * 永久保持true。清除本次查询并释放共享总线，
         * 下一个轮询周期会自动重新发送状态查询。
         */
        if (motor.Ask_State &&
            step.queryStartedMs != 0 &&
            now - step.queryStartedMs >=
                STEPPER_QUERY_RESPONSE_TIMEOUT_MS)
        {
            motor.recDate_Clear();
            step.queryStartedMs = 0;
            _sharedPollOwner = nullptr;
            return false;
        }

        if (now - _lastSharedBusPollMs < STEPPER_POLL_MS)
            return false;

        if (_sharedPollOwner == nullptr)
            _sharedPollOwner = &motor;

        _lastSharedBusPollMs = now;
        const bool responsePending = motor.Ask_State;
        motor.state_update();

        if (!responsePending && motor.Ask_State)
            step.queryStartedMs = now;

        // Ask_State清零表示该电机的应答已经完整解析。
        if (!motor.Ask_State)
        {
            step.queryStartedMs = 0;
            _sharedPollOwner = nullptr;
        }
        return responsePending && !motor.Ask_State;
    }

    if (motor.Ask_State &&
        step.queryStartedMs != 0 &&
        now - step.queryStartedMs >=
            STEPPER_QUERY_RESPONSE_TIMEOUT_MS)
    {
        motor.recDate_Clear();
        step.queryStartedMs = 0;
        return false;
    }

    if (step.lastPollMs != 0 &&
        now - step.lastPollMs < STEPPER_POLL_MS)
        return false;

    step.lastPollMs = now;
    const bool responsePending = motor.Ask_State;
    motor.state_update();
    if (!responsePending && motor.Ask_State)
        step.queryStartedMs = now;
    else if (!motor.Ask_State)
        step.queryStartedMs = 0;
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
        _preparedStorageSlot = -1;
        _storageServo.setAngle(step.target);
        break;
    case StepKind::OpenGripper:
    case StepKind::OpenGripperMax:
        _gripperServo.setAngle(
            step.target,
            GRIPPER_OPEN_INTERVAL_MS,
            0);
        break;
    case StepKind::CloseGripperUnloaded:
    case StepKind::CloseGripper:
        _gripperServo.setAngle(
            step.target,
            GRIPPER_CLOSE_INTERVAL_MS,
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

    const uint32_t elapsed = millis() - step.startedMs;
    if (step.kind == StepKind::RotateStorage)
    {
        if (elapsed < STORAGE_SERVO_SETTLE_MS)
            return false;
        _preparedStorageSlot = storageSlotFromAngle(step.target);
        return true;
    }

    /*
     * FashionStar角度查询会同步等待串口应答。比赛主循环采用确定的
     * 命令时长加稳定余量，避免查询阻塞底盘，同时保证夹爪完成动作
     * 后才允许升降。
     */
    const uint16_t interval =
        step.kind == StepKind::OpenGripper ||
                step.kind == StepKind::OpenGripperMax
            ? GRIPPER_OPEN_INTERVAL_MS
            : GRIPPER_CLOSE_INTERVAL_MS;
    return elapsed >=
           static_cast<uint32_t>(interval) +
               GRIPPER_SETTLE_MS;
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

    case TaskPhase::CollectApproaching:
        /*
         * 下降前已经完成视觉对准；升降轴到位后直接闭爪，不再执行
         * 低位二次视觉检测或等待新帧。
         */
        startGripperClosing();
        break;

    case TaskPhase::CollectGrasping:
        acceptPickup();
        break;

    case TaskPhase::CollectDepositing:
        advanceAfterPickup();
        break;

    case TaskPhase::CollectSkipping:
        advanceAfterPickup();
        break;

    case TaskPhase::CollectAligning:
        fail("unexpected grasp alignment completion");
        break;

    case TaskPhase::CollectReturningToRoute:
        fail("unexpected route-anchor action completion");
        break;

    case TaskPhase::RoughPlacing:
        if (selectNextAvailableItem())
        {
            loadStorageToRingAction(
                _itemIndex + 1,
                ROUGH_RING_POSES[
                    _batch.roughPositions[_itemIndex]],
                0,
                hasAvailableItemAfterCurrent(),
                true);
        }
        else
        {
            _phase = TaskPhase::RoughRetrieving;
            if (selectFirstAvailableItem())
            {
                loadRingToStorageAction(
                    _itemIndex + 1,
                    ROUGH_RING_POSES[
                        _batch.roughPositions[_itemIndex]],
                    true);
            }
            else
            {
                finishStationTask();
            }
        }
        break;

    case TaskPhase::RoughRetrieving:
        if (selectNextAvailableItem())
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
        if (selectNextAvailableItem())
        {
            loadStorageToRingAction(
                _itemIndex + 1,
                FINAL_STORAGE_RING_POSES[
                    _batch.storagePositions[_itemIndex]],
                currentStorageStackLevel(),
                hasAvailableItemAfterCurrent(),
                true);
        }
        else
        {
            finishStationTask();
        }
        break;

    case TaskPhase::CalibratingRoughPlacement:
        _roughPlacementCalibrationActive = false;
        _phase = TaskPhase::Idle;
        _result = AsyncResult::Succeeded;
        clearAction();
        break;

    case TaskPhase::CalibratingRoughReturn:
        _roughReturnCalibrationActive = false;
        _phase = TaskPhase::Idle;
        _result = AsyncResult::Succeeded;
        clearAction();
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

const char *MechanismTaskExecutor::stepperTimeoutMessage(
    const TTL_Stepper &motor) const
{
    if (&motor == &_lift)
        return "lift stepper timeout";
    if (&motor == &_extension)
        return "extension stepper timeout";
    return "base stepper timeout";
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

int8_t MechanismTaskExecutor::storageSlotFromAngle(float angle)
{
    for (uint8_t slot = 0; slot <= MATERIALS_PER_BATCH; ++slot)
    {
        if (fabsf(angle - TRAY_SLOT_ANGLE[slot]) < 0.5F)
            return static_cast<int8_t>(slot);
    }
    return -1;
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
