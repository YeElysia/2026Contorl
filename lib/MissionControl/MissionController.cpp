#include "MissionController.h"

#include <string.h>

#include "MissionRoutes.h"

using namespace mission_routes;

MissionController::MissionController(
    RouteExecutor &route,
    IMissionDataProvider &missionData,
    IAlignmentProvider &alignment,
    IStationTaskExecutor &stationTask)
    : _route(route),
      _missionData(missionData),
      _alignment(alignment),
      _stationTask(stationTask)
{
}

void MissionController::begin(uint32_t startupDelayMs)
{
    _route.cancel();
    _alignment.cancel();
    _stationTask.cancel();

    memset(&_plan, 0, sizeof(_plan));
    _round = 0;
    _startRequested = false;
    _scanPlanReady = false;
    _startupDelayMs = startupDelayMs;
    _startupStartedMs = millis();
    _faultMessage = "";
    _state = State::Startup;
}

void MissionController::update()
{
    // 各服务始终独立更新，任务状态机不承担其通信或控制细节。
    _missionData.update();
    _alignment.update();
    _stationTask.update();
    _route.update();

    /*
     * 机构可能在底盘行驶期间执行收纳动作。此时发生堵转或超时
     * 也必须立即终止整车任务，不能等到达下一个工位才发现。
     */
    if (_state != State::Fault &&
        _stationTask.result() == AsyncResult::Failed)
    {
        const char *message = _stationTask.faultMessage();
        fail(message && message[0] ? message : "mechanism task failed");
        return;
    }

    switch (_state)
    {
    case State::Startup:
        updateStartup();
        break;

    case State::WaitingForStart:
        updateWaitingForStart();
        break;

    case State::MovingToScan:
    case State::MovingToMaterial:
    case State::MovingToRoughProcessing:
    case State::MovingToStorage:
    case State::ReturningHome:
        updateRouteState();
        break;

    case State::Scanning:
        updateScanning();
        break;

    case State::AligningMaterial:
    case State::AligningRoughProcessing:
    case State::AligningStorage:
        updateAlignmentState();
        break;

    case State::CollectingMaterial:
    case State::RoughProcessing:
    case State::StoringFinishedProduct:
        updateStationTaskState();
        break;

    case State::Finished:
    case State::Fault:
        break;
    }
}

void MissionController::requestStart()
{
    _startRequested = true;
}

void MissionController::abort()
{
    _route.cancel();
    _alignment.cancel();
    _stationTask.cancel();
    fail("mission aborted");
}

bool MissionController::selectStartZone(StartZone zone)
{
    /*
     * 启停区只允许在发车前修改。运行途中改变会让返程目标与实际
     * 出发位置不一致，因此直接拒绝。
     */
    if (_state != State::Startup &&
        _state != State::WaitingForStart)
    {
        return false;
    }

    _startZone = zone;
    return true;
}

MissionController::State MissionController::state() const
{
    return _state;
}

uint8_t MissionController::round() const
{
    return _round;
}

StartZone MissionController::startZone() const
{
    return _startZone;
}

bool MissionController::running() const
{
    return _state != State::Startup &&
           _state != State::WaitingForStart &&
           _state != State::Finished &&
           _state != State::Fault;
}

bool MissionController::finished() const
{
    return _state == State::Finished;
}

bool MissionController::faulted() const
{
    return _state == State::Fault;
}

bool MissionController::startPending() const
{
    return _startRequested;
}

const char *MissionController::faultMessage() const
{
    return _faultMessage;
}

void MissionController::updateStartup()
{
    if (millis() - _startupStartedMs < _startupDelayMs)
        return;

    // 机械臂保持上电初始化位，运输收纳动作改在扫码区启动。
    _state = State::WaitingForStart;
}

void MissionController::updateWaitingForStart()
{
    if (!_startRequested)
        return;

    _startRequested = false;
    _round = 0;
    _scanPlanReady = false;

    // 首段只移动底盘，机械臂到达扫码区后再从初始化位切到收纳位。
    if (!startRoute(
            ROUTE_TO_SCAN,
            State::MovingToScan))
        fail("failed to start route to scan station");
}

void MissionController::updateRouteState()
{
    switch (_route.result())
    {
    case AsyncResult::Succeeded:
        onRouteCompleted();
        break;
    case AsyncResult::Failed:
    {
        const char *message = _route.faultMessage();
        fail(message && message[0]
                 ? message
                 : "chassis route failed");
        break;
    }
    default:
        break;
    }
}

void MissionController::updateScanning()
{
    if (!_scanPlanReady)
    {
        switch (_missionData.result())
        {
        case AsyncResult::Succeeded:
            _plan = _missionData.plan();
            if (!validateStackingPlan())
            {
                fail("mission stacking color mapping invalid");
                return;
            }
            _scanPlanReady = true;
            break;

        case AsyncResult::Failed:
            fail("QR mission data invalid");
            return;

        default:
            return;
        }
    }

    /*
     * 扫码和机械臂收纳同时进行。二维码先读完时留在扫码区等待，
     * 避免机械臂尚未达到安全运输位就转向原料区。
     */
    if (_stationTask.result() == AsyncResult::Running)
        return;

    if (!startRoute(
            ROUTE_SCAN_TO_MATERIAL,
            State::MovingToMaterial))
    {
        fail("failed to start route to material station");
    }
}

void MissionController::updateAlignmentState()
{
    switch (_alignment.result())
    {
    case AsyncResult::Succeeded:
        onAlignmentCompleted();
        break;
    case AsyncResult::Failed:
        fail("station alignment failed");
        break;
    default:
        break;
    }
}

void MissionController::updateStationTaskState()
{
    switch (_stationTask.result())
    {
    case AsyncResult::Succeeded:
        onStationTaskCompleted();
        break;
    default:
        break;
    }
}

bool MissionController::startRoute(
    RouteDefinition route,
    State routeState)
{
    if (!_route.start(route))
        return false;

    _state = routeState;
    return true;
}

bool MissionController::startRouteWithTravelPreparation(
    RouteDefinition route,
    State routeState,
    TravelDestination destination)
{
    if (!startRoute(route, routeState))
        return false;

    /*
     * 路线已经启动后再发机构收纳命令，两者从同一次update开始并行。
     * 若机构拒绝动作，立即取消刚启动的路线，保持故障处理原子性。
     */
    if (!_stationTask.prepareForTravel(destination))
    {
        _route.cancel();
        return false;
    }

    return true;
}

bool MissionController::startAlignment(
    Station station,
    State alignmentState)
{
    AlignmentRequest request;
    request.station = station;
    request.round = _round;
    if (station == Station::Storage && _round == 1)
        request.referenceColor = storageReferenceColor();

    if (!_alignment.start(request))
        return false;

    _state = alignmentState;
    return true;
}

bool MissionController::validateStackingPlan() const
{
    const BatchMission &first = _plan.batches[0];
    const BatchMission &second = _plan.batches[1];
    uint8_t storageReferenceCount = 0;

    for (uint8_t firstIndex = 0;
         firstIndex < MATERIALS_PER_BATCH;
         ++firstIndex)
    {
        if (first.colors[firstIndex] < 1 ||
            first.colors[firstIndex] > 4 ||
            second.colors[firstIndex] < 1 ||
            second.colors[firstIndex] > 4 ||
            first.storagePositions[firstIndex] < 1 ||
            first.storagePositions[firstIndex] > 3)
        {
            return false;
        }

        if (first.storagePositions[firstIndex] == 2)
            ++storageReferenceCount;

        for (uint8_t other = firstIndex + 1;
             other < MATERIALS_PER_BATCH;
             ++other)
        {
            if (first.colors[firstIndex] == first.colors[other] ||
                second.colors[firstIndex] == second.colors[other] ||
                first.storagePositions[firstIndex] ==
                    first.storagePositions[other])
            {
                return false;
            }
        }
    }

    if (storageReferenceCount != 1)
        return false;

    /*
     * 第二批每个物料必须能在第一批找到唯一同色物料，并且其码垛
     * 位置必须等于第一批同色物料的暂存位置。
     */
    for (uint8_t secondIndex = 0;
         secondIndex < MATERIALS_PER_BATCH;
         ++secondIndex)
    {
        uint8_t matchingFirstCount = 0;
        uint8_t expectedPosition = 0;
        for (uint8_t firstIndex = 0;
             firstIndex < MATERIALS_PER_BATCH;
             ++firstIndex)
        {
            if (second.colors[secondIndex] ==
                first.colors[firstIndex])
            {
                ++matchingFirstCount;
                expectedPosition =
                    first.storagePositions[firstIndex];
            }
        }

        if (matchingFirstCount != 1 ||
            second.storagePositions[secondIndex] != expectedPosition)
        {
            return false;
        }
    }

    return true;
}

uint8_t MissionController::storageReferenceColor() const
{
    const BatchMission &first = _plan.batches[0];
    for (uint8_t i = 0; i < MATERIALS_PER_BATCH; ++i)
    {
        if (first.storagePositions[i] == 2)
            return first.colors[i];
    }
    return 0;
}

bool MissionController::startStationTask(
    StationTask task,
    State taskState)
{
    if (!_stationTask.start(
            task,
            _round,
            _plan.batches[_round]))
    {
        return false;
    }

    _state = taskState;
    return true;
}

void MissionController::onRouteCompleted()
{
    switch (_state)
    {
    case State::MovingToScan:
        /*
         * 到达扫码区后同时启动扫描和机构收纳：途中不会误读二维码，
         * 出发时机械臂也保持上电初始化位。
         */
        if (!_stationTask.prepareForTravel(
                TravelDestination::Material))
        {
            fail("failed to prepare mechanism at scan station");
            break;
        }
        _scanPlanReady = false;
        _missionData.start();
        _state = State::Scanning;
        break;

    case State::MovingToMaterial:
        if (_stationTask.result() == AsyncResult::Running)
            return;
        if (!startAlignment(
                Station::Material,
                State::AligningMaterial))
            fail("failed to start material alignment");
        break;

    case State::MovingToRoughProcessing:
        if (_stationTask.result() == AsyncResult::Running)
            return;
        if (!startAlignment(
                Station::RoughProcessing,
                State::AligningRoughProcessing))
            fail("failed to start rough-process alignment");
        break;

    case State::MovingToStorage:
        if (_stationTask.result() == AsyncResult::Running)
            return;
        if (!startAlignment(
                Station::Storage,
                State::AligningStorage))
            fail("failed to start storage alignment");
        break;

    case State::ReturningHome:
        // 终点必须同时满足底盘到位和机械臂完成运输收纳。
        if (_stationTask.result() == AsyncResult::Running)
            return;
        _state = State::Finished;
        break;

    default:
        fail("unexpected route completion");
        break;
    }
}

void MissionController::onAlignmentCompleted()
{
    /*
     * 短路线可能比机械臂收纳更早完成。保持在对准状态等待机构，
     * 确保绝不会在上一段运输动作尚未完成时启动下一工位动作。
     */
    if (_stationTask.result() == AsyncResult::Running)
        return;

    switch (_state)
    {
    case State::AligningMaterial:
        if (!startStationTask(
                StationTask::CollectMaterial,
                State::CollectingMaterial))
            fail("failed to start material task");
        break;

    case State::AligningRoughProcessing:
        if (!startStationTask(
                StationTask::RoughProcessing,
                State::RoughProcessing))
            fail("failed to start rough-process task");
        break;

    case State::AligningStorage:
        if (!startStationTask(
                StationTask::StoreFinishedProduct,
                State::StoringFinishedProduct))
            fail("failed to start storage task");
        break;

    default:
        fail("unexpected alignment completion");
        break;
    }
}

void MissionController::onStationTaskCompleted()
{
    switch (_state)
    {
    case State::CollectingMaterial:
        if (!startRouteWithTravelPreparation(
                ROUTE_MATERIAL_TO_ROUGH[_round],
                State::MovingToRoughProcessing,
                TravelDestination::RoughProcessing))
            fail("failed to leave material task");
        break;

    case State::RoughProcessing:
        if (!startRouteWithTravelPreparation(
                ROUTE_ROUGH_TO_STORAGE[_round],
                State::MovingToStorage,
                TravelDestination::Storage))
            fail("failed to leave rough-process task");
        break;

    case State::StoringFinishedProduct:
        if (_round == 0)
        {
            _round = 1;
            if (!startRouteWithTravelPreparation(
                    ROUTE_STORAGE_TO_MATERIAL_SECOND,
                    State::MovingToMaterial,
                    TravelDestination::Material))
                fail("failed to start second round");
        }
        else
        {
            if (!startRouteWithTravelPreparation(
                    storageToHomeRoute(_startZone),
                    State::ReturningHome,
                    TravelDestination::Home))
                fail("failed to start return-home route");
        }
        break;

    default:
        fail("unexpected station-task completion");
        break;
    }
}

void MissionController::fail(const char *message)
{
    _route.cancel();
    _alignment.cancel();
    _stationTask.cancel();
    _faultMessage = message;
    _state = State::Fault;
}
