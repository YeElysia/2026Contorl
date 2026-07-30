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

MissionController::State MissionController::state() const
{
    return _state;
}

uint8_t MissionController::round() const
{
    return _round;
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

const char *MissionController::faultMessage() const
{
    return _faultMessage;
}

void MissionController::updateStartup()
{
    if (_stationTask.result() == AsyncResult::Failed)
    {
        const char *message = _stationTask.faultMessage();
        fail(message && message[0] ? message : "mechanism initialization failed");
        return;
    }

    if (millis() - _startupStartedMs < _startupDelayMs)
        return;

    if (!_stationTask.ready())
        return;

    _state = State::WaitingForStart;
}

void MissionController::updateWaitingForStart()
{
    if (!_startRequested)
        return;

    _startRequested = false;
    _round = 0;

    if (!startRoute(TO_SCAN, countOf(TO_SCAN), State::MovingToScan))
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
        fail("chassis route failed");
        break;
    default:
        break;
    }
}

void MissionController::updateScanning()
{
    switch (_missionData.result())
    {
    case AsyncResult::Succeeded:
        _plan = _missionData.plan();
        if (!startRoute(
                SCAN_TO_MATERIAL,
                countOf(SCAN_TO_MATERIAL),
                State::MovingToMaterial))
        {
            fail("failed to start route to material station");
        }
        break;

    case AsyncResult::Failed:
        fail("QR mission data invalid");
        break;

    default:
        break;
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
    case AsyncResult::Failed:
    {
        const char *message = _stationTask.faultMessage();
        fail(message && message[0] ? message : "station mechanism task failed");
        break;
    }
    default:
        break;
    }
}

bool MissionController::startRoute(
    const RouteAction *actions,
    size_t count,
    State routeState)
{
    if (!_route.start(actions, count))
        return false;

    _state = routeState;
    return true;
}

bool MissionController::startAlignment(
    Station station,
    State alignmentState)
{
    if (!_alignment.start(station, _round))
        return false;

    _state = alignmentState;
    return true;
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
         * 到达扫码区后才启动扫描，避免途中误读场地上的其他二维码。
         * start()之后由Scanning状态持续等待异步结果。
         */
        _missionData.start();
        _state = State::Scanning;
        break;

    case State::MovingToMaterial:
        if (!startAlignment(
                Station::Material,
                State::AligningMaterial))
            fail("failed to start material alignment");
        break;

    case State::MovingToRoughProcessing:
        if (!startAlignment(
                Station::RoughProcessing,
                State::AligningRoughProcessing))
            fail("failed to start rough-process alignment");
        break;

    case State::MovingToStorage:
        if (!startAlignment(
                Station::Storage,
                State::AligningStorage))
            fail("failed to start storage alignment");
        break;

    case State::ReturningHome:
        _state = State::Finished;
        break;

    default:
        fail("unexpected route completion");
        break;
    }
}

void MissionController::onAlignmentCompleted()
{
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
        if (_round == 0)
        {
            if (!startRoute(
                    MATERIAL_TO_ROUGH_FIRST,
                    countOf(MATERIAL_TO_ROUGH_FIRST),
                    State::MovingToRoughProcessing))
                fail("failed to leave first material task");
        }
        else
        {
            if (!startRoute(
                    MATERIAL_TO_ROUGH_SECOND,
                    countOf(MATERIAL_TO_ROUGH_SECOND),
                    State::MovingToRoughProcessing))
                fail("failed to leave second material task");
        }
        break;

    case State::RoughProcessing:
        if (_round == 0)
        {
            if (!startRoute(
                    ROUGH_TO_STORAGE_FIRST,
                    countOf(ROUGH_TO_STORAGE_FIRST),
                    State::MovingToStorage))
                fail("failed to leave first rough-process task");
        }
        else
        {
            if (!startRoute(
                    ROUGH_TO_STORAGE_SECOND,
                    countOf(ROUGH_TO_STORAGE_SECOND),
                    State::MovingToStorage))
                fail("failed to leave second rough-process task");
        }
        break;

    case State::StoringFinishedProduct:
        if (_round == 0)
        {
            _round = 1;
            if (!startRoute(
                    STORAGE_TO_MATERIAL_SECOND,
                    countOf(STORAGE_TO_MATERIAL_SECOND),
                    State::MovingToMaterial))
                fail("failed to start second round");
        }
        else
        {
            if (!startRoute(
                    STORAGE_TO_HOME,
                    countOf(STORAGE_TO_HOME),
                    State::ReturningHome))
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
