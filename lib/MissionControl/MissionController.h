#pragma once

#include <Arduino.h>

#include "MissionPorts.h"
#include "RouteExecutor.h"
#include "start_zone.h"

/**
 * @brief 两轮比赛任务的高层非阻塞状态机。
 *
 * 本类只负责编排，不直接控制任何硬件。底盘、扫码、视觉和机械臂
 * 都通过接口协作，从而允许各模块独立调试和替换。
 */
class MissionController
{
public:
    enum class State : uint8_t
    {
        Startup,
        WaitingForStart,
        MovingToScan,
        Scanning,
        MovingToMaterial,
        AligningMaterial,
        CollectingMaterial,
        MovingToRoughProcessing,
        AligningRoughProcessing,
        RoughProcessing,
        MovingToStorage,
        AligningStorage,
        StoringFinishedProduct,
        ReturningHome,
        Finished,
        Fault
    };

    MissionController(
        RouteExecutor &route,
        IMissionDataProvider &missionData,
        IAlignmentProvider &alignment,
        IStationTaskExecutor &stationTask);

    void begin(uint32_t startupDelayMs);
    void update();
    void requestStart();
    void abort();
    bool selectStartZone(StartZone zone);

    State state() const;
    uint8_t round() const;
    StartZone startZone() const;
    bool running() const;
    bool finished() const;
    bool faulted() const;
    bool startPending() const;
    const char *faultMessage() const;

private:
    RouteExecutor &_route;
    IMissionDataProvider &_missionData;
    IAlignmentProvider &_alignment;
    IStationTaskExecutor &_stationTask;

    State _state = State::Startup;
    uint8_t _round = 0;
    MissionPlan _plan = {};
    bool _startRequested = false;
    bool _scanPlanReady = false;
    StartZone _startZone = StartZone::LowerRight;
    uint32_t _startupStartedMs = 0;
    uint32_t _startupDelayMs = 0;
    const char *_faultMessage = "";

    void updateStartup();
    void updateWaitingForStart();
    void updateRouteState();
    void updateScanning();
    void updateAlignmentState();
    void updateStationTaskState();

    bool startRoute(RouteDefinition route, State routeState);
    bool startRouteWithTravelPreparation(
        RouteDefinition route,
        State routeState,
        TravelDestination destination);
    bool startAlignment(Station station, State alignmentState);
    bool startStationTask(StationTask task, State taskState);
    bool validateStackingPlan() const;
    uint8_t storageReferenceColor() const;

    void onRouteCompleted();
    void onAlignmentCompleted();
    void onStationTaskCompleted();
    void fail(const char *message);
};
