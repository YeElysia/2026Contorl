#include "StartupDiagnostics.h"

#include <math.h>

StartupDiagnostics::StartupDiagnostics(
    HardwareSerial &serial,
    uint32_t buttonPin,
    MissionController &mission,
    RouteExecutor &route,
    ChassisControl &chassis,
    MaixProRingAlignment &alignment,
    MechanismTaskExecutor &mechanism)
    : _serial(serial),
      _buttonPin(buttonPin),
      _mission(mission),
      _route(route),
      _chassis(chassis),
      _alignment(alignment),
      _mechanism(mechanism)
{
}

void StartupDiagnostics::begin(uint32_t baud)
{
    _serial.begin(baud);
    _lastButtonLevel = digitalRead(_buttonPin);
    _serial.println("DBG boot diagnostics ready");
    printSnapshot("boot");
}

void StartupDiagnostics::noteButtonClick()
{
    ++_clickCount;
    _serial.print("EVT t=");
    _serial.print(millis());
    _serial.print(" button_click count=");
    _serial.println(_clickCount);
}

void StartupDiagnostics::noteStartZoneSelection(
    StartZone zone,
    bool receivedFromScreen)
{
    _serial.print("EVT t=");
    _serial.print(millis());
    _serial.print(" start_zone=");
    _serial.print(zone == StartZone::UpperRight ? 1 : 0);
    _serial.print(" source=");
    _serial.println(receivedFromScreen ? "screen" : "default");
}

void StartupDiagnostics::update(
    uint32_t intervalMs,
    bool outputAllowed)
{
    const int buttonLevel = digitalRead(_buttonPin);
    if (buttonLevel != _lastButtonLevel)
    {
        _lastButtonLevel = buttonLevel;
        if (outputAllowed)
            printSnapshot(buttonLevel == LOW
                              ? "button_down"
                              : "button_up");
    }

    const MissionController::State missionState = _mission.state();
    if (missionState != _lastMissionState)
    {
        _lastMissionState = missionState;
        if (outputAllowed)
            printSnapshot("mission_state");
    }

    // 长诊断行会占满串口发送缓冲区，底盘行驶时不允许阻塞STEP调度。
    if (!outputAllowed)
        return;

    const uint32_t now = millis();
    if (now - _lastReportMs >= intervalMs)
        printSnapshot("periodic");
}

void StartupDiagnostics::printSnapshot(const char *reason)
{
    _lastReportMs = millis();
    const ChassisControl::Pose2D pose = _chassis.worldPose();
    const char *missionFault = _mission.faultMessage();
    const char *chassisFault = _chassis.faultMessage();
    const char *mechanismFault = _mechanism.faultMessage();
    const MechanismTaskExecutor::ServoDebugState &storage =
        _mechanism.storageServoDebug();
    const MechanismTaskExecutor::ServoDebugState &gripper =
        _mechanism.gripperServoDebug();
    const MechanismTaskExecutor::GraspDebugState &grasp =
        _mechanism.graspDebug();
    const MaixProRingAlignment::DebugState &alignment =
        _alignment.debugState();

    _serial.print("DBG t=");
    _serial.print(_lastReportMs);
    _serial.print(" reason=");
    _serial.print(reason);
    _serial.print(" raw=");
    _serial.print(digitalRead(_buttonPin));
    _serial.print(" pressed=");
    _serial.print(digitalRead(_buttonPin) == LOW ? 1 : 0);
    _serial.print(" clicks=");
    _serial.print(_clickCount);
    _serial.print(" mission=");
    _serial.print(missionStateName(_mission.state()));
    _serial.print(" pending=");
    _serial.print(_mission.startPending() ? 1 : 0);
    _serial.print(" zone=");
    _serial.print(
        _mission.startZone() == StartZone::UpperRight ? 1 : 0);
    _serial.print(" mech=");
    _serial.print(asyncResultName(_mechanism.result()));
    _serial.print("/");
    _serial.print(_mechanism.debugPhase());
    _serial.print(" ready=");
    _serial.print(_mechanism.ready() ? 1 : 0);
    _serial.print(" route=");
    _serial.print(asyncResultName(_route.result()));
    _serial.print(" chassis=");
    _serial.print(chassisStateName(_chassis.state()));
    _serial.print(" imu=");
    _serial.print(_chassis.imuReady() ? 1 : 0);
    _serial.print(" pose=");
    _serial.print(lroundf(pose.xMm));
    _serial.print(",");
    _serial.print(lroundf(pose.yMm));
    _serial.print(",");
    _serial.print(lroundf(pose.yawDeg));
    _serial.print(" align=");
    _serial.print(asyncResultName(_alignment.result()));
    _serial.print("/m");
    _serial.print(alignment.targetMode, HEX);
    _serial.print("/s");
    _serial.print(alignment.targetSelector);
    _serial.print("/");
    _serial.print(
        alignment.hasObservation && alignment.found ? 1 : 0);
    _serial.print("/");
    _serial.print(alignment.dx);
    _serial.print("/");
    _serial.print(alignment.dy);
    _serial.print("/q");
    _serial.print(alignment.quality);
    _serial.print("/n");
    _serial.print(alignment.stableFrames);
    _serial.print(alignment.movePending ? "/MOVE" : "/WAIT");
    printServoState(" storage=", storage);
    printServoState(" gripper=", gripper);
    printGraspState(grasp);
    _serial.print(" fault=");

    if (missionFault != nullptr && missionFault[0] != '\0')
        _serial.print(missionFault);
    else if (chassisFault != nullptr && chassisFault[0] != '\0')
        _serial.print(chassisFault);
    else if (mechanismFault != nullptr && mechanismFault[0] != '\0')
        _serial.print(mechanismFault);
    else
        _serial.print("-");

    _serial.println();
}

void StartupDiagnostics::printGraspState(
    const MechanismTaskExecutor::GraspDebugState &state)
{
    _serial.print(" grasp=");
    if (!state.tracking && !state.hasObservation)
    {
        _serial.print("-");
        return;
    }

    _serial.print(state.item);
    _serial.print("/");
    if (state.hasObservation)
    {
        _serial.print(state.found ? 1 : 0);
        _serial.print("/");
        _serial.print(state.dx);
        _serial.print("/");
        _serial.print(state.dy);
        _serial.print("/q");
        _serial.print(state.quality);
    }
    else
    {
        _serial.print("NA/NA/NA/qNA");
    }
    _serial.print("/f");
    _serial.print(lroundf(state.forwardOffsetMm));
    _serial.print("/e");
    _serial.print(lroundf(state.extensionTarget));
    _serial.print(state.tracking ? "/RUN" : "/STOP");
}

void StartupDiagnostics::printServoState(
    const char *label,
    const MechanismTaskExecutor::ServoDebugState &state)
{
    _serial.print(label);
    if (!state.issued)
    {
        _serial.print("-");
        return;
    }

    _serial.print(lroundf(state.target));
    _serial.print("/");
    if (state.hasActual)
        _serial.print(lroundf(state.actual));
    else
        _serial.print("NA");
    _serial.print("/s");
    _serial.print(state.status);
    _serial.print("/");
    _serial.print(state.validPolls);
    _serial.print("/");
    _serial.print(state.polls);
    _serial.print("/p");
    if (state.hasPower)
        _serial.print(state.power);
    else
        _serial.print("-");
}

const char *StartupDiagnostics::missionStateName(
    MissionController::State state)
{
    switch (state)
    {
    case MissionController::State::Startup:
        return "STARTUP";
    case MissionController::State::WaitingForStart:
        return "READY";
    case MissionController::State::MovingToScan:
        return "MOVE_SCAN";
    case MissionController::State::Scanning:
        return "SCAN";
    case MissionController::State::MovingToMaterial:
        return "MOVE_MATERIAL";
    case MissionController::State::AligningMaterial:
        return "ALIGN_MATERIAL";
    case MissionController::State::CollectingMaterial:
        return "COLLECT";
    case MissionController::State::MovingToRoughProcessing:
        return "MOVE_ROUGH";
    case MissionController::State::AligningRoughProcessing:
        return "ALIGN_ROUGH";
    case MissionController::State::RoughProcessing:
        return "ROUGH";
    case MissionController::State::MovingToStorage:
        return "MOVE_STORAGE";
    case MissionController::State::AligningStorage:
        return "ALIGN_STORAGE";
    case MissionController::State::StoringFinishedProduct:
        return "STORE";
    case MissionController::State::ReturningHome:
        return "RETURN_HOME";
    case MissionController::State::Finished:
        return "FINISHED";
    case MissionController::State::Fault:
        return "FAULT";
    }
    return "UNKNOWN";
}

const char *StartupDiagnostics::asyncResultName(AsyncResult result)
{
    switch (result)
    {
    case AsyncResult::Idle:
        return "IDLE";
    case AsyncResult::Running:
        return "RUN";
    case AsyncResult::Succeeded:
        return "OK";
    case AsyncResult::Failed:
        return "FAIL";
    }
    return "UNKNOWN";
}

const char *StartupDiagnostics::chassisStateName(
    ChassisControl::State state)
{
    switch (state)
    {
    case ChassisControl::State::Idle:
        return "IDLE";
    case ChassisControl::State::Translating:
        return "MOVE";
    case ChassisControl::State::Rotating:
        return "ROTATE";
    case ChassisControl::State::Fault:
        return "FAULT";
    }
    return "UNKNOWN";
}
