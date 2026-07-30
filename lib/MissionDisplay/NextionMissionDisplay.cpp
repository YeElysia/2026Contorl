#include "NextionMissionDisplay.h"

#include <stdio.h>
#include <string.h>

NextionMissionDisplay::NextionMissionDisplay(
    HardwareSerial &serial,
    MissionController &mission,
    IMissionDataProvider &missionData)
    : _serial(serial),
      _mission(mission),
      _missionData(missionData)
{
}

void NextionMissionDisplay::begin(
    uint32_t baud,
    uint32_t restartWaitMs)
{
    _serial.begin(baud);
    sendCommand("rest");

    /*
     * 屏幕重启期间不使用delay()，底盘和IMU仍可正常更新。
     * 到时后update()会自动刷新当前任务状态。
     */
    _screenReadyMs = millis() + restartWaitMs;
    _lastState = static_cast<MissionController::State>(0xFF);
    _lastQrText[0] = '\0';
}

void NextionMissionDisplay::update()
{
    if (static_cast<int32_t>(millis() - _screenReadyMs) < 0)
        return;

    readResponses();
    updateState();
    updateQrText();
    updateStartZoneSelection();
}

StartZone NextionMissionDisplay::selectedStartZone() const
{
    return _selectedStartZone;
}

bool NextionMissionDisplay::hasStartZoneSelection() const
{
    return _hasStartZoneSelection;
}

void NextionMissionDisplay::updateStartZoneSelection()
{
    /*
     * 仅在发车前读取DEBUG.n4，避免比赛运行期间无意义地占用串口。
     * 200ms周期允许操作界面后很快刷新，同时不会淹没屏幕接收缓冲区。
     */
    const MissionController::State state = _mission.state();
    if (state != MissionController::State::Startup &&
        state != MissionController::State::WaitingForStart)
    {
        return;
    }

    const uint32_t now = millis();
    if (now - _lastStartZoneQueryMs < 200)
        return;

    _lastStartZoneQueryMs = now;
    sendCommand("get DEBUG.n4.val");
}

void NextionMissionDisplay::readResponses()
{
    while (_serial.available())
    {
        const uint8_t value =
            static_cast<uint8_t>(_serial.read());

        if (value == 0xFF)
        {
            if (++_responseTerminatorCount == 3)
            {
                processResponse();
                _responseLength = 0;
                _responseTerminatorCount = 0;
            }
            continue;
        }

        /*
         * 终止符必须连续出现。异常帧中的单个0xFF不应让下一包被误判
         * 为完整响应；该字节本身也不是数值响应的有效内容。
         */
        _responseTerminatorCount = 0;
        if (_responseLength < sizeof(_response))
            _response[_responseLength++] = value;
        else
            _responseLength = 0;
    }
}

void NextionMissionDisplay::processResponse()
{
    // Nextion数值返回：0x71 + uint32小端值 + 0xFF 0xFF 0xFF。
    if (_responseLength != 5 || _response[0] != 0x71)
        return;

    const uint32_t value =
        static_cast<uint32_t>(_response[1]) |
        (static_cast<uint32_t>(_response[2]) << 8) |
        (static_cast<uint32_t>(_response[3]) << 16) |
        (static_cast<uint32_t>(_response[4]) << 24);

    _selectedStartZone = value == 1
                             ? StartZone::UpperRight
                             : StartZone::LowerRight;
    _hasStartZoneSelection = true;
}

void NextionMissionDisplay::updateState()
{
    const MissionController::State current = _mission.state();
    if (current == _lastState)
        return;

    _lastState = current;
    showState(current);
}

void NextionMissionDisplay::updateQrText()
{
    if (_missionData.result() != AsyncResult::Succeeded)
        return;

    const char *current = _missionData.rawText();
    if (current == nullptr ||
        strncmp(current, _lastQrText, sizeof(_lastQrText)) == 0)
    {
        return;
    }

    strncpy(_lastQrText, current, sizeof(_lastQrText) - 1);
    _lastQrText[sizeof(_lastQrText) - 1] = '\0';

    /*
     * 沿用new_project的任务码显示方式：
     *
     *   134+123+
     *   314+231
     *
     * 即前8个字符显示在第一行，剩余7个字符显示在第二行。
     * 这里只改变屏幕文本，解析器中保存的原始任务码保持不变。
     */
    char wrappedQrText[18] = {};
    strncpy(wrappedQrText, _lastQrText, 8);
    wrappedQrText[8] = '\r';
    wrappedQrText[9] = '\n';
    strncpy(
        wrappedQrText + 10,
        _lastQrText + 8,
        sizeof(wrappedQrText) - 11);

    setText("RUN.t0", wrappedQrText);
}

void NextionMissionDisplay::showState(
    MissionController::State state)
{
    switch (state)
    {
    case MissionController::State::Startup:
        sendCommand("page DEBUG");
        setText("DEBUG.t4", "Initializing");
        break;

    case MissionController::State::WaitingForStart:
        sendCommand("page DEBUG");
        setText("DEBUG.t4", "Ready");
        break;

    case MissionController::State::Finished:
        /*
         * 兼容new_project：先在运行页写入done，再切换结束页。
         * 若HMI工程没有END页，可删除下面的page命令。
         */
        setText("RUN.t1", "done");
        sendCommand("page END");
        break;

    case MissionController::State::Fault:
        sendCommand("page DEBUG");
        setText("DEBUG.t4", "Fault");
        setText("DEBUG.t0", _mission.faultMessage());
        break;

    default:
        sendCommand("page RUN");
        setText("RUN.t1", stateText(state));
        break;
    }
}

void NextionMissionDisplay::sendCommand(const char *command)
{
    _serial.print(command);
    _serial.write(0xFF);
    _serial.write(0xFF);
    _serial.write(0xFF);
}

void NextionMissionDisplay::setText(
    const char *component,
    const char *text)
{
    /*
     * 状态文本和二维码只包含ASCII字符，不允许外部内容直接拼接
     * 任意Nextion指令。
     */
    char command[96];
    snprintf(
        command,
        sizeof(command),
        "%s.txt=\"%s\"",
        component,
        text != nullptr ? text : "");
    sendCommand(command);
}

const char *NextionMissionDisplay::stateText(
    MissionController::State state)
{
    switch (state)
    {
    case MissionController::State::MovingToScan:
        return "MovingToScan";
    case MissionController::State::Scanning:
        return "Scanning";
    case MissionController::State::MovingToMaterial:
        return "MovingToMaterial";
    case MissionController::State::AligningMaterial:
        return "AligningMaterial";
    case MissionController::State::CollectingMaterial:
        return "CollectingMaterial";
    case MissionController::State::MovingToRoughProcessing:
        return "MovingToRough";
    case MissionController::State::AligningRoughProcessing:
        return "AligningRough";
    case MissionController::State::RoughProcessing:
        return "RoughProcessing";
    case MissionController::State::MovingToStorage:
        return "MovingToStorage";
    case MissionController::State::AligningStorage:
        return "AligningStorage";
    case MissionController::State::StoringFinishedProduct:
        return "StoringProduct";
    case MissionController::State::ReturningHome:
        return "ReturningHome";
    case MissionController::State::Startup:
        return "Initializing";
    case MissionController::State::WaitingForStart:
        return "Ready";
    case MissionController::State::Finished:
        return "done";
    case MissionController::State::Fault:
        return "Fault";
    }

    return "Unknown";
}
