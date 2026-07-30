#include "QRCodeMissionProvider.h"

#include <string.h>

QRCodeMissionProvider::QRCodeMissionProvider(
    HardwareSerial &serial,
    uint32_t baud,
    uint32_t timeoutMs)
    : _serial(serial),
      _baud(baud),
      _timeoutMs(timeoutMs)
{
}

void QRCodeMissionProvider::begin()
{
    _serial.begin(_baud);
    _result = AsyncResult::Idle;
    resetFrame();
}

void QRCodeMissionProvider::start()
{
    memset(&_plan, 0, sizeof(_plan));
    resetFrame();

    // 丢弃到达扫码区之前收到的旧数据，防止误用途中残留帧。
    while (_serial.available())
        _serial.read();

    _startedMs = millis();
    _result = AsyncResult::Running;
}

void QRCodeMissionProvider::update()
{
    if (_result != AsyncResult::Running)
        return;

    if (millis() - _startedMs >= _timeoutMs)
    {
        _result = AsyncResult::Failed;
        return;
    }

    while (_serial.available())
    {
        const char data = static_cast<char>(_serial.read());

        /*
         * LF可能是CRLF的第二个字节。它不参与帧内容，也不把已经
         * 正在接收的新帧清空。
         */
        if (data == '\n')
            continue;

        if (data == '\r')
        {
            if (!_discarding && _frameLength > 0)
            {
                _frame[_frameLength] = '\0';
                if (parseFrame())
                {
                    _result = AsyncResult::Succeeded;
                    return;
                }
            }

            // 无效帧不立即报故障，继续等待扫码器重发下一帧。
            resetFrame();
            continue;
        }

        if (_discarding)
            continue;

        if (_frameLength < MAX_FRAME_LENGTH)
        {
            _frame[_frameLength++] = data;
        }
        else
        {
            // 帧长度超过15字节，丢弃到下一个CR。
            _frameLength = 0;
            _discarding = true;
        }
    }
}

AsyncResult QRCodeMissionProvider::result() const
{
    return _result;
}

const MissionPlan &QRCodeMissionProvider::plan() const
{
    return _plan;
}

const char *QRCodeMissionProvider::rawText() const
{
    return _frame;
}

void QRCodeMissionProvider::resetFrame()
{
    memset(_frame, 0, sizeof(_frame));
    _frameLength = 0;
    _discarding = false;
}

bool QRCodeMissionProvider::parseFrame()
{
    /*
     * 只接受new_project的15字节四组格式。
     *
     * 第一、三组是颜色：允许1~4，但组内不可重复。
     * 第二、四组是圆环位置：必须为1、2、3的排列。
     */
    if (_frameLength != 15)
        return false;

    if (_frame[3] != '+' ||
        _frame[7] != '+' ||
        _frame[11] != '+')
        return false;

    BatchMission &first = _plan.batches[0];
    BatchMission &second = _plan.batches[1];

    if (!parseUniqueTriplet(0, 1, 4, first.colors) ||
        !parseUniqueTriplet(4, 1, 3, first.roughPositions) ||
        !parseUniqueTriplet(8, 1, 4, second.colors) ||
        !parseUniqueTriplet(12, 1, 3, second.roughPositions))
    {
        return false;
    }

    // 第一批在粗加工区和暂存区使用同一组位置。
    memcpy(
        first.storagePositions,
        first.roughPositions,
        sizeof(first.storagePositions));

    /*
     * 第二批码垛位置不直接出现在二维码中，而是由物料颜色在
     * 第一批中的暂存位置决定。
     */
    return calculateSecondBatchStoragePositions();
}

bool QRCodeMissionProvider::parseUniqueTriplet(
    uint8_t frameOffset,
    uint8_t minimum,
    uint8_t maximum,
    uint8_t output[MATERIALS_PER_BATCH])
{
    for (uint8_t i = 0; i < MATERIALS_PER_BATCH; ++i)
    {
        const char value = _frame[frameOffset + i];
        if (value < static_cast<char>('0' + minimum) ||
            value > static_cast<char>('0' + maximum))
        {
            return false;
        }

        output[i] = static_cast<uint8_t>(value - '0');
    }

    return output[0] != output[1] &&
           output[0] != output[2] &&
           output[1] != output[2];
}

bool QRCodeMissionProvider::calculateSecondBatchStoragePositions()
{
    const BatchMission &first = _plan.batches[0];
    BatchMission &second = _plan.batches[1];

    for (uint8_t secondIndex = 0;
         secondIndex < MATERIALS_PER_BATCH;
         ++secondIndex)
    {
        bool colorFound = false;

        for (uint8_t firstIndex = 0;
             firstIndex < MATERIALS_PER_BATCH;
             ++firstIndex)
        {
            if (second.colors[secondIndex] != first.colors[firstIndex])
                continue;

            second.storagePositions[secondIndex] =
                first.storagePositions[firstIndex];
            colorFound = true;
            break;
        }

        /*
         * 第二批出现第一批没有的颜色时无法确定码垛位置，
         * 因此整帧判定为无效。
         */
        if (!colorFound)
            return false;
    }

    return true;
}
