#include "MaixVisionService.h"

#include "DebugLog.h"
#include "vision_config.h"

namespace maixcam
{
MaixVisionService::MaixVisionService(MaixCamV2 &camera)
    : _camera(camera)
{
}

void MaixVisionService::begin(uint32_t baudrate)
{
    _camera.begin(baudrate);
    _token = 0;
    _hasLatest = false;
    _cameraKnown = false;
    startCommand(CMD_RESET, millis());
}

void MaixVisionService::update()
{
    using namespace vision_config;

    const uint32_t now = millis();
    Ack ack;
    Detection detection;
    for (;;)
    {
        const FrameType type = _camera.poll(ack, detection);
        if (type == FrameType::None)
            break;
        if (type == FrameType::Ack)
            handleAck(ack, now);
        else
            handleDetection(detection, now);
    }

    switch (_phase)
    {
    case Phase::ReleaseQueued:
        startCommand(CMD_RESET, now);
        break;

    case Phase::Resetting:
    case Phase::Targeting:
        if (now - _sentMs < COMMAND_RETRY_MS)
            break;
        if (_attempts >= COMMAND_MAX_ATTEMPTS)
        {
            onCommandExhausted();
            break;
        }
        LOGF_WARNING(
            "Vision",
            "RETRY cmd=%02X attempt=%u",
            static_cast<unsigned>(_command),
            static_cast<unsigned>(_attempts + 1));
        sendCommand(now);
        break;

    case Phase::Settling:
        if (now - _settleStartMs >= RESET_SETTLE_MS)
            startCommand(CMD_SET_TARGET, now);
        break;

    default:
        break;
    }
}

MaixVisionService::Token MaixVisionService::request(
    uint8_t mode,
    uint8_t selector)
{
    if (++_lastToken == 0)
        ++_lastToken;
    _token = _lastToken;
    _mode = mode;
    _selector = selector;
    _hasLatest = false;
    _skipNextDetection = false;

    LOGF_INFO(
        "Vision",
        "REQUEST token=%u target=%02X:%u",
        static_cast<unsigned>(_token),
        static_cast<unsigned>(mode),
        static_cast<unsigned>(selector));
    startTargetSequence(millis());
    return _token;
}

void MaixVisionService::release(Token token)
{
    if (token == 0 || token != _token)
        return;

    _token = 0;
    _hasLatest = false;
    switch (_phase)
    {
    case Phase::Idle:
    case Phase::Resetting:
        // 进行中的RESET应答后自然回到Idle。
        return;
    case Phase::Settling:
        // Settling只从已知IDLE进入，相机无需再复位。
        _phase = Phase::Idle;
        return;
    default:
        // 延迟到update发送，同一循环内的新请求可以直接切换目标。
        _phase = Phase::ReleaseQueued;
        return;
    }
}

MaixVisionService::Status MaixVisionService::status(
    Token token) const
{
    if (token == 0 || token != _token)
        return Status::Released;

    switch (_phase)
    {
    case Phase::Active:
        return Status::Active;
    case Phase::Rejected:
        return Status::Rejected;
    case Phase::Failed:
        return Status::Failed;
    default:
        return Status::Pending;
    }
}

bool MaixVisionService::readNew(
    Token token,
    uint32_t &cursor,
    Observation &observation) const
{
    if (token == 0 ||
        token != _token ||
        _phase != Phase::Active ||
        !_hasLatest ||
        _latest.index <= cursor)
    {
        return false;
    }

    observation = _latest;
    cursor = _latest.index;
    return true;
}

const char *MaixVisionService::statusName(Status status)
{
    switch (status)
    {
    case Status::Released:
        return "released";
    case Status::Pending:
        return "pending";
    case Status::Active:
        return "active";
    case Status::Rejected:
        return "rejected";
    case Status::Failed:
        return "failed";
    }
    return "unknown";
}

void MaixVisionService::startTargetSequence(uint32_t now)
{
    if (_phase == Phase::Resetting)
        return;
    if (_phase == Phase::Targeting)
        _cameraKnown = false;

    if (!_cameraKnown)
    {
        startCommand(CMD_RESET, now);
        return;
    }

    if (_cameraMode == MODE_IDLE)
    {
        _phase = Phase::Settling;
        _settleStartMs = _idleSinceMs;
        return;
    }

    // 相同目标或涉及圆环时必须经过IDLE，否则相机沿用旧跟踪结果。
    const bool sameTarget =
        _cameraMode == _mode && _cameraSelector == _selector;
    if (sameTarget ||
        _cameraMode == MODE_RING ||
        _mode == MODE_RING)
    {
        startCommand(CMD_RESET, now);
        return;
    }

    startCommand(CMD_SET_TARGET, now);
}

void MaixVisionService::startCommand(
    uint8_t command,
    uint32_t now)
{
    _command = command;
    _attempts = 0;
    _phase = command == CMD_RESET
                 ? Phase::Resetting
                 : Phase::Targeting;
    sendCommand(now);
}

void MaixVisionService::sendCommand(uint32_t now)
{
    const uint8_t sequence =
        _command == CMD_RESET
            ? _camera.reset()
            : _camera.setTarget(_mode, _selector);
    // 只有本服务发送命令，同一命令的重发SEQ连续递增。
    if (_attempts == 0)
        _firstSequence = sequence;
    ++_attempts;
    _sentMs = now;
}

void MaixVisionService::handleAck(const Ack &ack, uint32_t now)
{
    const bool expected =
        (_phase == Phase::Resetting ||
         _phase == Phase::Targeting) &&
        ack.command == _command &&
        static_cast<uint8_t>(ack.sequence - _firstSequence) <
            _attempts;
    if (!expected)
    {
        LOGF_DEBUG(
            "Vision",
            "STALE_ACK cmd=%02X seq=%u",
            static_cast<unsigned>(ack.command),
            static_cast<unsigned>(ack.sequence));
        return;
    }

    if (ack.result != 0)
    {
        _cameraKnown = false;
        if (_token == 0)
        {
            _phase = Phase::Idle;
            return;
        }
        _phase = Phase::Rejected;
        LOGF_ERROR(
            "Vision",
            "REJECTED cmd=%02X target=%02X:%u result=%u",
            static_cast<unsigned>(ack.command),
            static_cast<unsigned>(_mode),
            static_cast<unsigned>(_selector),
            static_cast<unsigned>(ack.result));
        return;
    }

    if (_phase == Phase::Resetting)
    {
        _cameraKnown = true;
        _cameraMode = MODE_IDLE;
        _cameraSelector = 0;
        _idleSinceMs = now;
        if (_token == 0)
        {
            _phase = Phase::Idle;
            return;
        }
        _phase = Phase::Settling;
        _settleStartMs = now;
        return;
    }

    _cameraKnown = true;
    _cameraMode = _mode;
    _cameraSelector = _selector;
    _phase = Phase::Active;
    // ACK前已开始处理的图像可能仍按旧目标计算。
    _skipNextDetection = true;
    LOGF_INFO(
        "Vision",
        "ACTIVE token=%u target=%02X:%u attempts=%u",
        static_cast<unsigned>(_token),
        static_cast<unsigned>(_mode),
        static_cast<unsigned>(_selector),
        static_cast<unsigned>(_attempts));
}

void MaixVisionService::handleDetection(
    const Detection &detection,
    uint32_t now)
{
    if (_phase != Phase::Active || detection.mode != _mode)
        return;

    if (_skipNextDetection)
    {
        _skipNextDetection = false;
        return;
    }

    _latest.detection = detection;
    _latest.receivedMs = now;
    _latest.index = ++_observationCount;
    _hasLatest = true;
}

void MaixVisionService::onCommandExhausted()
{
    _cameraKnown = false;
    if (_token == 0)
    {
        _phase = Phase::Idle;
        LOG_WARNING("Vision", "RESET unacknowledged");
        return;
    }

    _phase = Phase::Failed;
    LOGF_ERROR(
        "Vision",
        "NO_ACK cmd=%02X target=%02X:%u",
        static_cast<unsigned>(_command),
        static_cast<unsigned>(_mode),
        static_cast<unsigned>(_selector));
}
} // namespace maixcam
