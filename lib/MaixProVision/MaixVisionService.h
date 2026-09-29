#pragma once

#include "MaixCamV2.h"

namespace maixcam
{
/**
 * @brief 共享相机的唯一入口。
 *
 * 每次request分配新令牌，新请求会顶替旧令牌。服务负责命令重发、
 * 按SEQ匹配ACK、经过IDLE清空圆环跟踪器，以及丢弃切换后第一帧。
 * update()必须每个主循环调用一次，空闲时也要持续读空串口。
 */
class MaixVisionService
{
public:
    using Token = uint16_t;

    enum class Status : uint8_t
    {
        // 令牌已释放或被新请求顶替。
        Released,
        Pending,
        Active,
        Rejected,
        Failed
    };

    struct Observation
    {
        Detection detection;
        uint32_t receivedMs = 0;
        uint32_t index = 0;
    };

    explicit MaixVisionService(MaixCamV2 &camera);

    void begin(uint32_t baudrate);
    void update();

    Token request(uint8_t mode, uint8_t selector);
    void release(Token token);
    Status status(Token token) const;

    /**
     * @brief 读取比cursor更新的最近一帧，不影响其他读者。
     *
     * 只返回当前令牌生效后、模式匹配的帧；cursor由调用方保存。
     */
    bool readNew(
        Token token,
        uint32_t &cursor,
        Observation &observation) const;

    static const char *statusName(Status status);

private:
    enum class Phase : uint8_t
    {
        Idle,
        ReleaseQueued,
        Resetting,
        Settling,
        Targeting,
        Active,
        Rejected,
        Failed
    };

    MaixCamV2 &_camera;
    Phase _phase = Phase::Idle;
    Token _token = 0;
    Token _lastToken = 0;
    uint8_t _mode = MODE_IDLE;
    uint8_t _selector = 0;

    bool _cameraKnown = false;
    uint8_t _cameraMode = MODE_IDLE;
    uint8_t _cameraSelector = 0;
    uint32_t _idleSinceMs = 0;

    uint8_t _command = 0;
    uint8_t _firstSequence = 0;
    uint8_t _attempts = 0;
    uint32_t _sentMs = 0;
    uint32_t _settleStartMs = 0;

    bool _skipNextDetection = false;
    bool _hasLatest = false;
    Observation _latest;
    uint32_t _observationCount = 0;

    void startTargetSequence(uint32_t now);
    void startCommand(uint8_t command, uint32_t now);
    void sendCommand(uint32_t now);
    void handleAck(const Ack &ack, uint32_t now);
    void handleDetection(const Detection &detection, uint32_t now);
    void onCommandExhausted();
};
} // namespace maixcam
