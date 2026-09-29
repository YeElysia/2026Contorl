#include <Arduino.h>
#include <OneButton.h>

#include "GraspVisionPorts.h"
#include "MechanismTaskExecutor.h"
#include "debug_config.h"
#include "mechanism_config.h"
#include "mission_config.h"
#include "vision_config.h"

namespace
{
/**
 * @brief 机械臂独立测试使用的居中目标。
 *
 * 测试固件不连接MaixPro，每50 ms产生一次观测。第一次跟踪会在
 * 对准完成后模拟物料随转盘漂走，以验证非致命重试；第二次及后续
 * 跟踪保持居中。这里只替换视觉输入，机构动作和保护与比赛一致。
 */
class CenteredTestVision : public IGraspVisionProvider
{
public:
    void begin() override
    {
        _tracking = false;
        _observationReady = false;
    }

    bool startTracking(uint8_t color) override
    {
        if (color < 1 || color > 4)
            return false;

        _tracking = true;
        _observationReady = false;
        _lastObservationMs = 0;
        _observationCount = 0;
        if (_trackingSession < UINT8_MAX)
            ++_trackingSession;
        return true;
    }

    void update() override
    {
        if (!_tracking)
            return;

        const uint32_t now = millis();
        if (_lastObservationMs == 0 ||
            now - _lastObservationMs >= 50)
        {
            _lastObservationMs = now;
            _observationReady = true;
        }
    }

    bool takeObservation(GraspObservation &observation) override
    {
        if (!_observationReady)
            return false;

        _observationReady = false;
        observation.found = true;
        observation.dx = vision_config::TARGET_DX_PX;
        observation.dy = vision_config::TARGET_DY_PX;
        observation.quality = 255;
        observation.receivedMs = millis();
        if (_observationCount < UINT8_MAX)
            ++_observationCount;
        return true;
    }

    void stop() override
    {
        _tracking = false;
        _observationReady = false;
    }

    bool faulted() const override
    {
        return false;
    }

private:
    bool _tracking = false;
    bool _observationReady = false;
    uint32_t _lastObservationMs = 0;
    uint8_t _trackingSession = 0;
    uint8_t _observationCount = 0;
};

/**
 * @brief 机构独立测试中的底盘前后微调替身。
 *
 * 居中视觉不会实际提交移动；保留本替身用于验证正式接口接线。
 */
class ImmediateTestForwardPositioner : public IGraspForwardPositioner
{
public:
    bool moveBodyRelative(float, float) override
    {
        return true;
    }

    bool busy() const override
    {
        return false;
    }

    bool faulted() const override
    {
        return false;
    }

    void stop() override {}
    void release() override {}
};

enum class TestStage : uint8_t
{
    TravelPreparation,
    CollectMaterial,
    RoughProcessing,
    StoreFinishedProduct,
    Completed
};

HardwareSerial serialMechanismStepper(
    mechanism_config::STEPPER_RX_PIN,
    mechanism_config::STEPPER_TX_PIN);
HardwareSerial serialMechanismBase(
    mechanism_config::BASE_RX_PIN,
    mechanism_config::BASE_TX_PIN);
HardwareSerial serialMechanismServo(
    mechanism_config::SERVO_RX_PIN,
    mechanism_config::SERVO_TX_PIN);
HardwareSerial serialTestDebug(
    debug_config::RX_PIN,
    debug_config::TX_PIN);

CenteredTestVision testVision;
ImmediateTestForwardPositioner testForwardPositioner;
MechanismTaskExecutor mechanism(
    serialMechanismStepper,
    serialMechanismBase,
    serialMechanismServo,
    testVision,
    testForwardPositioner);
OneButton testButton(
    mission_config::START_BUTTON_PIN,
    true,
    true);

const BatchMission TEST_BATCH = {
    {1, 2, 3},
    {1, 2, 3},
    {1, 2, 3}};

TestStage stage = TestStage::TravelPreparation;
bool actionActive = false;
bool travelAfterStationPending = false;
bool testFault = false;
uint32_t lastDebugMs = 0;

const char *stageName(TestStage value)
{
    switch (value)
    {
    case TestStage::TravelPreparation:
        return "travel";
    case TestStage::CollectMaterial:
        return "collect";
    case TestStage::RoughProcessing:
        return "rough";
    case TestStage::StoreFinishedProduct:
        return "storage";
    case TestStage::Completed:
        return "completed";
    }
    return "unknown";
}

void updateDebugOutput()
{
    const uint32_t now = millis();
    if (now - lastDebugMs < 500)
        return;
    lastDebugMs = now;

    serialTestDebug.print("MECH_TEST t=");
    serialTestDebug.print(now);
    serialTestDebug.print(" stage=");
    serialTestDebug.print(stageName(stage));
    serialTestDebug.print(" active=");
    serialTestDebug.print(actionActive ? 1 : 0);
    serialTestDebug.print(" ready=");
    serialTestDebug.print(mechanism.ready() ? 1 : 0);
    serialTestDebug.print(" result=");
    serialTestDebug.print(
        static_cast<uint8_t>(mechanism.result()));
    serialTestDebug.print(" phase=");
    serialTestDebug.print(mechanism.debugPhase());
    const MechanismTaskExecutor::MotionDebugState &motion =
        mechanism.motionDebug();
    serialTestDebug.print(" cmd=L");
    serialTestDebug.print(motion.liftCommandCount);
    serialTestDebug.print('@');
    serialTestDebug.print(motion.liftTarget, 0);
    serialTestDebug.print('/');
    serialTestDebug.print(motion.liftIssuedMs);
    serialTestDebug.print("/H");
    serialTestDebug.print(motion.liftHomeConfirmedMs);
    serialTestDebug.print(",B");
    serialTestDebug.print(motion.baseCommandCount);
    serialTestDebug.print('@');
    serialTestDebug.print(motion.baseTarget, 0);
    serialTestDebug.print('/');
    serialTestDebug.print(motion.baseIssuedMs);
    serialTestDebug.print(",E");
    serialTestDebug.print(motion.extensionCommandCount);
    serialTestDebug.print('@');
    serialTestDebug.print(motion.extensionTarget, 0);
    serialTestDebug.print('/');
    serialTestDebug.print(motion.extensionIssuedMs);
    serialTestDebug.print(" fault=");
    const char *fault = mechanism.faultMessage();
    serialTestDebug.println(
        fault != nullptr && fault[0] != '\0' ? fault : "-");
}

void startCurrentStage()
{
    if (actionActive ||
        testFault ||
        stage == TestStage::Completed ||
        !mechanism.ready() ||
        mechanism.result() == AsyncResult::Running)
    {
        return;
    }

    bool accepted = false;
    travelAfterStationPending = false;

    switch (stage)
    {
    case TestStage::TravelPreparation:
        accepted = mechanism.prepareForTravel(
            TravelDestination::Material);
        break;

    case TestStage::CollectMaterial:
        accepted = mechanism.start(
            StationTask::CollectMaterial,
            0,
            TEST_BATCH);
        travelAfterStationPending = accepted;
        break;

    case TestStage::RoughProcessing:
        accepted = mechanism.start(
            StationTask::RoughProcessing,
            0,
            TEST_BATCH);
        travelAfterStationPending = accepted;
        break;

    case TestStage::StoreFinishedProduct:
        accepted = mechanism.start(
            StationTask::StoreFinishedProduct,
            0,
            TEST_BATCH);
        travelAfterStationPending = accepted;
        break;

    case TestStage::Completed:
        return;
    }

    if (accepted)
        actionActive = true;
    else
        testFault = true;
}

void advanceStage()
{
    switch (stage)
    {
    case TestStage::TravelPreparation:
        stage = TestStage::CollectMaterial;
        break;
    case TestStage::CollectMaterial:
        stage = TestStage::RoughProcessing;
        break;
    case TestStage::RoughProcessing:
        stage = TestStage::StoreFinishedProduct;
        break;
    case TestStage::StoreFinishedProduct:
        stage = TestStage::Completed;
        break;
    case TestStage::Completed:
        break;
    }
}

void updateTestSequence()
{
    if (mechanism.result() == AsyncResult::Failed)
    {
        testFault = true;
        actionActive = false;
        return;
    }

    if (!actionActive ||
        mechanism.result() != AsyncResult::Succeeded)
    {
        return;
    }

    /*
     * 三个工位动作返回成功时，机械臂只保证物料已放稳且升降轴
     * 已抬高。这里立即追加运输收纳动作，单独验证比赛中的离场流程。
     */
    if (travelAfterStationPending)
    {
        travelAfterStationPending = false;
        TravelDestination destination = TravelDestination::Home;
        if (stage == TestStage::CollectMaterial)
            destination = TravelDestination::RoughProcessing;
        else if (stage == TestStage::RoughProcessing)
            destination = TravelDestination::Storage;

        if (!mechanism.prepareForTravel(destination))
        {
            testFault = true;
            actionActive = false;
        }
        return;
    }

    actionActive = false;
    advanceStage();
}

void updateStatusLed()
{
    const uint32_t now = millis();

    if (testFault)
    {
        // 快闪：测试或机构故障。
        digitalWrite(
            mission_config::STATUS_LED_PIN,
            (now / 150) % 2 ? HIGH : LOW);
    }
    else if (!mechanism.ready())
    {
        // 慢闪：上电初始化尚未完成。
        digitalWrite(
            mission_config::STATUS_LED_PIN,
            (now / 500) % 2 ? HIGH : LOW);
    }
    else if (actionActive)
    {
        // 常亮：当前测试动作正在运行。
        digitalWrite(mission_config::STATUS_LED_PIN, HIGH);
    }
    else if (stage == TestStage::Completed)
    {
        // 双倍慢闪：四项测试已经全部完成。
        digitalWrite(
            mission_config::STATUS_LED_PIN,
            (now / 1000) % 2 ? HIGH : LOW);
    }
    else
    {
        // 熄灭：等待按键启动下一项。
        digitalWrite(mission_config::STATUS_LED_PIN, LOW);
    }
}
} // namespace

void setup()
{
    delay(1000);

    pinMode(mission_config::STATUS_LED_PIN, OUTPUT);
    digitalWrite(mission_config::STATUS_LED_PIN, LOW);

    testButton.reset();
    testButton.attachClick(startCurrentStage);
    serialTestDebug.begin(debug_config::BAUD);
    serialTestDebug.println("MECH_TEST boot");
    mechanism.begin();
}

void loop()
{
    mechanism.update();
    testButton.tick();
    updateTestSequence();
    updateStatusLed();
    updateDebugOutput();
}
