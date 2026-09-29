#include <Arduino.h>
#include <OneButton.h>

#include "ChassisControl.h"
#include "chassis_config.h"
#include "debug_config.h"
#include "mission_config.h"

/**
 * @brief 四方向距离标定测试程序
 * 
 * 使用方法：
 * 1. 上电等待 IMU 就绪，状态灯熄灭
 * 2. 在地面标记起点位置（建议用胶带标出底盘中心投影和车头方向）
 * 3. 单击按键循环选择方向：前 -> 后 -> 左 -> 右 -> 前...（状态灯闪烁次数表示）
 * 4. 长按按键 1 秒启动测试
 * 5. 底盘移动指定距离后自动停止
 * 6. 测量实际到达位置，记录偏差
 * 7. 根据串口输出的建议系数更新 chassis_config.h
 * 
 * 调试串口输出格式：
 * - CALIB_READY: 系统就绪，显示当前测试参数
 * - CALIB_SELECT: 当前选择的方向
 * - CALIB_START: 开始移动，记录起始航向
 * - CALIB_DONE: 完成移动，显示位姿变化和建议系数
 * - CALIB_HEADING_DRIFT: 航向偏移诊断
 */

namespace
{
    // 标定参数
    constexpr float TEST_DISTANCE_MM = 1000.0F;  // 目标距离
    constexpr float TEST_RPM = chassis_config::DRIVE_RPM;
    constexpr float TEST_ACCEL = chassis_config::DRIVE_ACCEL_RPM_PER_S;

    enum class Direction : uint8_t
    {
        Forward = 0,  // 前进
        Backward = 1, // 后退
        Left = 2,     // 左移
        Right = 3     // 右移
    };

    const char* directionName(Direction dir)
    {
        switch (dir)
        {
        case Direction::Forward:  return "FORWARD";
        case Direction::Backward: return "BACKWARD";
        case Direction::Left:     return "LEFT";
        case Direction::Right:    return "RIGHT";
        default:                  return "UNKNOWN";
        }
    }

    HardwareSerial serialImu(
        chassis_config::IMU_RX_PIN,
        chassis_config::IMU_TX_PIN);
    HardwareSerial serialDebug(
        debug_config::RX_PIN,
        debug_config::TX_PIN);

    ChassisControl chassis(&serialImu);
    OneButton startButton(
        mission_config::START_BUTTON_PIN,
        true,
        true);

    Direction selectedDirection = Direction::Forward;
    bool testRunning = false;
    bool readyReported = false;
    uint32_t testStartMs = 0;
    float startYawDeg = 0.0F;
    ChassisControl::Pose2D startPose = {0.0F, 0.0F, 0.0F};

    // LED 状态指示
    uint8_t ledBlinkCount = 0;
    uint32_t lastBlinkMs = 0;
    bool ledState = false;

    void blinkLed(uint8_t count)
    {
        ledBlinkCount = count * 2;  // 亮灭各一次
        lastBlinkMs = millis();
        ledState = false;
        digitalWrite(mission_config::STATUS_LED_PIN, LOW);
    }

    void updateLedBlink()
    {
        if (ledBlinkCount == 0)
            return;

        const uint32_t now = millis();
        if (now - lastBlinkMs >= 150)  // 150ms 周期
        {
            lastBlinkMs = now;
            ledState = !ledState;
            digitalWrite(mission_config::STATUS_LED_PIN, ledState ? HIGH : LOW);
            if (!ledState)
                --ledBlinkCount;
        }
    }

    void onButtonClick()
    {
        if (testRunning)
            return;

        // 循环选择方向
        uint8_t next = static_cast<uint8_t>(selectedDirection) + 1;
        if (next > 3)
            next = 0;
        selectedDirection = static_cast<Direction>(next);

        serialDebug.print("CALIB_SELECT direction=");
        serialDebug.print(directionName(selectedDirection));
        serialDebug.print(" (");
        serialDebug.print(next + 1);
        serialDebug.println("/4)");

        // 闪烁次数表示方向：前1次 后2次 左3次 右4次
        blinkLed(next + 1);
    }

    void onButtonLongPress()
    {
        if (testRunning || !chassis.imuReady())
            return;

        // 记录起始状态
        startPose = chassis.worldPose();
        startYawDeg = chassis.yawDeg();
        testStartMs = millis();

        // 下发移动指令
        float forwardMm = 0.0F;
        float rightMm = 0.0F;

        switch (selectedDirection)
        {
        case Direction::Forward:
            forwardMm = TEST_DISTANCE_MM;
            break;
        case Direction::Backward:
            forwardMm = -TEST_DISTANCE_MM;
            break;
        case Direction::Left:
            rightMm = -TEST_DISTANCE_MM;
            break;
        case Direction::Right:
            rightMm = TEST_DISTANCE_MM;
            break;
        }

        if (!chassis.moveRelative(forwardMm, rightMm, TEST_RPM, TEST_ACCEL))
        {
            serialDebug.print("CALIB_START_FAILED fault=");
            serialDebug.println(chassis.faultMessage());
            blinkLed(10);  // 快速闪烁表示错误
            return;
        }

        testRunning = true;
        digitalWrite(mission_config::STATUS_LED_PIN, HIGH);

        serialDebug.print("CALIB_START direction=");
        serialDebug.print(directionName(selectedDirection));
        serialDebug.print(" target=");
        serialDebug.print(TEST_DISTANCE_MM, 0);
        serialDebug.print("mm startYaw=");
        serialDebug.print(startYawDeg, 2);
        serialDebug.print(" startPose=");
        serialDebug.print(startPose.xMm, 1);
        serialDebug.print(",");
        serialDebug.print(startPose.yMm, 1);
        serialDebug.print(",");
        serialDebug.println(startPose.yawDeg, 2);
    }

    void finishTest()
    {
        testRunning = false;
        digitalWrite(mission_config::STATUS_LED_PIN, LOW);

        const uint32_t elapsedMs = millis() - testStartMs;
        const ChassisControl::Pose2D endPose = chassis.worldPose();
        const float endYawDeg = chassis.yawDeg();

        // 检查故障
        if (chassis.state() == ChassisControl::State::Fault)
        {
            serialDebug.print("CALIB_FAULT ");
            serialDebug.println(chassis.faultMessage());
            blinkLed(10);
            return;
        }

        // 计算实际位移
        const float deltaX = endPose.xMm - startPose.xMm;
        const float deltaY = endPose.yMm - startPose.yMm;
        const float actualDistance = sqrtf(deltaX * deltaX + deltaY * deltaY);
        
        // 计算航向偏移
        float yawDrift = endYawDeg - startYawDeg;
        if (yawDrift > 180.0F)
            yawDrift -= 360.0F;
        else if (yawDrift < -180.0F)
            yawDrift += 360.0F;

        // 输出结果
        serialDebug.println("========================================");
        serialDebug.print("CALIB_DONE direction=");
        serialDebug.println(directionName(selectedDirection));
        serialDebug.print("  Elapsed: ");
        serialDebug.print(elapsedMs);
        serialDebug.println(" ms");
        
        serialDebug.print("  Target distance: ");
        serialDebug.print(TEST_DISTANCE_MM, 0);
        serialDebug.println(" mm");
        
        serialDebug.print("  Odometry distance: ");
        serialDebug.print(actualDistance, 1);
        serialDebug.println(" mm");
        
        serialDebug.print("  Delta X: ");
        serialDebug.print(deltaX, 1);
        serialDebug.print(" mm, Delta Y: ");
        serialDebug.print(deltaY, 1);
        serialDebug.println(" mm");

        // 计算建议的标定系数
        float currentScale = 1.0F;
        const char* scaleConfigName = nullptr;

        switch (selectedDirection)
        {
        case Direction::Forward:
            currentScale = chassis_config::FORWARD_DISTANCE_SCALE;
            scaleConfigName = "FORWARD_DISTANCE_SCALE";
            break;
        case Direction::Backward:
            currentScale = chassis_config::BACKWARD_DISTANCE_SCALE;
            scaleConfigName = "BACKWARD_DISTANCE_SCALE";
            break;
        case Direction::Left:
            currentScale = chassis_config::LEFT_DISTANCE_SCALE;
            scaleConfigName = "LEFT_DISTANCE_SCALE";
            break;
        case Direction::Right:
            currentScale = chassis_config::RIGHT_DISTANCE_SCALE;
            scaleConfigName = "RIGHT_DISTANCE_SCALE";
            break;
        }

        serialDebug.println("  ---");
        serialDebug.print("  Current ");
        serialDebug.print(scaleConfigName);
        serialDebug.print(": ");
        serialDebug.println(currentScale, 4);

        // 测量实际距离并计算建议系数
        serialDebug.println("  ---");
        serialDebug.println("  [ACTION REQUIRED]");
        serialDebug.println("  1. Measure ACTUAL distance traveled from start mark");
        serialDebug.println("  2. Calculate new scale = 1000.0 / measured_distance");
        serialDebug.print("     If odometry (");
        serialDebug.print(actualDistance, 1);
        serialDebug.println(" mm) is accurate:");
        
        if (fabsf(actualDistance - TEST_DISTANCE_MM) > 1.0F)
        {
            const float suggestedScale = currentScale * TEST_DISTANCE_MM / actualDistance;
            serialDebug.print("     Suggested ");
            serialDebug.print(scaleConfigName);
            serialDebug.print(" = ");
            serialDebug.println(suggestedScale, 4);
        }
        else
        {
            serialDebug.println("     Distance accurate, no adjustment needed");
        }

        // 航向偏移诊断
        serialDebug.println("  ---");
        serialDebug.print("  Heading drift: ");
        serialDebug.print(yawDrift, 2);
        serialDebug.println(" degrees");

        if (fabsf(yawDrift) > 2.0F)
        {
            serialDebug.println("  [WARNING] Significant heading drift detected!");
            serialDebug.println("  Possible causes:");
            serialDebug.println("  - Wheel diameter mismatch between left/right");
            serialDebug.println("  - Motor direction signs need adjustment");
            serialDebug.println("  - Uneven floor or wheel slippage");
            serialDebug.println("  - IMU mounting not level");
            
            switch (selectedDirection)
            {
            case Direction::Forward:
            case Direction::Backward:
                if (yawDrift > 0)
                    serialDebug.println("  -> Drifting LEFT (counterclockwise)");
                else
                    serialDebug.println("  -> Drifting RIGHT (clockwise)");
                serialDebug.println("  Check: left vs right wheel diameter/friction");
                break;
            case Direction::Left:
            case Direction::Right:
                serialDebug.println("  Lateral movement should not cause yaw drift");
                serialDebug.println("  Check: mecanum wheel orientation");
                break;
            }
        }
        else
        {
            serialDebug.println("  Heading drift within acceptable range");
        }

        serialDebug.println("========================================");
        serialDebug.println();

        // 短暂闪烁表示完成
        blinkLed(2);
    }

    void printReady()
    {
        serialDebug.println("========================================");
        serialDebug.println("Distance Calibration Test Ready");
        serialDebug.println("========================================");
        serialDebug.print("Test distance: ");
        serialDebug.print(TEST_DISTANCE_MM, 0);
        serialDebug.println(" mm");
        serialDebug.print("Max RPM: ");
        serialDebug.println(TEST_RPM, 0);
        serialDebug.print("Acceleration: ");
        serialDebug.print(TEST_ACCEL, 0);
        serialDebug.println(" RPM/s");
        serialDebug.print("Microsteps: ");
        serialDebug.println(chassis_config::MICROSTEPS);
        serialDebug.print("Wheel diameter: ");
        serialDebug.print(chassis_config::WHEEL_DIAMETER_MM, 1);
        serialDebug.println(" mm");
        serialDebug.println("---");
        serialDebug.println("Current scales:");
        serialDebug.print("  FORWARD:  ");
        serialDebug.println(chassis_config::FORWARD_DISTANCE_SCALE, 4);
        serialDebug.print("  BACKWARD: ");
        serialDebug.println(chassis_config::BACKWARD_DISTANCE_SCALE, 4);
        serialDebug.print("  LEFT:     ");
        serialDebug.println(chassis_config::LEFT_DISTANCE_SCALE, 4);
        serialDebug.print("  RIGHT:    ");
        serialDebug.println(chassis_config::RIGHT_DISTANCE_SCALE, 4);
        serialDebug.println("---");
        serialDebug.println("Controls:");
        serialDebug.println("  Short click: Cycle direction (LED blinks show choice)");
        serialDebug.println("  Long press:  Start test");
        serialDebug.println("========================================");
        serialDebug.print("Selected: ");
        serialDebug.println(directionName(selectedDirection));
        serialDebug.println();

        blinkLed(1);  // 单次闪烁表示就绪
    }

} // namespace

void setup()
{
    delay(1000);

    pinMode(mission_config::STATUS_LED_PIN, OUTPUT);
    digitalWrite(mission_config::STATUS_LED_PIN, HIGH);

    serialDebug.begin(debug_config::BAUD);
    serialDebug.println("CALIB_BOOT Waiting for IMU...");

    chassis.begin();
    chassis.resetWorldPose(0.0F, 0.0F, 0.0F);

    startButton.reset();
    startButton.attachClick(onButtonClick);
    startButton.attachLongPressStart(onButtonLongPress);
    startButton.setPressTicks(1000);  // 长按 1 秒

    serialDebug.println("CALIB_BOOT System initialized");
}

void loop()
{
    startButton.tick();
    chassis.update();
    updateLedBlink();

    if (!readyReported && chassis.imuReady())
    {
        readyReported = true;
        digitalWrite(mission_config::STATUS_LED_PIN, LOW);
        printReady();
    }

    if (testRunning && !chassis.busy())
    {
        finishTest();
    }

    // 每 2 秒输出一次 IMU 状态（仅在空闲时）
    static uint32_t lastStatusMs = 0;
    if (!testRunning && 
        chassis.imuReady() && 
        millis() - lastStatusMs >= 2000)
    {
        lastStatusMs = millis();
        serialDebug.print("STATUS yaw=");
        serialDebug.print(chassis.yawDeg(), 2);
        serialDebug.print(" pose=");
        const ChassisControl::Pose2D pose = chassis.worldPose();
        serialDebug.print(pose.xMm, 1);
        serialDebug.print(",");
        serialDebug.print(pose.yMm, 1);
        serialDebug.print(",");
        serialDebug.println(pose.yawDeg, 2);
    }
}
