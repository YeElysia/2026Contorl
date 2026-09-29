#include <Arduino.h>
#include "ChassisEmm42TtlFeedback.h"
#include "chassis_config.h"
#include "debug_config.h"

/**
 * @brief EMM42 V5 TTL反馈测试程序
 * 
 * 功能：
 * 1. 轮询4个电机的TTL反馈状态
 * 2. 显示每个电机的：使能、到位、堵转、编码器位置
 * 3. 检测通信超时和地址冲突
 * 4. 每1秒输出一次完整状态报告
 * 
 * 接线要求：
 * - 4个电机TTL线并联连接到PA9(TX)/PA10(RX)
 * - 4个电机TTL地址必须不同且非零（默认1-4）
 * - 调试串口：PB12/PB13，波特率115200
 * 
 * 预期结果：
 * - 每个电机应显示 statusValid=1, positionValid=1
 * - enabled状态取决于电机当前是否使能
 * - encoderPosition会实时更新（手动转动轮子时变化）
 * - 如果某个电机通信失败，会显示对应错误
 */

namespace
{
    HardwareSerial serialMotorTtl(
        chassis_config::MOTOR_TTL_RX_PIN,
        chassis_config::MOTOR_TTL_TX_PIN);
    HardwareSerial serialDebug(
        debug_config::RX_PIN,
        debug_config::TX_PIN);

    ChassisEmm42TtlFeedback motorFeedback(
        serialMotorTtl,
        chassis_config::MOTOR_TTL_ADDRESSES,
        chassis_config::MOTOR_TTL_BAUD);

    uint32_t lastReportMs = 0;
    constexpr uint32_t REPORT_INTERVAL_MS = 1000;

    void printStatus()
    {
        serialDebug.println("========================================");
        serialDebug.println("EMM42 TTL Feedback Test");
        serialDebug.print("Time: ");
        serialDebug.print(millis() / 1000);
        serialDebug.println(" s");
        serialDebug.println("----------------------------------------");

        if (!motorFeedback.ready())
        {
            serialDebug.println("⚠️  Motor feedback NOT READY");
            if (motorFeedback.faultMessage()[0] != '\0')
            {
                serialDebug.print("Fault: ");
                serialDebug.println(motorFeedback.faultMessage());
            }
        }
        else if (!motorFeedback.healthy())
        {
            serialDebug.println("⚠️  Motor feedback UNHEALTHY");
            serialDebug.print("Fault: ");
            serialDebug.println(motorFeedback.faultMessage());
        }
        else
        {
            serialDebug.println("✅ Motor feedback HEALTHY");
        }

        serialDebug.println("----------------------------------------");

        const char* motorNames[4] = {
            "Left Front  (M1)",
            "Right Front (M2)",
            "Left Rear   (M3)",
            "Right Rear  (M4)"
        };

        for (uint8_t i = 0; i < 4; ++i)
        {
            const auto& motor = motorFeedback.motor(i);
            serialDebug.print("Motor ");
            serialDebug.print(i);
            serialDebug.print(" - ");
            serialDebug.print(motorNames[i]);
            serialDebug.print(" [Addr ");
            serialDebug.print(chassis_config::MOTOR_TTL_ADDRESSES[i]);
            serialDebug.println("]");

            serialDebug.print("  Status:   ");
            serialDebug.print(motor.statusValid ? "✓" : "✗");
            serialDebug.print("  Position: ");
            serialDebug.println(motor.positionValid ? "✓" : "✗");

            if (motor.statusValid)
            {
                serialDebug.print("  Enabled:  ");
                serialDebug.print(motor.enabled ? "YES" : "NO ");
                serialDebug.print("  InPos: ");
                serialDebug.print(motor.inPosition ? "YES" : "NO ");
                serialDebug.print("  Stalled: ");
                serialDebug.println(motor.stalled ? "YES" : "NO ");

                if (motor.stallProtected)
                {
                    serialDebug.println("  ⚠️  STALL PROTECTION TRIGGERED!");
                }

                serialDebug.print("  Last status: ");
                serialDebug.print(millis() - motor.lastStatusMs);
                serialDebug.println(" ms ago");
            }

            if (motor.positionValid)
            {
                serialDebug.print("  Encoder: ");
                serialDebug.print((int32_t)(motor.encoderPosition & 0xFFFFFFFF));
                serialDebug.print("  Last position: ");
                serialDebug.print(millis() - motor.lastPositionMs);
                serialDebug.println(" ms ago");
            }

            if (!motor.statusValid && !motor.positionValid)
            {
                serialDebug.println("  ❌ NO COMMUNICATION");
            }

            serialDebug.println();
        }

        serialDebug.println("========================================");
        serialDebug.println();
    }
}

void setup()
{
    delay(2000);

    serialDebug.begin(debug_config::BAUD);
    serialDebug.println("\n\n");
    serialDebug.println("========================================");
    serialDebug.println("EMM42 V5 TTL Feedback Test");
    serialDebug.println("========================================");
    serialDebug.println("Initializing...");
    serialDebug.println();
    serialDebug.print("TTL Serial: PA");
    serialDebug.print(chassis_config::MOTOR_TTL_TX_PIN & 0xFF);
    serialDebug.print(" (TX) / PA");
    serialDebug.print(chassis_config::MOTOR_TTL_RX_PIN & 0xFF);
    serialDebug.print(" (RX) @ ");
    serialDebug.print(chassis_config::MOTOR_TTL_BAUD);
    serialDebug.println(" baud");
    
    serialDebug.print("Motor addresses: ");
    for (uint8_t i = 0; i < 4; ++i)
    {
        if (i > 0) serialDebug.print(", ");
        serialDebug.print(chassis_config::MOTOR_TTL_ADDRESSES[i]);
    }
    serialDebug.println();
    serialDebug.println();

    motorFeedback.begin();
    
    serialDebug.println("Initialization complete.");
    serialDebug.println("Waiting for first data...");
    serialDebug.println("(It may take 1-2 seconds to query all motors)");
    serialDebug.println();

    lastReportMs = millis();
}

void loop()
{
    motorFeedback.update();

    const uint32_t now = millis();
    if (now - lastReportMs >= REPORT_INTERVAL_MS)
    {
        lastReportMs = now;
        printStatus();
    }
}
