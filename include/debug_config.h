#pragma once

#include <Arduino.h>

namespace debug_config
{
    // 调试输出选项：
    // - false: 使用独立USB-TTL (PB12/PB13)
    // - true: 使用USB CDC虚拟串口 (Serial, 通过DAP-Link)
    constexpr bool USE_USB_CDC = true;
    
    // 独立USB-TTL调试串口引脚（USE_USB_CDC=false时使用）
    constexpr uint32_t RX_PIN = PB12;
    constexpr uint32_t TX_PIN = PB13;
    constexpr uint32_t BAUD = 115200;

    // 低频输出，避免调试信息影响底盘和机构update()调用频率。
    constexpr uint32_t STARTUP_REPORT_INTERVAL_MS = 500;
} // namespace debug_config
