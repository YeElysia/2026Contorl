#pragma once

#include <Arduino.h>

namespace debug_config
{
    // 独立USB-TTL调试串口，与new_project保持一致。
    constexpr uint32_t RX_PIN = PB12;
    constexpr uint32_t TX_PIN = PB13;
    constexpr uint32_t BAUD = 115200;

    // 低频输出，避免调试信息影响底盘和机构update()调用频率。
    constexpr uint32_t STARTUP_REPORT_INTERVAL_MS = 500;
} // namespace debug_config
