#pragma once

#include <Arduino.h>

namespace mission_config
{
// 一键启动与状态灯
constexpr uint32_t START_BUTTON_PIN = PB9;
constexpr uint32_t STATUS_LED_PIN = PA15;
constexpr uint32_t STARTUP_STABLE_MS = 5000;

// GM75扫码模块，接线与new_project保持一致。
constexpr uint32_t QR_RX_PIN = PE0;
constexpr uint32_t QR_TX_PIN = PE1;
constexpr uint32_t QR_BAUD = 9600;

// 淘晶驰/Nextion串口屏，接线与new_project保持一致。
constexpr uint32_t SCREEN_RX_PIN = PB15;
constexpr uint32_t SCREEN_TX_PIN = PB14;
constexpr uint32_t SCREEN_BAUD = 115200;
constexpr uint32_t SCREEN_RESTART_WAIT_MS = 500;

/*
 * 到达扫码区后开始计时。超时进入任务故障，避免扫码器断线时
 * 状态机永久停留在Scanning。
 */
constexpr uint32_t QR_TIMEOUT_MS = 30000;

} // namespace mission_config
