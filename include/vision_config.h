#pragma once

#include <Arduino.h>

/**
 * @brief MaixPro视觉通信和原料抓取对准参数。
 *
 * 串口与协议参数以new_project为准；闭环初值来自其
 * test_maix_v2.cpp，实车联调时只修改本文件。
 */
namespace vision_config
{
constexpr uint32_t RX_PIN = PE7;
constexpr uint32_t TX_PIN = PE8;
constexpr uint32_t BAUD = 115200;

constexpr int16_t TARGET_DX_PX = 10;
constexpr int16_t TARGET_DY_PX = -25;
constexpr int16_t CENTER_TOLERANCE_PX = 8;
constexpr uint8_t REQUIRED_STABLE_FRAMES = 5;
constexpr uint8_t MIN_QUALITY = 30;

constexpr uint32_t ALIGN_INTERVAL_MS = 150;
constexpr uint32_t TARGET_STALE_MS = 500;
constexpr uint32_t TARGET_SEARCH_TIMEOUT_MS = 20000;

// 底座单位为0.1°，伸缩单位为0.1mm。
constexpr float BASE_UNITS_PER_PIXEL = 0.5F;
constexpr float EXTENSION_UNITS_PER_PIXEL = 3.0F;
constexpr float BASE_MAX_DELTA = 10.0F;
constexpr float EXTENSION_MAX_DELTA = 30.0F;

/*
 * 视觉闭环软限位只约束原料区跟随动作。
 * 不能把机械硬限位直接当作软件目标。
 */
constexpr float PICKUP_BASE_MIN = 1400.0F;
constexpr float PICKUP_BASE_MAX = 2200.0F;
constexpr float PICKUP_EXTENSION_MIN = 300.0F;
constexpr float PICKUP_EXTENSION_MAX = 1400.0F;
} // namespace vision_config
