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
constexpr uint8_t REQUIRED_STABLE_FRAMES = 3;
constexpr uint8_t MIN_QUALITY = 30;

constexpr uint32_t TARGET_STALE_MS = 500;
constexpr uint32_t TARGET_SEARCH_TIMEOUT_MS = 20000;

/*
 * 图像误差到机构修正量的二维标定矩阵：
 *
 *   baseDelta      = BASE_FROM_DX * dx + BASE_FROM_DY * dy
 *   extensionDelta = EXT_FROM_DX  * dx + EXT_FROM_DY  * dy
 *
 * 底座单位为0.1°，伸缩单位为0.1mm。交叉项初始为0，实车采集
 * 标定点后可用于补偿相机倾斜和两个方向之间的机械耦合。
 */
constexpr float BASE_FROM_DX = -0.5F;
constexpr float BASE_FROM_DY = 0.0F;
constexpr float EXTENSION_FROM_DX = 0.0F;
constexpr float EXTENSION_FROM_DY = 3.0F;

// 远离中心时直接大步到达，进入精调区后限制单次修正量。
constexpr int16_t FINE_ALIGNMENT_ZONE_PX = 24;
constexpr float COARSE_BASE_MAX_DELTA = 80.0F;
constexpr float COARSE_EXTENSION_MAX_DELTA = 300.0F;
constexpr float FINE_BASE_MAX_DELTA = 10.0F;
constexpr float FINE_EXTENSION_MAX_DELTA = 30.0F;

/*
 * 视觉闭环软限位只约束原料区跟随动作。
 * 不能把机械硬限位直接当作软件目标。
 */
constexpr float PICKUP_BASE_MIN = 1400.0F;
constexpr float PICKUP_BASE_MAX = 2200.0F;
constexpr float PICKUP_EXTENSION_MIN = 300.0F;
constexpr float PICKUP_EXTENSION_MAX = 1400.0F;
} // namespace vision_config
