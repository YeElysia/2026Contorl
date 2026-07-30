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
 *   forwardDelta   = FORWARD_FROM_DX * dx + FORWARD_FROM_DY * dy
 *   extensionDelta = EXT_FROM_DX     * dx + EXT_FROM_DY     * dy
 *
 * 底盘前后单位为mm，伸缩单位为0.1mm。底盘只允许前后移动，
 * 朝转盘方向的误差全部由伸缩轴补偿。交叉项可用于补偿相机倾斜。
 */
constexpr float FORWARD_FROM_DX = -0.2F;
constexpr float FORWARD_FROM_DY = 0.0F;
constexpr float EXTENSION_FROM_DX = 0.0F;
constexpr float EXTENSION_FROM_DY = 3.0F;

// 远离中心时直接大步到达，进入精调区后限制单次修正量。
constexpr int16_t FINE_ALIGNMENT_ZONE_PX = 24;
constexpr float COARSE_FORWARD_MAX_DELTA_MM = 15.0F;
constexpr float COARSE_EXTENSION_MAX_DELTA = 300.0F;
constexpr float FINE_FORWARD_MAX_DELTA_MM = 3.0F;
constexpr float FINE_EXTENSION_MAX_DELTA = 30.0F;

/*
 * 视觉闭环软限位只约束原料区跟随动作。
 * 不能把机械硬限位直接当作软件目标。
 */
constexpr float PICKUP_FORWARD_MIN_OFFSET_MM = -50.0F;
constexpr float PICKUP_FORWARD_MAX_OFFSET_MM = 50.0F;
constexpr float PICKUP_EXTENSION_MIN = 300.0F;
constexpr float PICKUP_EXTENSION_MAX = 1400.0F;

// -------------------- 粗加工区圆环整车对准 --------------------
// 参考new_project的圆环3定位，后续可按相机视野调整圆环编号。
constexpr uint8_t ROUGH_RING_ID = 3;
constexpr int16_t RING_TARGET_DX_PX = 0;
constexpr int16_t RING_TARGET_DY_PX = 0;
constexpr int16_t RING_CENTER_TOLERANCE_PX = 10;
constexpr int16_t RING_FINE_ALIGNMENT_ZONE_PX = 25;
constexpr uint8_t RING_REQUIRED_STABLE_FRAMES = 3;
constexpr uint8_t RING_MIN_QUALITY = 30;

// 相机误差到车体坐标：forward>0前进，right>0右移。
constexpr float RING_FORWARD_MM_PER_DY_PX = 0.8F;
constexpr float RING_RIGHT_MM_PER_DX_PX = -0.8F;
constexpr float RING_COARSE_MAX_MOVE_MM = 30.0F;
constexpr float RING_FINE_MAX_MOVE_MM = 10.0F;
constexpr uint32_t RING_TARGET_STALE_MS = 500;
constexpr uint32_t RING_ALIGNMENT_TIMEOUT_MS = 12000;
} // namespace vision_config
