#pragma once

#include <Arduino.h>

/**
 * @brief 比赛抽签确定的右侧启停区。
 *
 * 数值与串口屏 DEBUG.n4.val 保持一致，避免在界面和任务控制器之间
 * 再做一层容易出错的编号转换。
 */
enum class StartZone : uint8_t
{
    LowerRight = 0,
    UpperRight = 1
};
