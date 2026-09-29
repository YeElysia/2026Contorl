#pragma once

#include <Arduino.h>

/**
 * @brief 调试日志模块
 * 
 * 提供分级日志输出，方便分析整个任务流程。
 * 使用宏定义避免发布版本中包含调试代码。
 */

namespace DebugLog
{
    enum class Level : uint8_t
    {
        ERROR = 0,   // 错误（红色）
        WARNING = 1, // 警告（黄色）
        INFO = 2,    // 信息（白色）
        DEBUG = 3,   // 调试（灰色）
        TRACE = 4    // 跟踪（最详细）
    };

    // 全局日志级别（只输出 <= 此级别的日志）
    extern Level globalLevel;
    extern HardwareSerial *serialPort;

    void begin(HardwareSerial *serial, uint32_t baud, Level level = Level::INFO);
    void setLevel(Level level);

    // 基础日志函数
    void log(Level level, const char *tag, const char *message);
    void logf(Level level, const char *tag, const char *format, ...);

    // 便捷宏
#define LOG_ERROR(tag, msg) DebugLog::log(DebugLog::Level::ERROR, tag, msg)
#define LOG_WARNING(tag, msg) DebugLog::log(DebugLog::Level::WARNING, tag, msg)
#define LOG_INFO(tag, msg) DebugLog::log(DebugLog::Level::INFO, tag, msg)
#define LOG_DEBUG(tag, msg) DebugLog::log(DebugLog::Level::DEBUG, tag, msg)
#define LOG_TRACE(tag, msg) DebugLog::log(DebugLog::Level::TRACE, tag, msg)

    // 格式化日志宏
#define LOGF_ERROR(tag, fmt, ...) DebugLog::logf(DebugLog::Level::ERROR, tag, fmt, ##__VA_ARGS__)
#define LOGF_WARNING(tag, fmt, ...) DebugLog::logf(DebugLog::Level::WARNING, tag, fmt, ##__VA_ARGS__)
#define LOGF_INFO(tag, fmt, ...) DebugLog::logf(DebugLog::Level::INFO, tag, fmt, ##__VA_ARGS__)
#define LOGF_DEBUG(tag, fmt, ...) DebugLog::logf(DebugLog::Level::DEBUG, tag, fmt, ##__VA_ARGS__)
#define LOGF_TRACE(tag, fmt, ...) DebugLog::logf(DebugLog::Level::TRACE, tag, fmt, ##__VA_ARGS__)

    // 特定模块日志
    void logChassisMove(float x, float y, float yaw, const char *type);
    void logChassisState(const char *state, float x, float y, float yaw);
    void logVisionAlignment(const char *station, bool success, float dx, float dy);
    void logMissionState(const char *state);
    void logRouteAction(const char *action, int index, int total);

} // namespace DebugLog
