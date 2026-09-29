#include "DebugLog.h"
#include <stdarg.h>
#include <stdio.h>

namespace DebugLog
{
    Level globalLevel = Level::INFO;
    HardwareSerial *serialPort = nullptr;

    void begin(HardwareSerial *serial, uint32_t baud, Level level)
    {
        serialPort = serial;
        globalLevel = level;
        serial->begin(baud);
        delay(100);
        
        serialPort->println("\n\n========================================");
        serialPort->println("Debug Log System Initialized");
        serialPort->print("Log Level: ");
        switch(level) {
            case Level::ERROR: serialPort->println("ERROR"); break;
            case Level::WARNING: serialPort->println("WARNING"); break;
            case Level::INFO: serialPort->println("INFO"); break;
            case Level::DEBUG: serialPort->println("DEBUG"); break;
            case Level::TRACE: serialPort->println("TRACE"); break;
        }
        serialPort->print("Time: ");
        serialPort->print(millis());
        serialPort->println(" ms");
        serialPort->println("========================================\n");
    }

    void setLevel(Level level)
    {
        globalLevel = level;
    }

    const char* getLevelString(Level level)
    {
        switch(level) {
            case Level::ERROR: return "[ERROR]";
            case Level::WARNING: return "[WARN ]";
            case Level::INFO: return "[INFO ]";
            case Level::DEBUG: return "[DEBUG]";
            case Level::TRACE: return "[TRACE]";
            default: return "[?????]";
        }
    }

    void log(Level level, const char *tag, const char *message)
    {
        if (!serialPort || level > globalLevel)
            return;

        char buffer[256];
        snprintf(buffer, sizeof(buffer), "[%010lu] %s [%s] %s",
                 millis(), getLevelString(level), tag, message);
        serialPort->println(buffer);
    }

    void logf(Level level, const char *tag, const char *format, ...)
    {
        if (!serialPort || level > globalLevel)
            return;

        char message[192];
        va_list args;
        va_start(args, format);
        vsnprintf(message, sizeof(message), format, args);
        va_end(args);

        log(level, tag, message);
    }

    void logChassisMove(float x, float y, float yaw, const char *type)
    {
        if (!serialPort || Level::INFO > globalLevel)
            return;

        char buffer[128];
        snprintf(buffer, sizeof(buffer),
                 "CHASSIS_MOVE type=%s target=(%.1f, %.1f, %.1f°)",
                 type, x, y, yaw);
        log(Level::INFO, "Chassis", buffer);
    }

    void logChassisState(const char *state, float x, float y, float yaw)
    {
        if (!serialPort || Level::DEBUG > globalLevel)
            return;

        char buffer[128];
        snprintf(buffer, sizeof(buffer),
                 "CHASSIS_STATE state=%s pose=(%.1f, %.1f, %.1f°)",
                 state, x, y, yaw);
        log(Level::DEBUG, "Chassis", buffer);
    }

    void logVisionAlignment(const char *station, bool success, float dx, float dy)
    {
        if (!serialPort || Level::INFO > globalLevel)
            return;

        char buffer[128];
        snprintf(buffer, sizeof(buffer),
                 "VISION_ALIGN station=%s result=%s error=(%.1f, %.1f)px",
                 station, success ? "SUCCESS" : "FAILED", dx, dy);
        log(success ? Level::INFO : Level::WARNING, "Vision", buffer);
    }

    void logMissionState(const char *state)
    {
        if (!serialPort || Level::INFO > globalLevel)
            return;

        char buffer[64];
        snprintf(buffer, sizeof(buffer), "MISSION_STATE=%s", state);
        log(Level::INFO, "Mission", buffer);
    }

    void logRouteAction(const char *action, int index, int total)
    {
        if (!serialPort || Level::INFO > globalLevel)
            return;

        char buffer[64];
        snprintf(buffer, sizeof(buffer), "ROUTE_ACTION [%d/%d] %s",
                 index + 1, total, action);
        log(Level::INFO, "Route", buffer);
    }

} // namespace DebugLog
