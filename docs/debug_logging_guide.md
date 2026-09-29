# 串口日志记录系统使用指南

## 概述

提供自动串口日志记录工具和代码内调试日志系统，方便分析整个任务流程。

---

## 一、串口日志记录工具

### 功能特性

- ✅ 自动读取串口数据并保存到带时间戳的日志文件
- ✅ 实时彩色终端显示（错误/警告/成功高亮）
- ✅ 关键词自动识别和统计
- ✅ 支持macOS/Windows/Linux

### 安装依赖

```bash
# 安装pyserial
pip3 install pyserial
```

### 使用方法

#### 1. 列出所有可用串口

```bash
python3 tools/serial_logger.py --list
```

#### 2. 开始记录日志

**macOS:**
```bash
python3 tools/serial_logger.py -p /dev/cu.usbserial-1
```

**Windows:**
```bash
python tools/serial_logger.py -p COM3
```

**Linux:**
```bash
python3 tools/serial_logger.py -p /dev/ttyUSB0
```

#### 3. 自定义参数

```bash
# 自定义波特率
python3 tools/serial_logger.py -p COM3 -b 9600

# 自定义日志目录
python3 tools/serial_logger.py -p COM3 -o my_logs

# 完整示例
python3 tools/serial_logger.py -p /dev/cu.usbserial-1 -b 115200 -o logs/test1
```

#### 4. 停止记录

按 `Ctrl+C` 停止，工具会自动：
- 保存日志文件
- 关闭串口
- 显示统计信息

### 日志文件格式

```
================================================================================
串口日志记录
================================================================================
时间: 2025-01-10 14:30:25
串口: /dev/cu.usbserial-1
波特率: 115200
================================================================================

[14:30:26.123] [0000004123] [INFO ] [System] === 2026 GCDS Control System Starting ===
[14:30:26.234] [0000004234] [INFO ] [Init] Initializing chassis...
[14:30:27.456] [0000005456] [INFO ] [Chassis] MOVE_START forward=1000.0mm right=0.0mm yaw=0.0° speed=800RPM
[14:30:29.789] [0000007789] [INFO ] [Chassis] MOVE_COMPLETE pose=(1000.0, 0.0, 0.2°) duration=2333ms
```

### 彩色输出

终端实时显示会自动高亮：
- 🔴 **红色**: ERROR, FAULT, FAIL
- 🟡 **黄色**: WARNING, CAUTION
- 🟢 **绿色**: SUCCESS, COMPLETE, DONE
- 🔵 **蓝色**: STATE, STATUS, POSE
- 🟣 **紫色**: VISION, CAMERA, ALIGN
- 🔷 **青色**: MOVE, ROTATE, CHASSIS

### 统计信息

停止记录后自动显示：
```
============================================================
统计信息
============================================================
总行数: 1234
错误: 2
警告: 5
运动: 156
视觉: 23
============================================================
```

---

## 二、代码内调试日志系统

### 日志级别

```cpp
enum class Level {
    ERROR = 0,   // 错误（严重问题）
    WARNING = 1, // 警告（潜在问题）
    INFO = 2,    // 信息（关键事件）- 默认
    DEBUG = 3,   // 调试（详细信息）
    TRACE = 4    // 跟踪（最详细）
};
```

### 初始化

在 `main.cpp` 的 `setup()` 中：

```cpp
#include "DebugLog.h"

void setup() {
    // 初始化日志系统，设置级别为INFO
    DebugLog::begin(&serialDebug, 115200, DebugLog::Level::INFO);
    
    LOG_INFO("System", "System starting...");
}
```

### 基础使用

```cpp
// 简单日志
LOG_ERROR("Module", "Something went wrong");
LOG_WARNING("Module", "Potential issue");
LOG_INFO("Module", "Key event");
LOG_DEBUG("Module", "Debug info");
LOG_TRACE("Module", "Detailed trace");

// 格式化日志
LOGF_INFO("Chassis", "Moving to (%.1f, %.1f)", x, y);
LOGF_DEBUG("Sensor", "IMU yaw=%.2f° rate=%.2f°/s", yaw, rate);
```

### 专用日志函数

```cpp
// 底盘运动
DebugLog::logChassisMove(1000.0, 500.0, 45.0, "WorldTo");
// 输出: CHASSIS_MOVE type=WorldTo target=(1000.0, 500.0, 45.0°)

// 底盘状态
DebugLog::logChassisState("Translating", x, y, yaw);
// 输出: CHASSIS_STATE state=Translating pose=(1000.0, 500.0, 45.0°)

// 视觉对位
DebugLog::logVisionAlignment("RoughProcessing", true, 2.3, -1.5);
// 输出: VISION_ALIGN station=RoughProcessing result=SUCCESS error=(2.3, -1.5)px

// 任务状态
DebugLog::logMissionState("MovingToMaterial");
// 输出: MISSION_STATE=MovingToMaterial

// 路径动作
DebugLog::logRouteAction("MoveTo(1200,300)", 3, 10);
// 输出: ROUTE_ACTION [4/10] MoveTo(1200,300)
```

### 动态调整日志级别

```cpp
// 运行时修改日志级别
DebugLog::setLevel(DebugLog::Level::DEBUG);  // 显示更多细节
DebugLog::setLevel(DebugLog::Level::WARNING); // 只显示警告和错误
```

---

## 三、已集成的日志点

### 主程序 (`main.cpp`)
- ✅ 系统启动
- ✅ 各模块初始化（底盘、视觉、机械臂等）
- ✅ 初始化完成

### 底盘控制 (`ChassisControl`)
- ✅ 运动开始 (`MOVE_START`)
  - 车体坐标前进/右移距离
  - 保持航向
  - 运动速度
- ✅ 运动完成 (`MOVE_COMPLETE`)
  - 最终位姿
  - 运动耗时
- ✅ 旋转开始 (`ROTATE_START`)
  - 当前/目标航向
  - 转角大小
- ✅ 旋转完成 (`ROTATE_COMPLETE`)
  - 最终航向
  - 耗时
- ✅ 世界坐标运动 (`WORLD_MOVE`)
  - 世界坐标增量
  - 转换后的车体坐标
- ✅ 故障 (`FAULT`)

### 任务控制 (`MissionController`)
- ✅ 状态切换
  - WaitingForStart
  - Scanning
  - Finished
- ✅ 任务故障 (`MISSION_FAULT`)

---

## 四、使用场景

### 场景1: 调试底盘运动问题

1. 启动串口日志记录工具
2. 设置日志级别为 DEBUG
3. 运行任务
4. 分析日志中的 `MOVE_START` 和 `MOVE_COMPLETE`
5. 检查运动参数、耗时、最终位姿

**示例日志片段：**
```
[INFO ] [Chassis] MOVE_START forward=1000.0mm right=0.0mm yaw=0.0° speed=800RPM
[DEBUG] [Chassis] WORLD_MOVE world=(500.0,866.0) body=(1000.0,0.0) yaw=60.0°
[INFO ] [Chassis] MOVE_COMPLETE pose=(1500.0, 866.0, 60.0°) duration=2156ms
```

### 场景2: 分析任务流程

1. 保持默认 INFO 级别
2. 记录完整任务运行
3. 搜索关键词：
   - `STATE:` - 任务状态切换
   - `MOVE_START` - 底盘运动
   - `VISION_ALIGN` - 视觉对位
   - `FAULT` - 故障点

### 场景3: 性能分析

1. 记录日志
2. 统计各阶段耗时：
   ```bash
   grep "duration=" logs/serial_log_*.txt
   ```
3. 分析运动效率和瓶颈

### 场景4: 故障诊断

1. 运行直到故障
2. 在日志中搜索 `[ERROR]` 或 `FAULT`
3. 查看故障前的状态和参数
4. 定位问题根源

---

## 五、日志输出示例

### 完整启动流程

```
========================================
Debug Log System Initialized
Log Level: INFO
Time: 4123 ms
========================================

[0000004123] [INFO ] [System] === 2026 GCDS Control System Starting ===
[0000004234] [INFO ] [Init] Initializing chassis...
[0000004345] [INFO ] [Init] Start pose: (200.0, 200.0, 0.0°)
[0000004456] [INFO ] [Init] Initializing vision camera...
[0000004567] [INFO ] [Init] Initializing QR code scanner...
[0000004678] [INFO ] [Init] Initializing mission controller...
[0000004789] [INFO ] [Init] Initializing diagnostics...
[0000004890] [INFO ] [Init] Initializing mechanism...
[0000005001] [INFO ] [Init] Initializing display...
[0000005112] [INFO ] [System] === Initialization Complete ===
[0000010234] [INFO ] [Mission] STATE: WaitingForStart
[0000015456] [INFO ] [Mission] STATE: Scanning
[0000018789] [INFO ] [Chassis] MOVE_START forward=1000.0mm right=0.0mm yaw=0.0° speed=800RPM
[0000021122] [INFO ] [Chassis] MOVE_COMPLETE pose=(1000.0, 200.0, 0.1°) duration=2333ms
```

### 视觉对位流程

```
[0000025456] [INFO ] [Vision] VISION_ALIGN station=RoughProcessing result=SUCCESS error=(1.2, -0.8)px
[0000025467] [INFO ] [Chassis] MOVE_START forward=3.6mm right=2.4mm yaw=0.1° speed=90RPM
[0000025890] [INFO ] [Chassis] MOVE_COMPLETE pose=(1003.6, 202.4, 0.1°) duration=423ms
```

### 故障示例

```
[0000030123] [ERROR] [Chassis] FAULT: IMU stale while translating
[0000030124] [ERROR] [Mission] MISSION_FAULT: chassis fault during route execution
```

---

## 六、常见问题

### Q1: 串口连接失败？
**A:** 检查：
1. 串口名称是否正确（`--list` 查看）
2. 串口是否被其他程序占用
3. 权限问题（Linux需要 `sudo` 或加入 `dialout` 组）

### Q2: 日志文件在哪里？
**A:** 默认保存在 `logs/` 目录，文件名格式：`serial_log_YYYYMMDD_HHMMSS.txt`

### Q3: 如何减少日志输出？
**A:** 
```cpp
// 只显示警告和错误
DebugLog::setLevel(DebugLog::Level::WARNING);
```

### Q4: 日志影响性能吗？
**A:** 
- INFO/WARNING/ERROR 级别：影响极小
- DEBUG/TRACE 级别：可能影响实时性，建议只在调试时使用

### Q5: 如何在发布版本中禁用日志？
**A:** 在 `platformio.ini` 中添加：
```ini
build_flags = 
    -D DISABLE_DEBUG_LOG
```

---

## 七、最佳实践

1. **开发阶段**：使用 INFO 或 DEBUG 级别
2. **测试阶段**：使用 INFO 级别，记录完整日志
3. **比赛阶段**：使用 WARNING 级别，只记录异常
4. **定期分析日志**：发现潜在问题和优化点
5. **关键节点添加日志**：状态切换、运动开始/结束、视觉对位
6. **日志简洁明确**：避免无意义的输出
7. **保存重要日志**：成功运行和故障案例都要保留

---

## 八、扩展日志点

如需在其他模块添加日志，按此模式：

```cpp
// 1. 包含头文件
#include "DebugLog.h"

// 2. 在关键位置添加日志
void MyModule::importantFunction() {
    LOG_INFO("MyModule", "Function started");
    
    // ... 代码 ...
    
    LOGF_DEBUG("MyModule", "Processing value=%.2f", value);
    
    if (error) {
        LOG_ERROR("MyModule", "Something failed");
    }
}
```

推荐添加日志的位置：
- 状态机切换
- 异步操作开始/结束
- 重要参数变化
- 错误和异常处理
- 性能敏感操作

---

## 总结

现在你有完整的日志系统：
- ✅ Python串口记录工具（自动保存+彩色显示）
- ✅ C++调试日志库（分级+格式化）
- ✅ 主要模块已集成日志输出
- ✅ 完整的使用文档

运行测试，观察串口输出，分析任务流程！
