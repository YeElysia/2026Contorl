# 串口输出系数读取指南

## 📡 串口连接设置

### 硬件连接
- **调试串口引脚**: PB12 (RX) / PB13 (TX)
- **波特率**: 115200
- **数据位**: 8
- **停止位**: 1
- **校验**: 无

### 软件工具
推荐使用以下任一工具：
- **串口助手**（Windows）
- **CoolTerm**（macOS/Windows）
- **screen**（macOS/Linux 终端）
- **PlatformIO Serial Monitor**

### 打开串口监视器

#### 方法 1：使用 PlatformIO
```bash
pio device monitor -b 115200
```

#### 方法 2：使用 screen（macOS）
```bash
# 1. 找到设备
ls /dev/tty.usb*

# 2. 连接（假设设备是 /dev/tty.usbserial-xxx）
screen /dev/tty.usbserial-xxx 115200

# 退出：按 Ctrl+A 然后按 K，确认退出
```

---

## 📋 完整输出示例和解读

### 启动阶段

```
CALIB_BOOT Waiting for IMU...
CALIB_BOOT System initialized
========================================
Distance Calibration Test Ready
========================================
Test distance: 1000 mm
Max RPM: 800
Acceleration: 200 RPM/s
Microsteps: 32
Wheel diameter: 100.0 mm
---
Current scales:
  FORWARD:  1.0000
  BACKWARD: 1.0000
  LEFT:     1.0000
  RIGHT:    1.0000
---
Controls:
  Short click: Cycle direction (LED blinks show choice)
  Long press:  Start test
========================================
Selected: FORWARD
```

**解读**：
- ✅ IMU 已就绪
- 📊 当前标定系数都是 1.0（未标定）
- 🎯 默认选择前进方向

---

### 选择方向

每次短按按键后：
```
CALIB_SELECT direction=BACKWARD (2/4)
```

**解读**：
- 当前选择：后退方向
- 这是第 2 个方向（共 4 个）
- LED 会闪烁 2 次表示选择了后退

---

### 开始测试

长按 1 秒后：
```
CALIB_START direction=FORWARD target=1000mm startYaw=0.12 startPose=0.0,0.0,0.12
```

**解读**：
- 测试方向：前进
- 目标距离：1000mm
- 起始航向：0.12°
- 起始位姿：(X=0, Y=0, Yaw=0.12°)

---

### 测试完成（关键部分）⭐

```
========================================
CALIB_DONE direction=FORWARD
  Elapsed: 3250 ms                           ← 用时 3.25 秒
  Target distance: 1000 mm                   ← 目标距离
  Odometry distance: 1023.4 mm               ← 里程计记录的距离（基于脉冲）
  Delta X: -2.1 mm, Delta Y: 1023.2 mm       ← X/Y 方向位移
  ---
  Current FORWARD_DISTANCE_SCALE: 1.0000     ← 当前系数
  ---
  [ACTION REQUIRED]                           ← ⚠️ 需要你手动操作
  1. Measure ACTUAL distance traveled from start mark
  2. Calculate new scale = 1000.0 / measured_distance
     If odometry (1023.4 mm) is accurate:
     Suggested FORWARD_DISTANCE_SCALE = 0.9771    ← 🎯 建议系数（关键数字）
  ---
  Heading drift: 0.8 degrees                 ← 航向偏移
  Heading drift within acceptable range      ← 偏移评估
========================================
```

---

## 🎯 如何读取和使用系数

### 第一步：找到建议系数

在输出中查找这一行：
```
Suggested FORWARD_DISTANCE_SCALE = 0.9771
                                   ^^^^^^
                                   这就是建议系数！
```

### 第二步：理解这个数字的含义

**情况 A：系数 < 1.0（例如 0.9771）**
- 含义：底盘跑得**比目标远**
- 原因：实际轮径大于配置值，或脉冲计算偏大
- 例子：目标 1000mm，里程计显示 1023.4mm
- 修正：乘以 0.9771，下次就会正好跑 1000mm

**情况 B：系数 > 1.0（例如 1.0204）**
- 含义：底盘跑得**比目标近**
- 原因：实际轮径小于配置值，或脉冲计算偏小
- 例子：目标 1000mm，里程计显示 980mm
- 修正：乘以 1.0204，下次就会正好跑 1000mm

**情况 C：系数 = 1.0（准确）**
- 含义：距离已经很准确
- 输出：`Distance accurate, no adjustment needed`

### 第三步：验证（重要！）

**⚠️ 如果里程计和实测差距很大，以实测为准**

假设串口显示：
```
Odometry distance: 1023.4 mm
Suggested FORWARD_DISTANCE_SCALE = 0.9771
```

但你用卷尺测量实际只跑了 **980mm**，那么：

```
正确的系数 = 1000.0 / 980.0 = 1.0204  ← 用实测值计算
            不是 0.9771！
```

**为什么会不一致？**
- 轮子打滑
- 地面不平
- 测量误差

---

## 📝 记录表格（推荐格式）

准备一个表格记录所有数据：

```
测试日期：2024-XX-XX
电池电压：12.4V
测试场地：实验室水泥地

┌──────────┬────────┬──────────┬────────────┬──────────┬────────────┐
│ 方向     │ 目标   │ 里程计   │ 实际测量   │ 航向偏移 │ 最终系数   │
├──────────┼────────┼──────────┼────────────┼──────────┼────────────┤
│ 前进     │ 1000mm │ 1023.4mm │  982 mm    │  +0.8°   │  1.0183    │
├──────────┼────────┼──────────┼────────────┼──────────┼────────────┤
│ 后退     │ 1000mm │  995.2mm │  1015 mm   │  -1.2°   │  0.9852    │
├──────────┼────────┼──────────┼────────────┼──────────┼────────────┤
│ 左移     │ 1000mm │  1008mm  │  990 mm    │  +0.3°   │  1.0101    │
├──────────┼────────┼──────────┼────────────┼──────────┼────────────┤
│ 右移     │ 1000mm │  973mm   │  950 mm    │  -0.5°   │  1.0526    │
└──────────┴────────┴──────────┴────────────┴──────────┴────────────┘

最终系数（填入 chassis_config.h）：
FORWARD_DISTANCE_SCALE  = 1.0183f
BACKWARD_DISTANCE_SCALE = 0.9852f
LEFT_DISTANCE_SCALE     = 1.0101f
RIGHT_DISTANCE_SCALE    = 1.0526f
```

---

## 💡 实际操作流程

### 完整示例

#### 1. 测试前进方向

**串口输出**：
```
CALIB_START direction=FORWARD target=1000mm startYaw=0.12
[底盘移动...]
CALIB_DONE direction=FORWARD
  Odometry distance: 1023.4 mm
  Suggested FORWARD_DISTANCE_SCALE = 0.9771
  Heading drift: 0.8 degrees
```

**你的操作**：
1. 用卷尺测量底盘中心到起点标记的距离：**982mm**
2. 计算正确系数：1000 / 982 = **1.0183**
3. 记录：
   - 里程计：1023.4mm
   - 实测：982mm
   - 系数：1.0183（✅ 用实测值）
   - 航向：+0.8°

#### 2. 更新配置文件

编辑 `include/chassis_config.h`：

```cpp
// 第 38 行
constexpr float FORWARD_DISTANCE_SCALE = 1.0183f;  // 填入计算的系数
```

#### 3. 重复其他方向

短按按键选择后退 → 长按测试 → 测量 → 记录系数

---

## ⚠️ 常见问题

### Q1: 里程计和实测差距很大（>5%）怎么办？

**可能原因**：
1. 轮子打滑严重
2. 地面不平
3. 轮径配置错误

**解决方案**：
```cpp
// 检查 chassis_config.h 第 24 行
constexpr float WHEEL_DIAMETER_MM = 100.0f;  // 用卷尺实际测量轮子直径
```

### Q2: 看不到串口输出

**检查清单**：
- [ ] 串口线是否连接到 PB12/PB13
- [ ] 波特率是否设置为 115200
- [ ] 是否选错了串口设备（试试其他 /dev/tty.usb*）
- [ ] 程序是否正常烧录（状态灯应该熄灭表示 IMU 就绪）

### Q3: 系数应该保留几位小数？

**建议保留 4 位小数**：
```cpp
constexpr float FORWARD_DISTANCE_SCALE = 1.0183f;  // ✅ 推荐
constexpr float FORWARD_DISTANCE_SCALE = 1.02f;    // ⚠️ 精度不够
constexpr float FORWARD_DISTANCE_SCALE = 1.018345f; // ❌ 过度精确
```

### Q4: 四个方向的系数差距很大正常吗？

**正常范围**：
- 前进 vs 后退：差距 < 3%
- 左移 vs 右移：差距 < 3%
- 前后 vs 左右：差距 < 5%（麦轮特性不同）

**异常情况**：
- 差距 > 10%：检查轮子磨损、安装
- 前后系数相反：电机方向配置可能错误

---

## 🖥️ 串口监视器使用

### 使用 PlatformIO Serial Monitor

```bash
# 打开串口监视器
pio device monitor -b 115200
```

**期望看到的输出**：
```
--- Available filters and text transformations: colorize, debug, default, direct, hexlify, log2file, nocontrol, printable, send_on_enter, time
--- More details at https://bit.ly/pio-monitor-filters
--- Miniterm on /dev/tty.usbserial-XXXX  115200,8,N,1 ---
--- Quit: Ctrl+C | Menu: Ctrl+T | Help: Ctrl+T followed by Ctrl+H ---
CALIB_BOOT Waiting for IMU...
...
```

**如果看到乱码**：
- 检查波特率是否为 115200
- 检查串口线是否接反（RX ↔ TX）

---

## 📌 快速参考卡

**打印或复制到桌面，测试时查阅**：

```
┌─────────────────────────────────────────────────────────┐
│ 距离标定系数读取快速参考                                │
├─────────────────────────────────────────────────────────┤
│ 1. 串口设置：115200, 8N1, PB12/PB13                     │
│ 2. 关键输出：Suggested XXXXXX_DISTANCE_SCALE = X.XXXX  │
│ 3. 计算公式：系数 = 1000 / 实测距离(mm)                │
│ 4. 验证方法：如果里程计≠实测，以实测为准！             │
│ 5. 保留位数：4 位小数（例如 1.0183f）                  │
│ 6. 配置文件：include/chassis_config.h 第 38-41 行      │
│ 7. 重新编译：pio run                                    │
└─────────────────────────────────────────────────────────┘
```

---

## 🎬 完整操作演示

### 场景：标定前进方向

```
第 1 步：连接串口
$ pio device monitor -b 115200
--- Miniterm on /dev/tty.usbserial-ABC123  115200,8,N,1 ---

第 2 步：等待系统就绪
CALIB_BOOT Waiting for IMU...
[等待 2-3 秒]
CALIB_BOOT System initialized
========================================
Distance Calibration Test Ready
[显示配置信息...]
Selected: FORWARD   ← LED 灯熄灭，可以开始

第 3 步：启动测试（长按按键 1 秒）
CALIB_START direction=FORWARD target=1000mm startYaw=0.12
[底盘开始移动，LED 常亮]
[3-4 秒后停止]

第 4 步：读取输出
CALIB_DONE direction=FORWARD
  Elapsed: 3250 ms
  Target distance: 1000 mm
  Odometry distance: 1023.4 mm              ← 记录这个数字
  ---
  Suggested FORWARD_DISTANCE_SCALE = 0.9771 ← 🎯 建议系数
  ---
  Heading drift: 0.8 degrees                ← 记录航向偏移

第 5 步：测量实际距离
[用卷尺测量底盘中心到起点标记]
实测结果：982mm                             ← 记录实测值

第 6 步：计算最终系数
系数 = 1000 / 982 = 1.0183                  ← 用实测值计算！

第 7 步：更新配置
编辑 include/chassis_config.h:
constexpr float FORWARD_DISTANCE_SCALE = 1.0183f;

第 8 步：重复其他方向
[短按按键选择后退 → LED 闪烁 2 次]
[长按 1 秒启动...]
```

---

## 📊 判断系数是否合理

### 合理的系数范围

| 方向 | 正常范围 | 可疑范围 | 异常范围 |
|------|---------|---------|---------|
| 前进 | 0.95 - 1.05 | 0.90 - 0.95 或 1.05 - 1.10 | < 0.90 或 > 1.10 |
| 后退 | 0.95 - 1.05 | 0.90 - 0.95 或 1.05 - 1.10 | < 0.90 或 > 1.10 |
| 左移 | 0.92 - 1.08 | 0.87 - 0.92 或 1.08 - 1.13 | < 0.87 或 > 1.13 |
| 右移 | 0.92 - 1.08 | 0.87 - 0.92 或 1.08 - 1.13 | < 0.87 或 > 1.13 |

**如果系数异常**：
- 检查轮径配置（`WHEEL_DIAMETER_MM`）
- 检查细分设置（`MICROSTEPS`）
- 重新测量确认

---

希望这份指南能帮到你！测试时有任何问题随时问我。🎯
