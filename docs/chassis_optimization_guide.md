# 底盘优化完整指南

## 目录
1. [快速开始](#快速开始)
2. [距离标定流程](#距离标定流程)
3. [航向偏移诊断与修正](#航向偏移诊断与修正)
4. [参数调优指南](#参数调优指南)
5. [故障排查](#故障排查)

---

## 快速开始

### 立即可做的优化（30分钟）

#### 1. 编译并运行距离标定测试
```bash
cd /Users/yee/elysium/GCDS/2026Contorl
pio run -e distance_calibration_test -t upload
```

#### 2. 标定四个方向
- 准备：卷尺、胶带、平整地面
- 测试：每个方向运行一次，测量实际距离
- 记录：填入下表

| 方向 | 目标(mm) | 实测(mm) | 航向偏移(°) | 建议系数 |
|------|---------|---------|------------|---------|
| 前进 | 1000 | _____ | _____ | 1000/实测 |
| 后退 | 1000 | _____ | _____ | 1000/实测 |
| 左移 | 1000 | _____ | _____ | 1000/实测 |
| 右移 | 1000 | _____ | _____ | 1000/实测 |

#### 3. 更新配置并重新编译
编辑 `include/chassis_config.h`:
```cpp
constexpr float FORWARD_DISTANCE_SCALE = ____f;   // 填入建议系数
constexpr float BACKWARD_DISTANCE_SCALE = ____f;
constexpr float RIGHT_DISTANCE_SCALE = ____f;
constexpr float LEFT_DISTANCE_SCALE = ____f;
```

```bash
pio run  # 重新编译主程序
```

---

## 距离标定流程

详见 `docs/distance_calibration_procedure.md`

### 核心步骤
1. 地面标记起点（十字线 + 车头方向）
2. 上电等待 IMU 就绪（状态灯熄灭）
3. 短按按键选择方向（LED 闪烁次数表示）
4. 长按 1 秒启动测试
5. 测量实际距离
6. 根据串口输出更新配置

### 判定标准
- ✅ 优秀：距离误差 < 10mm，航向偏移 < 1°
- ✅ 合格：距离误差 < 20mm，航向偏移 < 2°
- ⚠️ 需要调整：距离误差 20-50mm，航向偏移 2-5°
- ❌ 机械问题：距离误差 > 50mm，航向偏移 > 5°

---

## 航向偏移诊断与修正

### 诊断流程

#### 步骤 1：记录偏移模式
运行 `distance_calibration_test`，记录每个方向的航向偏移：

```
前进：+1.8°（向左偏）
后退：-1.5°（向右偏）
左移：+0.3°
右移：-0.2°
```

#### 步骤 2：分析原因

| 偏移特征 | 可能原因 | 解决方案 |
|---------|---------|---------|
| 前进向左偏，后退向右偏 | 右轮直径小或摩擦力大 | 更换右侧轮子 |
| 前进向右偏，后退向左偏 | 左轮直径小或摩擦力大 | 更换左侧轮子 |
| 前后偏移方向相同 | 电机方向配置错误 | 检查 `MOTOR_SIGN[4]` |
| 横移也有明显偏移 | 麦轮安装角度问题 | 检查 45° 安装角 |
| 所有方向不稳定 | IMU 安装不水平 | 重新安装 IMU |
| 重复测试结果差异大 | 地面不平或打滑 | 更换测试场地 |

#### 步骤 3：机械修正（优先）

**检查清单**：
- [ ] 四个轮子磨损程度是否一致
- [ ] 麦轮小滚子方向是否正确（45°）
- [ ] 轮子与地面接触是否良好（无悬空）
- [ ] IMU 是否水平安装（用水平仪检查）
- [ ] 底盘框架是否变形

#### 步骤 4：软件补偿（机械无法完全解决时）

**轻度偏移（< 2°）**：
```cpp
// include/chassis_config.h
constexpr float HEADING_KP_STEPS_PER_S_PER_DEG = 45.0f;  // 从 35 提升到 45
constexpr float HEADING_MAX_CORRECTION_RATIO = 0.35f;    // 从 0.30 提升到 0.35
```

**中度偏移（2-5°）**：
```cpp
constexpr float HEADING_KP_STEPS_PER_S_PER_DEG = 50.0f;
constexpr float HEADING_MAX_CORRECTION_RATIO = 0.40f;

// 并在 ChassisControl.cpp 中考虑集成增强策略
#define USE_ENHANCED_HEADING_CONTROL
```

**重度偏移（> 5°）**：
- ⚠️ 优先解决机械问题，软件补偿效果有限！

---

## 参数调优指南

### 场景 1：直线行驶偏航

**问题描述**: 前进 1000mm，航向偏移 3°

**调整方案**:
```cpp
// 1. 提高航向修正增益
constexpr float HEADING_KP_STEPS_PER_S_PER_DEG = 50.0f;  // 原 35

// 2. 允许更大的修正比例
constexpr float HEADING_MAX_CORRECTION_RATIO = 0.40f;  // 原 0.30

// 3. 收紧最终航向容差
constexpr float TRANSLATION_FINAL_HEADING_TOLERANCE_DEG = 0.6f;  // 原 0.8
```

**验证**: 重新测试前进 1000mm，期望偏移 < 1.5°

---

### 场景 2：小角度旋转超调

**问题描述**: 旋转 5°，实际到达 6.5°（超调 1.5°）

**调整方案**:
```cpp
// 1. 增加阻尼（D 项）
constexpr float ROTATE_KD_STEPS_PER_S_PER_DEG_PER_S = 22.0f;  // 原 18

// 2. 降低小角度最小速度
constexpr float ROTATE_MIN_STEPS_PER_S = 80.0f;  // 原 110

// 3. 收紧到位容差
constexpr float ROTATE_TOLERANCE_DEG = 0.5f;  // 原 0.6
```

**验证**: 连续旋转 90° × 4 次，检查累积误差

---

### 场景 3：斜移时航向不稳

**问题描述**: 45° 斜向移动时，航向波动 ±2°

**原因分析**: 
- 斜移时短行程轮速度低，航向修正能力受限
- 这是设计限制，不是缺陷

**优化方案**:
1. **降低斜移速度**（最有效）
2. **启用增强策略**（区分直线和斜移）
3. **路线规划时避免长距离斜移**

如果必须使用，可以考虑在 `ChassisControl.cpp` 第 576-581 行修改：
```cpp
// 原代码
maxCorrection = min(maxCorrection, _translationSpeed[i] * 0.8f);

// 改为（风险：可能导致短行程轮反转）
maxCorrection = min(maxCorrection, _translationSpeed[i] * 0.85f);
```

---

### 场景 4：长距离累积误差

**问题描述**: 连续移动 3000mm 后，位置误差 80mm，航向误差 4°

**根本原因**: 里程计累积误差无修正

**解决方案**:

#### 方案 A：利用视觉对准修正（推荐）
在每个工位视觉对准完成后，反馈修正底盘位姿：

```cpp
// 在 MaixProRingAlignment::update() 对准完成时
if (对准完成) {
    ChassisControl::Pose2D current = _chassis.worldPose();
    
    // 根据视觉修正量计算实际位置
    float correctedX = current.xMm + totalVisionCorrectionX;
    float correctedY = current.yMm + totalVisionCorrectionY;
    
    // 修正底盘世界坐标（不改变航向基准）
    _chassis.correctWorldPosition(correctedX, correctedY);
}
```

#### 方案 B：在区域基准点强制重置
```cpp
// 在 MissionController 每个工位完成后
// 假设已知暂存区精确坐标为 (1800, 1200)
chassis.correctWorldPosition(1800.0F, 1200.0F);
```

#### 方案 C：启用 EMM42 TTL 反馈（需要硬件支持）
```cpp
// main.cpp
ChassisEmm42TtlFeedback motorFeedback(
    serialMotorTtl, 
    {1, 2, 3, 4},  // 四个电机 ID
    115200
);
ChassisControl chassis(&serialImu, &motorFeedback);
```

---

## 故障排查

### 问题：编译失败

**现象**: `pio run -e distance_calibration_test` 报错

**检查**:
```bash
# 1. 确认 platformio.ini 中已添加环境
grep "distance_calibration_test" platformio.ini

# 2. 清理缓存重新编译
pio run -t clean
pio run -e distance_calibration_test
```

---

### 问题：IMU 不就绪

**现象**: 串口输出 "CALIB_BOOT Waiting for IMU..." 一直等待

**检查**:
1. JY901 接线：
   - RX -> PD9
   - TX -> PD8
   - VCC -> 5V
   - GND -> GND
2. 波特率：115200
3. 串口冲突：PD9/PD8 是否被其他设备占用

**测试**:
```cpp
// 临时在 setup() 添加
serialImu.println("AT+BAUD=6\r\n");  // 设置为 115200
delay(100);
```

---

### 问题：测量距离与里程计差距过大

**现象**: 目标 1000mm，里程计显示 1020mm，实测只有 850mm

**可能原因**:
1. **轮径配置错误**
   ```cpp
   // 检查 chassis_config.h
   constexpr float WHEEL_DIAMETER_MM = 100.0f;  // 实际是多少？
   ```
   用卷尺测量轮子直径，精确到 1mm

2. **细分数配置错误**
   ```cpp
   constexpr uint16_t MICROSTEPS = 32;  // 驱动器设置是多少？
   ```
   检查 EMM42 驱动器的细分拨码开关

3. **严重打滑**
   - 轮子气压不足（如果是充气轮）
   - 地面太光滑
   - 负载过重

---

### 问题：每次测试结果差异超过 30mm

**现象**: 同一方向重复测试，距离分别为 985mm, 1012mm, 997mm

**排查**:
1. **地面问题**: 换到水泥地或瓷砖地
2. **电池电压**: 充满电后测试
3. **起点标记**: 确保每次精确对准
4. **加速度过大**: 降低 `TEST_ACCEL` 到 150 RPM/s

---

### 问题：航向一直向一侧偏

**现象**: 前进向左偏 +3°，后退向右偏 -3°，每次都稳定出现

**诊断**: 这是典型的左右轮不对称

**解决**:
1. **机械修正**（优先）:
   - 测量左右轮直径，差异应 < 0.5mm
   - 检查左右轮磨损，更换磨损严重的
   - 检查底盘是否水平（一侧悬空会导致摩擦力不同）

2. **软件补偿**（临时方案）:
   ```cpp
   // 提高航向保持增益
   constexpr float HEADING_KP_STEPS_PER_S_PER_DEG = 55.0f;
   constexpr float HEADING_MAX_CORRECTION_RATIO = 0.45f;
   ```

---

## 高级优化（可选）

### 启用增强航向控制

如果标定后仍有偏移，可以尝试增强策略：

1. 复制增强配置：
   ```bash
   cp include/chassis_config_enhanced.h include/chassis_config_enhanced_backup.h
   ```

2. 参考 `chassis_config_enhanced.h` 中的参数调整 `chassis_config.h`

3. （可选）集成 `ChassisHeadingEnhancement.h` 到 `ChassisControl.cpp`：
   - 需要修改 `updateTranslation()` 函数
   - 建议先备份原文件

---

## 测试检查清单

### 初步验证（标定后必做）
- [ ] 前进 1000mm: 误差 < 20mm, 偏航 < 2°
- [ ] 后退 1000mm: 误差 < 20mm, 偏航 < 2°
- [ ] 左移 1000mm: 误差 < 20mm, 偏航 < 2°
- [ ] 右移 1000mm: 误差 < 20mm, 偏航 < 2°

### 综合测试
- [ ] 正方形路径: 前进→右移→后退→左移, 回到起点误差 < 50mm
- [ ] 连续旋转: 90° × 4 次，累积误差 < 3°
- [ ] 长距离: 前进 2000mm, 误差 < 50mm

### 比赛场景测试
- [ ] 模拟原料区到粗加工区路线
- [ ] 模拟完整双批次路线
- [ ] 检查最终累积误差 < 100mm

---

## 参数快速参考

| 参数 | 默认值 | 调优范围 | 影响 |
|-----|--------|---------|------|
| HEADING_KP | 35.0 | 40-60 | 航向修正速度 |
| HEADING_MAX_CORRECTION_RATIO | 0.30 | 0.35-0.50 | 最大修正比例 |
| ROTATE_KP | 135.0 | 120-150 | 旋转响应速度 |
| ROTATE_KD | 18.0 | 15-25 | 旋转阻尼 |
| ROTATE_MIN_STEPS_PER_S | 110.0 | 60-120 | 小角度最小速度 |
| ROTATE_TOLERANCE_DEG | 0.6 | 0.4-0.8 | 旋转到位容差 |

---

## 联系与支持

如果遇到问题：
1. 检查本文档的故障排查部分
2. 查看 `docs/distance_calibration_procedure.md`
3. 查看串口调试输出（PB13, 115200 波特率）
4. 记录详细的测试数据和现象

**调试串口输出关键字**:
- `CALIB_*`: 标定测试相关
- `STRAIGHT_*`: 直线测试相关
- `STATUS`: 实时状态
- `FAULT`: 错误信息

祝标定顺利！🎯
