# 机械臂初始化诊断报告

## 问题现象

从串口日志分析，发现以下关键信息：

1. ✅ **setup() 函数执行完成**
   - 所有模块初始化成功
   - 机构伺服电机在线 (Storage=ONLINE, Gripper=ONLINE)
   - 显示器初始化完成
   - 输出了 "Initialization Complete" 和 "Entering main loop..."

2. ⚠️ **loop() 函数开始执行，但日志被截断**
   ```
   [0000004322] [INFO ] [Loop] *** LOO
   ```
   - 这条日志应该是 "*** LOOP() STARTED ***"
   - 但只输出了 "*** LOO" 就停止了

## 可能的原因

### 1. 系统复位循环（最可能）
- 程序在进入 loop() 后触发了某种错误导致自动复位
- 如果是这个原因，应该会看到系统反复重启的日志

### 2. 串口缓冲区问题
- 输出速度过快导致缓冲区溢出
- 数据传输丢失

### 3. 栈溢出
- loop() 中某个函数调用导致栈溢出
- 当前栈使用率：RAM 1.6% (8248/524288 bytes)
- Flash 使用率：45.6% (59732/131072 bytes)

### 4. 中断冲突
- 多个串口同时工作可能导致中断冲突
- 当前使用的串口：
  - Debug Serial (PB12/PB13)
  - IMU Serial (PD8/PD9)
  - Display Serial
  - Mechanism Serial (多个)

## 已采取的措施

### 1. 简化 loop() 函数
已注释掉大部分 update 调用，只保留：
- diagnostics.update()
- 每秒输出一次心跳日志

### 2. 添加详细日志
- 在 loop() 开始时输出确认信息
- 添加周期性心跳输出

## 下一步诊断

请运行以下命令，持续监听串口输出（至少30秒）：

```bash
cd /Users/yee/elysium/GCDS/2026Contorl
~/.platformio/penv/bin/pio device monitor --baud 115200 --port /dev/cu.usbmodem5078740666043
```

或者按下开发板的复位键，观察输出：

### 预期输出 A：正常运行
```
[0000004322] [INFO ] [Loop] *** LOOP() STARTED ***
[0000004323] [DEBUG] [Loop] First loop iteration
[0000005323] [INFO ] [Loop] Loop running, time=5323 ms
[0000006323] [INFO ] [Loop] Loop running, time=6323 ms
DBG t=5000 ...
DBG t=6000 ...
```

### 预期输出 B：重启循环
```
[0000004322] [INFO ] [Loop] *** LOO
[0000004110] [INFO ] [System] === 2026 GCDS Control System Starting ===
[0000004116] [INFO ] [Init] Initializing chassis...
...
[0000004322] [INFO ] [Loop] *** LOO
[0000004110] [INFO ] [System] === 2026 GCDS Control System Starting ===
```

### 预期输出 C：串口卡死
```
[0000004322] [INFO ] [Loop] *** LOO
(没有更多输出，但系统仍在运行)
```

## 如果是重启循环

需要检查：
1. **硬件看门狗**：是否启用了独立看门狗(IWDG)
2. **电源问题**：电压是否稳定，电流是否充足
3. **内存访问错误**：是否访问了无效地址
4. **HardFault 异常**：需要添加 HardFault 处理函数捕获崩溃信息

## 如果是串口卡死

需要：
1. 减少串口输出频率
2. 增加串口发送缓冲区
3. 使用 DMA 方式发送串口数据

## 如果正常运行

说明问题在被注释掉的某个 update() 函数中，需要逐个恢复测试：
1. startButton.tick()
2. chassis.update()
3. mission.update()
4. display.update()
5. updateStatusLed()
