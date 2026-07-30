# 2026Contorl

STM32H750VB 智能搬运机器人比赛控制工程，使用 Arduino Framework
和 PlatformIO。项目不依赖 OPS9，以底盘步进脉冲、JY901 航向、
MaixPro 视觉和分层非阻塞状态机完成比赛流程。

## 当前模块

| 模块 | 目录 | 职责 |
|---|---|---|
| 底盘控制 | `lib/ChassisControl` | 麦轮逆解算、相对位移、IMU 航向保持 |
| 路线执行 | `lib/MissionControl/RouteExecutor.*` | 顺序执行快速/精确移动和绝对旋转 |
| 比赛状态机 | `lib/MissionControl/MissionController.*` | 编排扫码、移动、对准和机构任务 |
| 二维码 | `lib/QRCodeMissionProvider` | 解析 `134+123+314+231` 格式任务码 |
| MaixPro | `lib/MaixProVision` | V2 帧协议和原料抓取视觉适配 |
| 机械臂 | `lib/MechanismControl` | 非阻塞取料、粗加工放取和暂存码垛 |
| 串口屏 | `lib/MissionDisplay` | 显示任务状态、故障和换行任务码 |

供应商库保留在 `lib/TTL_STEPPER`、`lib/AccelStepper-master`、
`lib/fashionstar-*` 等目录，自研业务逻辑不直接修改这些库。

## 配置入口

- `include/chassis_config.h`：底盘引脚、机械尺寸、速度和航向控制。
- `include/mechanism_config.h`：机械臂 ID、引脚、速度和全部实车点位。
- `include/mission_config.h`：按键、状态灯、扫码器和串口屏。
- `include/vision_config.h`：MaixPro 引脚、目标中心、闭环增益和软限位。
- `lib/MissionControl/MissionRoutes.h`：各比赛区域之间的底盘路线。

实车标定只修改配置文件，不在状态机或动作执行器中散落常量。

## 主要硬件映射

| 设备 | 接口 |
|---|---|
| JY901 | PD9 RX / PD8 TX，115200 |
| GM75 | PE0 RX / PE1 TX，9600 |
| 串口屏 | PB15 RX / PB14 TX，115200 |
| MaixPro | PE7 RX / PE8 TX，115200 |
| 升降、伸缩步进 | PA3 RX / PA2 TX，ID 7 / ID 6 |
| 底座步进 | PA10 RX / PA9 TX，ID 5 |
| 夹爪、载物盘舵机 | PC7 RX / PC6 TX，ID 4 / ID 5 |
| 一键启动 | PB9 |
| 状态灯 | PA15 |

PB12/PB13 调试串口当前不启用，避免影响已经验证的底盘运行。

## 第一阶段任务流程

```text
上电初始化
→ 等待一键启动
→ 前往二维码区并读取任务码
→ 第一批：原料区取料 → 粗加工区放置并取回 → 暂存区平放
→ 第二批：原料区取料 → 粗加工区放置并取回 → 暂存区同色码垛
→ 返回启停区
```

原料抓取按照任务码颜色逐个执行：

```text
机械臂到抓取准备位
→ MaixPro 跟踪目标颜色
→ 底座和伸缩轴小步闭环
→ 连续稳定后下降夹取
→ 物料放入车载盘
```

工位最后一个物料放稳并抬升到安全高度后，底盘立即驶向下一
区域。机械臂底座、伸缩轴和载物盘在行驶期间异步回到运输收纳位；
如果底盘先到达，状态机会等待机构收纳完成后再启动下一工位动作。

## 当前未完成

- 粗加工区和暂存区的 MaixPro 整车到站对准仍为直通实现。
- 随机障碍物避障尚未接入。
- 两个随机启停区尚未做路线选择。
- 串口屏尚未统计正确抓取和正确放置次数。
- MaixPro 的中心点、修正方向和增益仍需实车联调。

## 构建

```sh
pio run
pio run -t upload
```

首次运行或修改机械臂点位后，应架空底盘并清空机构周围空间，
确认初始化方向、软限位和紧急停车行为后再落地测试。
