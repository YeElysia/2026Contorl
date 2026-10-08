#pragma once

#include <Arduino.h>

/**
 * @brief 机械臂硬件和标定参数。
 *
 * 硬件ID、引脚和电机基础参数以new_project为准。
 * 位置参数已经开始按实车标定，后续仍只在本文件中调整。
 *
 * TTL步进位置单位：
 * - 升降、伸缩：0.1 mm；
 * - 底座旋转：0.1°。
 */
namespace mechanism_config
{
    // -------------------- 调试选项 --------------------
    /**
     * @brief 跳过机构初始化（用于测试底盘和视觉系统）
     *
     * 设置为true时，机构会立即报告ready状态而不执行初始化动作。
     * 这允许在机构硬件未连接时测试底盘移动和视觉系统。
     *
     * ⚠️ 生产环境必须设置为false！
     */
    constexpr bool SKIP_MECHANISM_INIT = false;

    // -------------------- 硬件映射（以new_project为准） --------------------
    // 升降、伸缩两个步进驱动共用的串口（USART2）接收脚，接驱动器TX。
    constexpr uint32_t STEPPER_RX_PIN = PA3;
    // 升降、伸缩共用串口的发送脚，接驱动器RX。
    constexpr uint32_t STEPPER_TX_PIN = PA2;
    // 底座步进驱动独占的串口（USART1）接收脚。
    constexpr uint32_t BASE_RX_PIN = PA10;
    // 底座步进驱动独占串口的发送脚。
    constexpr uint32_t BASE_TX_PIN = PA9;
    // 夹爪与载物盘两个总线舵机共用的串口接收脚。
    constexpr uint32_t SERVO_RX_PIN = PC7;
    // 总线舵机共用串口的发送脚。
    constexpr uint32_t SERVO_TX_PIN = PC6;

    // 步进驱动与总线舵机串口的波特率，驱动器侧必须设成相同值。
    constexpr uint32_t BUS_BAUD = 115200;

    // 升降轴步进驱动的总线地址，与驱动器拨码/设置一致。
    constexpr uint8_t LIFT_STEPPER_ID = 7;
    // 伸缩轴步进驱动的总线地址。
    constexpr uint8_t EXTENSION_STEPPER_ID = 6;
    // 底座旋转步进驱动的总线地址。
    constexpr uint8_t BASE_STEPPER_ID = 5;
    // 夹爪舵机的总线ID。
    constexpr uint8_t GRIPPER_SERVO_ID = 4;
    // 载物盘（储料盘）舵机的总线ID，与夹爪舵机在同一条总线上。
    constexpr uint8_t STORAGE_SERVO_ID = 5;

    // -------------------- 步进驱动参数（以new_project为准） --------------------
    // 升降轴默认（下行及一般）运行速度，直接下发给驱动器的速度值。
    constexpr uint16_t LIFT_SPEED = 2500;
    // 升降轴默认加速度档（0~255，越大加减速越猛）。
    constexpr uint8_t LIFT_ACCELERATION = 253;
    // 升降坐标变小为向上，上行动作单独使用该档位。
    constexpr uint16_t LIFT_UP_SPEED = 5500;
    // 升降轴上行加速度档，仅在目标比上次命令更小（向上）时使用。
    constexpr uint8_t LIFT_UP_ACCELERATION = 255;
    // 升降轴电机正方向是否为顺时针；决定坐标增大对应的旋转方向，装配/接线改变后才需改。
    constexpr bool LIFT_CW = false;
    // 升降轴换算系数：把0.1mm单位的目标坐标换算为驱动器角度，与机械传动比有关，改传动结构后才需重新标定。
    constexpr float LIFT_CONVERT_K = 120.0F;
    // 升降轴驱动器细分数，必须与驱动器侧设置一致，否则行程比例错误。
    constexpr uint16_t LIFT_SUBSTEP = 16;

    // 伸缩轴默认运行速度，直接下发给驱动器的速度值。
    constexpr uint16_t EXTENSION_SPEED = 170;
    // 伸缩轴默认加速度档（0~255）。
    constexpr uint8_t EXTENSION_ACCELERATION = 200;
    // 原料视觉追踪采用旧项目接近的快速档，其他机构动作仍保持120RPM。
    constexpr uint16_t PICKUP_EXTENSION_TRACK_SPEED = 220;
    // 原料视觉追踪（CollectAligning阶段）时伸缩轴使用的加速度档。
    constexpr uint8_t PICKUP_EXTENSION_TRACK_ACCELERATION = 220;
    // 伸缩轴电机正方向是否为顺时针；决定坐标增大（收回）对应的旋转方向。
    constexpr bool EXTENSION_CW = true;
    // 伸缩轴换算系数：0.1mm目标坐标到驱动器角度，与机械传动比有关。
    constexpr float EXTENSION_CONVERT_K = 1131.0F;
    // 伸缩轴驱动器细分数，需与驱动器设置一致。
    constexpr uint16_t EXTENSION_SUBSTEP = 16;

    // 底座旋转默认速度，用于绝大多数底座转动；下发给驱动器的速度值。
    constexpr uint16_t BASE_SPEED = 250;
    // 底座默认加速度档（0~255）。
    constexpr uint8_t BASE_ACCELERATION = 220;
    // 底座电机正方向是否为顺时针；决定角度增大对应的旋转方向。
    constexpr bool BASE_CW = true;
    // 底座换算系数：0.1°目标角度到驱动器角度，与底座减速比有关。
    constexpr float BASE_CONVERT_K = 900.0F;
    // 底座驱动器细分数，需与驱动器设置一致。
    constexpr uint16_t BASE_SUBSTEP = 16;

    // 储料盘到粗加工圆环及暂存区第一层时，Base统一使用该快速档。
    constexpr uint16_t TRAY_TO_ROUGH_RING_BASE_SPEED = 460;
    // 载物盘到圆环这一段底座转动的加速度档，与上面的速度配套。
    constexpr uint8_t TRAY_TO_ROUGH_RING_BASE_ACCELERATION = 247;

    // 转盘取料后底座转回载物盘（约95°）使用的速度；仅此一段，不影响其他底座动作。
    constexpr uint16_t PICKUP_TO_TRAY_BASE_SPEED = 350;
    // 转盘取料后底座转回载物盘的加速度档，与上面的速度配套。
    constexpr uint8_t PICKUP_TO_TRAY_BASE_ACCELERATION = 230;

    /*
     * 粗加工圆环直接回到储料盘2770时使用的三组速度。
     * 参数保持实车当前值，使三个圆环的直接回程时间接近升降时间。
     * 下标1~3对应1~3号圆环，下标0为无效占位（取默认值）。
     */
    constexpr uint16_t ROUGH_RING_TO_TRAY_SPEED[4] = {
        BASE_SPEED, 180, 180, 200};
    // 对应圆环回载物盘时底座的加速度档，下标含义同上。
    constexpr uint8_t ROUGH_RING_TO_TRAY_ACCELERATION[4] = {
        BASE_ACCELERATION, 150, 200, 200};
    // first_layer_placement_test中1号圆环返回载物盘时的底座速度。
    constexpr uint16_t FIRST_LAYER_RING_ONE_TO_TRAY_SPEED = 150;
    // first_layer_placement_test中1号圆环返回载物盘时的底座加速度。
    constexpr uint8_t FIRST_LAYER_RING_ONE_TO_TRAY_ACCELERATION = 150;

    // -------------------- 舵机参数（以new_project为准） --------------------
    // 夹爪普通张开角度：圆环/载物盘取放、初始化后常用的张开位。
    constexpr float GRIPPER_OPEN_ANGLE = 80.0F;
    // 夹爪闭合角度：夹住物料（及空载初始化）时的目标角度，数值越大夹得越紧。
    constexpr float GRIPPER_CLOSE_ANGLE = 105.0F;
    // 夹爪最大张开角度：赴原料盘/工位视觉对准时使用，给相机和物料留最大空间；同时参与舵机角度范围设置。
    constexpr float GRIPPER_OPEN_MAX_ANGLE = 50.0F;
    // 夹爪闭合时的最大输出功率（舵机力矩上限），越大夹持越牢但越容易压伤/碰飞物料。
    constexpr uint16_t GRIPPER_MAX_POWER = 700;
    /*
     * 直接使用FSUS_Servo的非阻塞定时角度指令，不依赖FSGP夹爪
     * 封装中未实现的负载检测。
     */
    // 张开尽量快；闭合略慢，避免夹爪接触物料时将其碰飞。
    // 夹爪张开动作的指令时长（ms），也是判定张开完成的等待时间的一部分。
    constexpr uint16_t GRIPPER_OPEN_INTERVAL_MS = 50;
    // 夹爪闭合动作的指令时长；越大闭合越慢、越不易碰飞物料。
    constexpr uint16_t GRIPPER_CLOSE_INTERVAL_MS = 50;
    // 夹爪动作时长之外再加的稳定余量；完成等待 = 指令时长 + 本值（抓取另加保持时间）。
    constexpr uint16_t GRIPPER_SETTLE_MS = 50;
    // 转盘抓取闭合后额外保持时间，等夹爪夹紧物料后才允许抬升。仅转盘抓取（CloseGripperGrasp）使用。
    constexpr uint16_t GRIPPER_GRASP_HOLD_MS = 250;
    // 载物盘舵机转速（度/秒），载物盘转到各料槽、收纳位时使用。
    constexpr float STORAGE_SPEED_DPS = 600.0F;
    // 载物盘最大转角约195°，600°/s下留足转动及机械稳定时间。
    // 载物盘转动指令发出后判定到位的固定等待时间，不查询舵机反馈。
    constexpr uint16_t STORAGE_SERVO_SETTLE_MS = 450;

    // 步进驱动偶尔会在到位瞬间短暂置位堵转标志，连续确认后才报错。
    // 连续收到多少帧堵转反馈才判定为真正堵转并让机构报故障。
    constexpr uint8_t STEPPER_FAULT_CONFIRM_COUNT = 3;
    // Lift Home和环切安全高度均使用两次独立到位帧。
    // 升降轴到达关键安全高度时，需连续确认的到位反馈帧数，避免未到位就开始横向动作。
    constexpr uint8_t LIFT_CRITICAL_CONFIRM_COUNT = 2;

    /*
     * 载物盘只有三个物料槽。任务可能出现四种颜色中的任意三种，因此
     * 槽位按“本批抓取顺序”分配，而不是把四种颜色固定映射到三个槽。
     */
    // 载物盘舵机角度（度），下标即料槽号；载物盘转到该角度时对应料槽正对机械臂交接位。
    constexpr float TRAY_SLOT_ANGLE[4] = {
        105.0F, // 0：收纳/出发位
        70.0F,  // 1：本批第一个物料
        -20.0F, // 2：本批第二个物料
        -110.0F // 3：本批第三个物料
    };

    // -------------------- 待实车微调的位置参数 --------------------
    // 坐标约定：升降轴数值越小位置越高（0为最顶）；伸缩轴数值越小越伸出。
    // 上电初始化位置，与比赛动作完成后的收纳位置相互独立。
    // 上电初始化时升降轴目标位置。
    constexpr float LIFT_INITIAL = 20.0F;
    // 上电初始化时伸缩轴目标位置（比EXTENSION_HOME更伸出）。
    constexpr float EXTENSION_INITIAL = 900.0F;
    // 上电初始化时底座目标角度（0.1°单位，0为初始朝向）。
    constexpr float BASE_INITIAL = 0.0F;

    // 升降安全高度：行驶、转底座、切换动作前后都先回到这里，是最常用的“抬起”位置。
    constexpr float LIFT_HOME = 10.0F;
    // 原料盘搜索和视觉追踪专用高度；对准完成后再下降到夹取高度。
    constexpr float LIFT_MATERIAL_VISION = 150.0F;
    // 粗加工区和暂存区视觉对准时降低升降轴，避免机构遮挡相机。
    constexpr float LIFT_STATION_VISION = 1000.0F;
    // 转盘（原料盘）夹取高度：视觉对准完成后下降到此处夹料。
    constexpr float LIFT_TURNTABLE = 600.0F;
    // 载物盘处取放料时的升降高度；与EXTENSION_TRAY_TRANSFER、BASE_TRAY_TRANSFER构成固定交接坐标。
    constexpr float LIFT_TRAY_TRANSFER = 320.0F;
    // 从圆环夹紧物料后先竖直上抬的距离，脱离物料后才允许返回储料盘。
    constexpr float RING_PICKUP_RELEASE_LIFT = 100.0F;
    // 粗加工第三件放完后直切第一个圆环时的安全高度。
    constexpr float LIFT_RING_SWITCH_CLEARANCE = 900.0F;
    // 任意圆环放料松爪后先上抬该距离，再允许机构返回或切环。
    constexpr float RING_PLACEMENT_RELEASE_LIFT = 100.0F;
    // 单个物料的高度，用于码垛：第N层放置高度 = 圆环lift - N × 本值。
    constexpr float MATERIAL_HEIGHT = 600.0F;

    // 伸缩轴收纳位（实车1500时完全收回）：行驶及转底座前后都收到这里。
    constexpr float EXTENSION_HOME = 1500.0F;
    // 从载物盘取放物料时的伸缩位置。
    constexpr float EXTENSION_TRAY_TRANSFER = 1620.0F;
    // 转盘（原料盘）取料的伸缩起始位；视觉微调时dy优先由伸缩轴在此基础上补偿。
    constexpr float EXTENSION_TURNTABLE = 1450.0F;

    // 行驶/收纳时的底座角度（0.1°单位，1800即180°车内收纳方向），工位动作完成后转到这里。
    constexpr float BASE_HOME = 1800.0F;
    // 从载物盘取放物料时的底座角度（0.1°单位，2750即275°）；也是圆环回盘的终点。
    constexpr float BASE_TRAY_TRANSFER = 2750.0F;
    // 转盘（原料盘）取料时的底座角度（0.1°单位，1800即180°）。
    constexpr float BASE_TURNTABLE = 1800.0F;

    /**
     * @brief 从底盘统一基准位置到某个圆环的机械臂点位。
     *
     * base、lift和extension分别对应底座、升降和伸缩轴的绝对目标值。
     * 粗加工区视觉只负责把整车对准统一基准，随后不再移动底盘。
     */
    struct RingPose
    {
        float base;      // 底座角度（0.1°单位）
        float lift;      // 升降高度（0.1mm单位，越小越高）；第二层放置时再减去MATERIAL_HEIGHT
        float extension; // 伸缩位置（0.1mm单位，越小越伸出）
    };

    /*
     * 粗加工区圆环点位表。
     * 下标0是无效占位；下标1~3直接对应任务码中的1~3号圆环。
     * 当前先沿用原有初值，现场标定时只修改对应圆环的一行。
     */
    constexpr RingPose ROUGH_RING_POSES[4] = {
        {BASE_TRAY_TRANSFER, 0.0F, 0.0F},
        {2226.0F, 1325.0F, 760.0F},  // 1号圆环
        {1823.0F, 1360.0F, 1605.0F}, // 2号圆环
        {1330.0F, 1400.0F, 1225.0F}, // 3号圆环
    };

    /*
     * 暂存区保持独立标定，避免今后调整粗加工区时影响码垛。
     * 当前数值与原逻辑一致，后续同样按圆环编号逐行修改。
     */
    constexpr RingPose FINAL_STORAGE_RING_POSES[4] = {
        {BASE_TRAY_TRANSFER, 0.0F, 0.0F},
        {2233.0F, 1310.0F, 795.0F},  // 1号圆环
        {1817.0F, 1340.0F, 1605.0F}, // 2号圆环
        {1325.0F, 1370.0F, 1220.0F}, // 3号圆环
        // {2226.0F, 1325.0F, 760.0F},  // 1号圆环
        // {1823.0F, 1360.0F, 1605.0F}, // 2号圆环
        // {1330.0F, 1400.0F, 1225.0F}, // 3号圆环

    };

    // -------------------- 非阻塞执行保护 --------------------
    // 步进驱动状态查询的最小间隔（ms），升降与伸缩共用串口，按此间隔轮流查询以避免占满总线。
    constexpr uint32_t STEPPER_POLL_MS = 25;
    // TTL状态查询丢帧后不能永久卡在Ask_State，超时后重发。
    // 状态查询发出后等待应答的超时时间（ms），超时视为丢帧并重发查询。
    constexpr uint32_t STEPPER_QUERY_RESPONSE_TIMEOUT_MS = 100;
    // 单个步进动作从下发到确认到位的总超时（ms），超时则机构报故障，防止卡死。
    constexpr uint32_t STEPPER_TIMEOUT_MS = 15000;

} // namespace mechanism_config
