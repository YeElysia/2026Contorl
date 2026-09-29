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
    constexpr int16_t TARGET_DY_PX = 1;
    // 原料抓取允许更大的进入下降窗口。
    constexpr int16_t CENTER_TOLERANCE_PX = 25;
    // 旧项目约连续稳定50ms即下降；两帧可兼顾速度和单帧抗噪。
    constexpr uint8_t REQUIRED_STABLE_FRAMES = 2;
    constexpr uint8_t MIN_QUALITY = 30;

    // 识别结果按15~20FPS输出；超过约4帧未更新即视为目标陈旧。
    constexpr uint32_t TARGET_STALE_MS = 250;
    // 机构准备期间提前找料，只复用足够新的最后一帧。
    constexpr uint32_t PRIMED_OBSERVATION_MAX_AGE_MS = 200;
    // 运动结束时可复用刚拍到的高质量帧，省去等待下一相机周期。
    constexpr uint32_t MOVING_OBSERVATION_GRACE_MS = 120;
    /*
     * 原料盘持续旋转且相机无法同时看到全部物料。搜索窗口必须覆盖
     * 至少一整圈，否则目标颜色尚未转入可抓取范围就会退出识别。
     */
    constexpr uint32_t TARGET_SEARCH_TIMEOUT_MS = 60000;

    /*
     * 相机暂时看不到物料时，沿赛道方向执行由近到远的扩展搜索。
     * 该序列来自旧项目test_maix_align，覆盖路线基准前后100mm。
     * 每个点等待约2~3个识别帧；完整扫描两遍仍未发现时，
     * 将本件记为漏抓并继续任务，不阻塞整车流程。
     */
    constexpr float PICKUP_SEARCH_OFFSETS_MM[] = {
        0.0F,
        20.0F,
        -20.0F,
        40.0F,
        -40.0F,
        60.0F,
        -60.0F,
        80.0F,
        -80.0F,
        100.0F,
        -100.0F};
    constexpr uint8_t PICKUP_SEARCH_POSITION_COUNT =
        sizeof(PICKUP_SEARCH_OFFSETS_MM) /
        sizeof(PICKUP_SEARCH_OFFSETS_MM[0]);
    constexpr uint8_t PICKUP_SEARCH_MAX_PASSES = 2;
    constexpr uint32_t PICKUP_SEARCH_WAYPOINT_WAIT_MS = 180;

    /*
     * 图像误差到机构修正量的二维标定矩阵：
     *
     *   forwardDelta   = FORWARD_FROM_DX * dx + FORWARD_FROM_DY * dy
     *   extensionDelta = EXT_FROM_DX     * dx + EXT_FROM_DY     * dy
     *
     * 底盘前后单位为mm，伸缩单位为0.1mm。dy先由伸缩轴补偿；
     * 只有伸缩轴已到1500最大收缩位且仍需继续收缩时，
     * 才用底盘左移补偿剩余误差。
     */
    constexpr float FORWARD_FROM_DX = -0.3F;
    constexpr float FORWARD_FROM_DY = 0.0F;
    constexpr float EXTENSION_FROM_DX = 0.0F;
    constexpr float EXTENSION_FROM_DY = 4.0F;
    // 与圆环对齐标定符号一致：dy为正时左移。
    constexpr float LEFT_FROM_DY = -0.3F;

    // 远离中心时直接大步到达，进入精调区后限制单次修正量。
    constexpr int16_t FINE_ALIGNMENT_ZONE_PX = 24;
    constexpr float COARSE_FORWARD_MAX_DELTA_MM = 25.0F;
    constexpr float COARSE_EXTENSION_MAX_DELTA = 450.0F;
    constexpr float COARSE_LEFT_MAX_DELTA_MM = 25.0F;
    constexpr float FINE_FORWARD_MAX_DELTA_MM = 5.0F;
    constexpr float FINE_EXTENSION_MAX_DELTA = 60.0F;
    constexpr float FINE_LEFT_MAX_DELTA_MM = 5.0F;
    // 小于该值的修正收益低于下发和到位查询耗时，直接等待下一帧。
    constexpr float FORWARD_COMMAND_DEADBAND_MM = 1.0F;
    constexpr float RIGHT_COMMAND_DEADBAND_MM = 1.0F;
    constexpr float EXTENSION_COMMAND_DEADBAND = 5.0F;

    /*
     * 视觉闭环软限位只约束原料区跟随动作。
     * 不能把机械硬限位直接当作软件目标。
     */
    /*
     * 原料区路线基准保持在x=1200。
     * 完成后仍由状态机回到路线基准，不影响后续固定坐标移动。
     */
    constexpr float PICKUP_FORWARD_MIN_OFFSET_MM = -100.0F;
    constexpr float PICKUP_FORWARD_MAX_OFFSET_MM = 100.0F;
    constexpr float PICKUP_EXTENSION_MIN = 300.0F;
    constexpr float PICKUP_EXTENSION_MAX = 1500.0F;
    // right<0为左移；视觉微调禁止右移。
    constexpr float PICKUP_RIGHT_MIN_OFFSET_MM = -100.0F;
    constexpr float PICKUP_RIGHT_MAX_OFFSET_MM = 0.0F;

    // -------------------- 粗加工区圆环整车对准 --------------------
    // 参考new_project的圆环3定位，后续可按相机视野调整圆环编号。
    constexpr uint8_t ROUGH_RING_ID = 2;
    // 暂存区以2号位为统一视觉基准，第二轮再识别该位置上的颜色。
    constexpr uint8_t STORAGE_REFERENCE_RING_ID = 2;
    // 圆环和原料识别使用同一套相机光轴标定中心，避免重复配置漂移。
    constexpr int16_t RING_TARGET_DX_PX = TARGET_DX_PX;
    constexpr int16_t RING_TARGET_DY_PX = TARGET_DY_PX;
    /*
     * 圆环目标复用上方相机标定中心。实车识别值会在中心附近波动，
     * 底盘也无法稳定执行亚毫米修正，因此允许±1px，避免反复振荡。
     */
    constexpr int16_t RING_CENTER_TOLERANCE_PX = 1;
    constexpr int16_t RING_FINE_ALIGNMENT_ZONE_PX = 25;
    constexpr uint8_t RING_REQUIRED_STABLE_FRAMES = 30;
    constexpr uint8_t RING_MIN_QUALITY = 30;

    /*
     * 实车闭环标定结果：
     * 直接令小车前进/右移会使正的dx/dy继续增大，因此控制修正量
     * 必须取反。这里描述的是“消除误差所需的底盘运动”，不是图像
     * 中圆环所在的方向。
     */
    constexpr float RING_FORWARD_MM_PER_DX_PX = -0.3F;
    constexpr float RING_RIGHT_MM_PER_DY_PX = -0.3F;
    constexpr float RING_COARSE_MAX_MOVE_MM = 30.0F;
    constexpr float RING_FINE_MAX_MOVE_MM = 10.0F;
    // 15~20FPS下留出约4帧，避免短暂抖动误清稳定计数。
    constexpr uint32_t RING_TARGET_STALE_MS = 250;
    constexpr uint32_t RING_ALIGNMENT_TIMEOUT_MS = 12000;
    /*
     * 第二轮暂存区优先识别第一轮2号位物料颜色；若该物料漏抓或漏放，
     * 数秒内没有任何可靠颜色观测就切回仍然可见的固定2号圆环。
     */
    constexpr uint32_t STORAGE_COLOR_FALLBACK_MS = 3500;

    /*
     * 固定视觉地标居中时，底盘几何中心的实测世界坐标。
     * 路线中的ROUGH_ANCHOR/STORAGE_ANCHOR仍作为安全进退点；
     * 对齐成功后先重定位到下列作业点，离站路线会向场地中心撤离。
     */
    constexpr float ROUGH_ALIGNED_WORLD_X_MM = 1220.0F;
    constexpr float ROUGH_ALIGNED_WORLD_Y_MM = 310.0F;
    constexpr float STORAGE_ALIGNED_WORLD_X_MM = 2090.0F;
    constexpr float STORAGE_ALIGNED_WORLD_Y_MM = 1220.0F;

    /*
     * 防止错认地标后把全局位置跳到错误坐标。该门限覆盖路线进退点
     * 到视觉作业点约120mm的正常差值，并为累计里程误差保留余量。
     */
    constexpr float RING_WORLD_CORRECTION_MAX_ERROR_MM = 250.0F;
} // namespace vision_config
