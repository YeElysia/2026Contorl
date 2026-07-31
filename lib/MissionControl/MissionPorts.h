#pragma once

#include <Arduino.h>

/**
 * @brief 所有异步模块统一使用的执行结果。
 *
 * MissionController只依赖这些抽象接口，不直接包含二维码、MaixCam
 * 或机械臂驱动头文件，因此更换模块或通信协议不会影响任务状态机。
 */
enum class AsyncResult : uint8_t
{
    Idle,
    Running,
    Succeeded,
    Failed
};

enum class Station : uint8_t
{
    Material,
    RoughProcessing,
    Storage
};

enum class StationTask : uint8_t
{
    CollectMaterial,
    RoughProcessing,
    StoreFinishedProduct
};

enum class TravelDestination : uint8_t
{
    Scan,
    Material,
    RoughProcessing,
    Storage,
    Home
};

enum class MaterialColor : uint8_t
{
    Red = 1,
    Yellow = 2,
    Blue = 3,
    Green = 4
};

constexpr uint8_t BATCH_COUNT = 2;
constexpr uint8_t MATERIALS_PER_BATCH = 3;

/**
 * @brief 一批三个物料的完整任务数据。
 *
 * 三个数组使用相同下标：
 * - colors[i]：第i个搬运物料的颜色；
 * - roughPositions[i]：该物料在粗加工区的圆环编号；
 * - storagePositions[i]：该物料在暂存区的圆环编号。第一批复制任务
 *   码第二组；第二批根据第一批同色物料的位置计算，用于同色码垛。
 */
struct BatchMission
{
    uint8_t colors[MATERIALS_PER_BATCH] = {};
    uint8_t roughPositions[MATERIALS_PER_BATCH] = {};
    uint8_t storagePositions[MATERIALS_PER_BATCH] = {};
};

struct MissionPlan
{
    BatchMission batches[BATCH_COUNT];
};

/**
 * @brief 二维码任务数据提供者。
 *
 * 颜色编号：1=红、2=黄、3=蓝、4=绿。
 */
class IMissionDataProvider
{
public:
    virtual ~IMissionDataProvider() = default;
    virtual void start() = 0;
    virtual void update() = 0;
    virtual AsyncResult result() const = 0;
    virtual const MissionPlan &plan() const = 0;
    virtual const char *rawText() const = 0;
};

/**
 * @brief 一次工位视觉对准请求。
 *
 * referenceColor仅用于第二轮暂存区：其值是第一轮放在2号位的颜色。
 * 由任务状态机根据任务码计算，视觉模块不依赖二维码数据结构。
 */
struct AlignmentRequest
{
    Station station = Station::Material;
    uint8_t round = 0;
    uint8_t referenceColor = 0;
};

/**
 * @brief 工位视觉对准接口。
 *
 * 原料逐物料抓取使用独立的IGraspVisionProvider。本接口负责
 * 粗加工区、暂存区等工位的整车到站对准。
 */
class IAlignmentProvider
{
public:
    virtual ~IAlignmentProvider() = default;
    virtual bool start(const AlignmentRequest &request) = 0;
    virtual void update() = 0;
    virtual AsyncResult result() const = 0;
    virtual void cancel() = 0;
};

/**
 * @brief 工位机构任务接口。
 *
 * batch包含当前轮的颜色顺序、粗加工位置和暂存位置。
 * 机械臂控制器不需要再次解释二维码字符串。
 */
class IStationTaskExecutor
{
public:
    virtual ~IStationTaskExecutor() = default;
    virtual bool ready() const = 0;
    virtual const char *faultMessage() const = 0;

    /**
     * @brief 在底盘行驶期间将机构异步收回安全运输位。
     *
     * 调用成功只表示动作已经启动，完成状态仍通过result()查询。
     * 下一工位开始动作前，任务状态机必须等待该动作执行完成。
     */
    virtual bool prepareForTravel(
        TravelDestination destination) = 0;

    virtual bool start(
        StationTask task,
        uint8_t round,
        const BatchMission &batch) = 0;
    /**
     * @brief 推进机构任务。
     *
     * 底盘连续行驶时可禁止阻塞式反馈读取。机械动作命令仍会立即下发，
     * 只把舵机到位确认延后到停车后，避免打断软件产生的STEP脉冲。
     */
    virtual void update(bool allowBlockingFeedback = true) = 0;
    virtual AsyncResult result() const = 0;
    virtual void cancel() = 0;
};
