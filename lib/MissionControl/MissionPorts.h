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
 * - storagePositions[i]：该物料在暂存区的圆环编号。
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
 * @brief 工位视觉对准接口。
 *
 * 原料逐物料抓取使用独立的IGraspVisionProvider。本接口负责
 * 粗加工区、暂存区等工位的整车到站对准。
 */
class IAlignmentProvider
{
public:
    virtual ~IAlignmentProvider() = default;
    virtual bool start(Station station, uint8_t round) = 0;
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
    virtual bool start(
        StationTask task,
        uint8_t round,
        const BatchMission &batch) = 0;
    virtual void update() = 0;
    virtual AsyncResult result() const = 0;
    virtual void cancel() = 0;
};
