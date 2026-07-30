#pragma once

#include "MissionPorts.h"

/**
 * @brief 工位到站对准的临时直通实现。
 *
 * 原料抓取已经使用MaixPro闭环，但粗加工区和暂存区的整车
 * 到站对准尚未接入。替换本类不会影响任务状态机和机械臂模块。
 */
class PassThroughAlignmentProvider : public IAlignmentProvider
{
public:
    bool start(Station, uint8_t) override
    {
        _result = AsyncResult::Succeeded;
        return true;
    }

    void update() override {}

    AsyncResult result() const override
    {
        return _result;
    }

    void cancel() override
    {
        _result = AsyncResult::Idle;
    }

private:
    AsyncResult _result = AsyncResult::Idle;
};
