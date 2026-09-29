#pragma once

/**
 * @brief 底盘航向保持增强策略
 * 
 * 解决问题：
 * 1. 直线行驶时的小角度累积偏移
 * 2. 斜移时航向修正能力受限
 * 3. 不同运动模式需要不同的修正策略
 * 
 * 使用方法：
 * 在 chassis_config.h 中定义宏 USE_ENHANCED_HEADING_CONTROL
 * 以启用增强策略，否则使用原始简单 P 控制。
 */

#include <Arduino.h>

namespace chassis_heading_enhancement
{
    /**
     * @brief 运动模式分类
     * 
     * 不同运动模式采用不同的航向控制策略：
     * - 直线运动：允许更大的修正比例
     * - 斜移运动：受短行程轮速度约束
     * - 原地旋转：专用 PD 控制器
     */
    enum class MotionMode : uint8_t
    {
        Stopped,        // 静止
        StraightLine,   // 直线（前进/后退/左移/右移）
        Diagonal,       // 斜向移动
        Rotating        // 原地旋转
    };

    /**
     * @brief 根据前进和右移分量判断运动模式
     */
    inline MotionMode classifyMotion(float forwardMm, float rightMm)
    {
        const float absForward = fabsf(forwardMm);
        const float absRight = fabsf(rightMm);

        if (absForward < 1.0F && absRight < 1.0F)
            return MotionMode::Stopped;

        // 判断是否为直线运动（一个方向占主导，比例 > 3:1）
        if (absForward > absRight * 3.0F || absRight > absForward * 3.0F)
            return MotionMode::StraightLine;

        return MotionMode::Diagonal;
    }

    /**
     * @brief 增强型航向误差修正计算
     * 
     * @param errorDeg 航向误差（目标 - 当前）
     * @param mode 当前运动模式
     * @param maxTranslationSpeed 当前平移的最大轮速
     * @param minWheelSpeed 四个运动轮中最小的计划速度
     * @param kp 比例增益（steps/s per degree）
     * @param maxRatio 最大修正比例
     * @return 修正速度（steps/s）
     */
    inline float calculateHeadingCorrection(
        float errorDeg,
        MotionMode mode,
        float maxTranslationSpeed,
        float minWheelSpeed,
        float kp,
        float maxRatio)
    {
        // P 控制基础修正量
        float correction = errorDeg * kp;

        // 根据运动模式调整修正上限
        float effectiveMaxRatio = maxRatio;
        if (mode == MotionMode::StraightLine)
        {
            // 直线运动允许更大的修正比例（提升到 1.5 倍）
            effectiveMaxRatio *= 1.5F;
        }

        float maxCorrection = maxTranslationSpeed * effectiveMaxRatio;

        // 关键改进：在斜移时，不能让修正量超过最短行程轮速度，
        // 但对于直线运动，可以放宽这个约束
        if (mode == MotionMode::Diagonal && minWheelSpeed > 0.1F)
        {
            // 斜移时严格限制，防止短行程轮反转
            maxCorrection = min(maxCorrection, minWheelSpeed * 0.8F);
        }
        else if (mode == MotionMode::StraightLine)
        {
            // 直线时使用更宽松的限制（90% 而不是 80%）
            maxCorrection = min(maxCorrection, minWheelSpeed * 0.9F);
        }

        return constrain(correction, -maxCorrection, maxCorrection);
    }

    /**
     * @brief 航向偏移积分补偿
     * 
     * 对于持续的小角度偏移（如左右轮直径差异），
     * 可以累积一个前馈补偿量。
     * 
     * 注意：这是实验性功能，可能引入振荡，谨慎启用。
     */
    class HeadingIntegrator
    {
    public:
        HeadingIntegrator(float ki, float maxIntegral)
            : _ki(ki), _maxIntegral(maxIntegral), _integral(0.0F)
        {
        }

        void reset()
        {
            _integral = 0.0F;
        }

        float update(float errorDeg, float deltaTimeS)
        {
            // 只在误差较小时累积积分，避免大误差时积分饱和
            if (fabsf(errorDeg) < 3.0F)
            {
                _integral += errorDeg * deltaTimeS;
                _integral = constrain(_integral, -_maxIntegral, _maxIntegral);
            }
            else
            {
                // 大误差时衰减积分
                _integral *= 0.9F;
            }

            return _integral * _ki;
        }

    private:
        float _ki;
        float _maxIntegral;
        float _integral;
    };

    /**
     * @brief 自适应航向控制增益
     * 
     * 根据误差大小动态调整 Kp：
     * - 小误差（< 1°）：高增益，快速修正
     * - 中误差（1-3°）：正常增益
     * - 大误差（> 3°）：降低增益，防止超调
     */
    inline float adaptiveGain(float errorDeg, float nominalKp)
    {
        const float absError = fabsf(errorDeg);

        if (absError < 1.0F)
        {
            // 小误差时提高增益 20%
            return nominalKp * 1.2F;
        }
        else if (absError < 3.0F)
        {
            // 中等误差保持标称增益
            return nominalKp;
        }
        else
        {
            // 大误差时降低增益 30%，防止超调
            return nominalKp * 0.7F;
        }
    }

    /**
     * @brief 速度相关的航向控制调整
     * 
     * 低速时增加修正比例，高速时降低修正比例
     */
    inline float velocityScaledRatio(
        float currentSpeed,
        float maxSpeed,
        float nominalRatio)
    {
        const float speedRatio = currentSpeed / maxSpeed;

        if (speedRatio < 0.3F)
        {
            // 低速时（< 30% 最大速度）增加修正能力
            return nominalRatio * 1.3F;
        }
        else if (speedRatio > 0.8F)
        {
            // 高速时（> 80% 最大速度）减少修正，保持稳定
            return nominalRatio * 0.9F;
        }

        return nominalRatio;
    }

} // namespace chassis_heading_enhancement
