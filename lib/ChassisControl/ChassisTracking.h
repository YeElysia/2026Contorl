#pragma once

#include <math.h>

/**
 * @brief 底盘直线路径跟踪的纯数学部分，不依赖Arduino，便于主机测试。
 *
 * 路径是起点S到目标T的线段，单位向量u = (T-S)/|T-S|，
 * 左法向n = (-u.y, u.x)。
 */
namespace chassis_tracking
{
    struct Vec2
    {
        float x;
        float y;
    };

    struct PathError
    {
        // 沿u方向到目标的剩余距离；越过目标后为负。
        float along;
        // 相对路径的横偏，位于路径左侧（n方向）为正。
        float cross;
    };

    inline PathError pathError(
        const Vec2 &start,
        const Vec2 &direction,
        const Vec2 &target,
        const Vec2 &pose)
    {
        const float toTargetX = target.x - pose.x;
        const float toTargetY = target.y - pose.y;
        const float fromStartX = pose.x - start.x;
        const float fromStartY = pose.y - start.y;
        return {
            toTargetX * direction.x + toTargetY * direction.y,
            direction.x * fromStartY - direction.y * fromStartX};
    }

    inline float clampMagnitude(float value, float limit)
    {
        return fmaxf(-limit, fminf(value, limit));
    }

    /**
     * @brief 沿程S曲线轨迹发生器的状态，位置从起点沿路径计。
     */
    struct JerkState
    {
        float position;
        float velocity;
        float acceleration;
    };

    inline void integrateJerk(JerkState &state, float jerk, float seconds)
    {
        if (seconds <= 0.0F)
            return;
        state.position +=
            state.velocity * seconds +
            state.acceleration * seconds * seconds * 0.5F +
            jerk * seconds * seconds * seconds / 6.0F;
        state.velocity +=
            state.acceleration * seconds + jerk * seconds * seconds * 0.5F;
        state.acceleration += jerk * seconds;
    }

    /**
     * @brief 从速度v≥0、加速度a出发，在|加速度|≤A、|加加速度|≤J下
     * 把速度和加速度都降到零所需的最短距离。
     */
    inline float jerkStopDistance(
        float velocity,
        float acceleration,
        float maximumAcceleration,
        float jerk)
    {
        JerkState state = {0.0F, velocity, acceleration};
        // 减速度已足够：只收回减速度，速度就会先过零，取首次到零处。
        if (acceleration < 0.0F &&
            velocity <= acceleration * acceleration / (2.0F * jerk))
        {
            const float seconds =
                (-acceleration -
                 sqrtf(fmaxf(
                     acceleration * acceleration - 2.0F * jerk * velocity,
                     0.0F))) /
                jerk;
            integrateJerk(state, jerk, seconds);
            return state.position;
        }

        // 梯形减速度：a→-peak，保持，再-peak→0，速度恰好减到零。
        float peak = maximumAcceleration;
        float holdSeconds =
            (velocity + acceleration * acceleration / (2.0F * jerk) -
             peak * peak / jerk) /
            peak;
        if (holdSeconds < 0.0F)
        {
            peak = sqrtf(
                jerk * velocity + acceleration * acceleration * 0.5F);
            holdSeconds = 0.0F;
        }
        integrateJerk(state, -jerk, (acceleration + peak) / jerk);
        integrateJerk(state, 0.0F, holdSeconds);
        integrateJerk(state, jerk, -state.acceleration / jerk);
        return state.position;
    }

    /**
     * @brief 轨迹发生器前进一个控制周期，不越过goal、不超速、不反向。
     *
     * 每周期依次尝试加加速度+J、0、-J，取第一个仍能在goal前刹停的；
     * 都不行时全力制动。控制周期越长，末端越需要吸附到goal，
     * 否则离散误差会让发生器停在离goal很近却再也起不了步的位置。
     */
    inline void jerkStep(
        JerkState &state,
        float goal,
        float maximumVelocity,
        float maximumAcceleration,
        float jerk,
        float seconds)
    {
        if (seconds <= 0.0F)
            return;

        bool advanced = false;
        const float jerks[3] = {jerk, 0.0F, -jerk};
        for (const float candidateJerk : jerks)
        {
            const float nextAcceleration =
                state.acceleration + candidateJerk * seconds;
            if (fabsf(nextAcceleration) > maximumAcceleration)
                continue;

            JerkState next = state;
            integrateJerk(next, candidateJerk, seconds);
            if (next.velocity < 0.0F)
                continue;
            const float peakVelocity =
                next.acceleration > 0.0F
                    ? next.velocity +
                          next.acceleration * next.acceleration /
                              (2.0F * jerk)
                    : next.velocity;
            if (peakVelocity > maximumVelocity)
                continue;
            if (next.position +
                    jerkStopDistance(
                        next.velocity,
                        next.acceleration,
                        maximumAcceleration,
                        jerk) >
                goal)
                continue;

            state = next;
            advanced = true;
            break;
        }

        if (!advanced)
        {
            const float brakingJerk = fmaxf(
                -jerk,
                (-maximumAcceleration - state.acceleration) / seconds);
            integrateJerk(state, brakingJerk, seconds);
            if (state.velocity < 0.0F)
            {
                state.velocity = 0.0F;
                state.acceleration = 0.0F;
            }
        }

        const float snapDistance =
            fmaxf(0.05F, 2.0F * jerk * seconds * seconds * seconds);
        const bool stoppedPastGoal =
            state.position > goal && state.velocity <= 0.0F;
        const bool creepingAtGoal =
            goal - state.position < snapDistance &&
            state.velocity < jerk * seconds * seconds;
        if (stoppedPastGoal || creepingAtGoal)
            state = {goal, 0.0F, 0.0F};
    }

    inline bool jerkFinished(const JerkState &state, float goal)
    {
        return state.position == goal &&
               state.velocity == 0.0F &&
               state.acceleration == 0.0F;
    }

    /**
     * @brief 横偏比例修正，输出沿n方向的速度。
     *
     * 容差外保留最低速度，否则小横偏在低增益下收敛过慢。
     */
    inline float crossCorrection(
        float cross,
        float gainPerSecond,
        float maximumSpeed,
        float minimumSpeed,
        float tolerance)
    {
        float speed = clampMagnitude(-cross * gainPerSecond, maximumSpeed);
        if (fabsf(cross) > tolerance &&
            fabsf(speed) < minimumSpeed)
        {
            speed = copysignf(fminf(minimumSpeed, maximumSpeed), -cross);
        }
        return speed;
    }

    /**
     * @brief 边转边走时的有效剩余距离：平移进度不超过IMU实测的转向进度。
     *
     * 越过目标（along<0）时照常返回负值往回拉。
     */
    inline float yawGatedRemaining(
        float along,
        float lengthMm,
        float remainingYawDeg,
        float totalYawDeg)
    {
        if (along <= 0.0F || totalYawDeg <= 0.0F)
            return along;
        const float progress = fmaxf(
            0.0F,
            fminf(1.0F, 1.0F - fabsf(remainingYawDeg) / totalYawDeg));
        return fmaxf(0.0F, along - lengthMm * (1.0F - progress));
    }
} // namespace chassis_tracking
