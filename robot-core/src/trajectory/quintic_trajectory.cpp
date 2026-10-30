#include "robot_core/trajectory/quintic_trajectory.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace robot_core::trajectory
{

TrajectoryPoint interpolateQuintic(
    double p0,
    double p1,
    double t,
    double duration)
{
    // TODO 1：
    // 检查 p0、p1、t、duration 是否有限。
    // duration 必须 > 0。
    // 无效参数抛出 std::invalid_argument。
    if (!(std::isfinite(p0) && std::isfinite(p1) && std::isfinite(t) && std::isfinite(duration) && duration > 0))
    {
        throw std::invalid_argument("Invalid input parameters.");
    }

    // TODO 2：
    // 将 t 限制在 [0, duration]。
    const double clamped_t = std::clamp(t, 0.0, duration);

    // TODO 3：
    // 计算归一化时间 s。
    const double s = clamped_t / duration;

    // TODO 4：
    // 计算关节位移。
    const double delta_p = p1 - p0;

    TrajectoryPoint result{};

    // TODO 5：
    // 使用五次多项式计算位置。
    result.position = p0 + delta_p * (6 * s * s * s * s * s - 15 * s * s * s * s + 10 * s * s * s);

    // TODO 6：
    // 计算速度。
    result.velocity = delta_p * (30 * s * s * s * s - 60 * s * s * s + 30 * s * s) / duration;

    // TODO 7：
    // 计算加速度。
    result.acceleration = delta_p * (120 * s * s * s - 180 * s * s + 60 * s) / (duration * duration);

    // TODO 8：
    // 对运动区间外的时间：
    // t < 0：保持起点位置，速度和加速度为 0。
    // t > duration：保持终点位置，速度和加速度为 0。
    //
    // 注意 t == 0 和 t == duration
    // 属于有效运动边界，不需要特殊处理。
    if (t < 0.0)
    {
        result.position = p0;
        result.velocity = 0.0;
        result.acceleration = 0.0;
    }
    else if (t > duration)
    {
        result.position = p1;
        result.velocity = 0.0;
        result.acceleration = 0.0;
    }
    return result;
}

double planQuinticDuration(
    double p0,
    double p1,
    double max_velocity,
    double max_acceleration)
{
    // TODO 1:
    // 检查所有输入是否为有限数。
    // max_velocity 和 max_acceleration 必须 > 0。
    // 无效参数抛出 std::invalid_argument。
    if (!(std::isfinite(p0) && std::isfinite(p1) && std::isfinite(max_velocity) && std::isfinite(max_acceleration) && max_velocity > 0 && max_acceleration > 0))
    {
        throw std::invalid_argument("Invalid input parameters.");
    }

    // TODO 2:
    // 计算位移绝对值。
    const double distance = std::abs(p1 - p0);

    // TODO 3:
    // 如果位移为 0，直接返回 0。
    // 这里约定：零位移代表不需要执行运动。
    if (distance == 0)
    {
        return 0;
    }
    if (!std::isfinite(distance))
    {
        throw std::invalid_argument("Invalid displacement.");
    }
    // TODO 4:
    // 计算速度约束要求的最小时间。
    const double Tv = 15.0 * distance / (8.0 * max_velocity);

    // TODO 5:
    // 计算加速度约束要求的最小时间。
    const double Ta = std::sqrt((10 * std::sqrt(3) * distance) / (3.0 * max_acceleration));

    // TODO 6:
    // 取较大的运动时间并返回。
    const double duration = std::max(Tv, Ta);

    if (!std::isfinite(duration) || duration <= 0.0)
    {
        throw std::overflow_error("Invalid trajectory duration.");
    }

    return duration;
}

}