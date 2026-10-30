#include "robot_core/trajectory/cubic_trajectory.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace robot_core::trajectory
{

TrajectoryPoint interpolateCubic(
    double p0,
    double p1,
    double t,
    double duration)
{
    // TODO 1：
    // 检查 p0、p1、t、duration 是否为有限数。
    // 检查 duration 是否大于 0。
    // 参数不合法时抛出 std::invalid_argument。
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
    // 计算位移 delta_p。
    const double delta_p = p1 - p0;

    TrajectoryPoint result{};

    // TODO 5：
    // 根据三次多项式计算位置。
    result.position = p0 + delta_p * (3 * s * s - 2 * s * s * s);

    // TODO 6：
    // 计算速度。
    result.velocity = delta_p * (6 * s - 6 * s * s) / duration;

    // TODO 7：
    // 计算加速度。
    result.acceleration = delta_p * (6 - 12 * s) / (duration * duration);

    // TODO 8：
    // 处理运动区间外的静止状态：
    // t < 0 或 t > duration 时，
    // 位置保持端点，速度和加速度均为 0。
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

}