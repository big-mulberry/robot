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
    return interpolateCubic(p0, p1, 0.0, 0.0, t, duration);
}
TrajectoryPoint interpolateCubic(double p0, double p1, double v0, double v1, double t, double duration)
{
    // TODO 1:
    // 验证所有输入是否为有限数。
    // duration > 0
    if (!std::isfinite(p0) || !std::isfinite(p1) ||
        !std::isfinite(v0) || !std::isfinite(v1) ||
        !std::isfinite(t) || !std::isfinite(duration) ||
        duration <= 0.0)
    {
        throw std::invalid_argument(
            "Inputs must be finite, duration positive.");
    }

    if (t < 0.0)
    {
        return TrajectoryPoint{p0, 0.0, 0.0};
    }

    if (t > duration)
    {
        return TrajectoryPoint{p1, 0.0, 0.0};
    }

    const double T = duration;
    const double delta_p = p1 - p0;

    const double a0 = p0;
    const double a1 = v0;

    // TODO 2:
    // 根据本节推导，计算 a2、a3。
    const double a2 = (3 * delta_p - (2 * v0 + v1) * T) / (T * T);
    const double a3 = (-2 * delta_p + (v0 + v1) * T) / (T * T * T);

    // TODO 3:
    // 根据三次多项式及其一、二阶导数，
    // 计算 position、velocity、acceleration。
    TrajectoryPoint result{};
    result.position = a0 + a1 * t + a2 * t * t + a3 * t * t * t;
    result.velocity = a1 + 2 * a2 * t + 3 * a3 * t * t;
    result.acceleration = 2 * a2 + 6 * a3 * t;

    // TODO 4:
    // 返回 TrajectoryPoint。
    return result;
}
}