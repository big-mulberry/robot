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

JunctionContinuity checkJunctionContinuity(
    const TwoSegmentCubicTrajectory& trajectory,
    const ContinuityTolerance& tolerance)
{
    // TODO 3：
    // 检查三个容差：
    // 1. 必须有限；
    // 2. 必须 >= 0。
    if (!std::isfinite(tolerance.position) ||
        !std::isfinite(tolerance.velocity) ||
        !std::isfinite(tolerance.acceleration) ||
        tolerance.position < 0.0 || tolerance.velocity < 0.0 ||
        tolerance.acceleration < 0.0)
    {
        throw std::invalid_argument("Continuity tolerances must be finite and non-negative.");
    }

    const auto first_end = interpolateCubic(
        trajectory.first.p0,
        trajectory.first.p1,
        trajectory.first.v0,
        trajectory.first.v1,
        trajectory.first.duration,
        trajectory.first.duration);

    const auto second_start = interpolateCubic(
        trajectory.second.p0,
        trajectory.second.p1,
        trajectory.second.v0,
        trajectory.second.v1,
        0.0,
        trajectory.second.duration);

    JunctionContinuity report;

    // TODO 4：
    // 使用 std::abs() 计算三种误差。
    //
    // 然后设置：
    // C0 = 位置误差满足容差
    // C1 = C0 && 速度误差满足容差
    // C2 = C1 && 加速度误差满足容差
    //
    // 不要只比较速度就认定 C1 成立。
    report.position_error = std::abs(first_end.position - second_start.position);
    report.velocity_error = std::abs(first_end.velocity - second_start.velocity);
    report.acceleration_error = std::abs(first_end.acceleration - second_start.acceleration);

    report.c0_continuous = report.position_error <= tolerance.position;
    report.c1_continuous = report.c0_continuous &&
                           report.velocity_error <= tolerance.velocity;
    report.c2_continuous = report.c1_continuous &&
                           report.acceleration_error <= tolerance.acceleration;

    return report;
}

CubicSegment makeC1Continuation(
    const TrajectoryPoint& previous_end,
    double target_position,
    double target_velocity,
    double duration)
{
    // 1. 验证所有输入有限，duration > 0。
    if (!std::isfinite(previous_end.position) ||
        !std::isfinite(previous_end.velocity) ||
        !std::isfinite(previous_end.acceleration) ||
        !std::isfinite(target_position) || !std::isfinite(target_velocity) ||
        !std::isfinite(duration) || duration <= 0.0)
    {
        throw std::invalid_argument("Inputs must be finite and duration positive.");
    }

    // 2. 继承上一段终点的位置和速度；C1 不约束加速度连续。
    // 3. 构造并返回 CubicSegment。
    return CubicSegment{
        .p0 = previous_end.position,
        .p1 = target_position,
        .v0 = previous_end.velocity,
        .v1 = target_velocity,
        .duration = duration
    };
}

}
