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

TrajectoryPoint sampleTwoSegmentCubic(
    const TwoSegmentCubicTrajectory& trajectory,
    double global_time)
{
    // TODO(TRAJ-11B-1): 验证 global_time、两个 duration 有限且 duration > 0；
    // 相加后的 total_duration 也必须有限。无效输入抛 std::invalid_argument。
    // TODO(TRAJ-11B-2): 处理全局时间越界静止、第一段/第二段选择；
    // 第二段将 global_time 转换为局部时间，拼接时刻使用第一段。
    // 对选中的 segment 复用现有接口，不重复实现多项式：
    // interpolateCubic(segment.p0, segment.p1, segment.v0, segment.v1,
    //                  local_time, segment.duration);
    (void)trajectory;
    (void)global_time;
    throw std::logic_error("TRAJ-11B: sampleTwoSegmentCubic TODO");
}

JunctionContinuity checkJunctionContinuity(
    const TwoSegmentCubicTrajectory& trajectory,
    const ContinuityTolerance& tolerance)
{
    // TODO(TRAJ-11B-3): 验证三个容差有限且 >= 0，否则抛 invalid_argument。
    // 复用六参数 interpolateCubic，分别采样 first 的 t=duration
    // 和 second 的 t=0；不可使用越界静止状态替代真实边界。
    // TODO(TRAJ-11B-4): 用 std::abs 计算位置、速度、加速度误差，
    // 按各自单位的容差判断 C0、C1、C2；高阶连续必须包含低阶连续。
    (void)trajectory;
    (void)tolerance;
    throw std::logic_error("TRAJ-11B: checkJunctionContinuity TODO");
}

}
