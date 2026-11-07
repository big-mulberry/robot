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
    return interpolateQuintic(p0, p1, 0.0, 0.0, 0.0, 0.0, t, duration);
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

TrajectoryPoint interpolateQuintic(
    double p0, double p1,
    double v0, double v1,
    double a0, double a1,
    double t, double duration)
{
    // TODO 1：检查所有输入有限，duration > 0。
    if (!std::isfinite(p0) || !std::isfinite(p1) ||
        !std::isfinite(v0) || !std::isfinite(v1) ||
        !std::isfinite(a0) || !std::isfinite(a1) ||
        !std::isfinite(t) || !std::isfinite(duration) || duration <= 0.0)
    {
        throw std::invalid_argument("Inputs must be finite and duration positive.");
    }

    // TODO 2：越界静止；边界保留真实位置、速度和加速度。
    if (t < 0.0) return TrajectoryPoint{p0, 0.0, 0.0};
    if (t > duration) return TrajectoryPoint{p1, 0.0, 0.0};

    // TODO 3：使用归一化时间 s=t/T，计算六个边界条件对应的系数。
    // p(s)=b0+b1*s+b2*s²+b3*s³+b4*s⁴+b5*s⁵。
    // t=T*s：原时间多项式的 ci*t^i 变成 (ci*T^i)*s^i，因此 bi=ci*T^i。
    // 起点：p(0)=b0；v(0)=b1/T；a(0)=2*b2/T²。
    // 终点约束可写成三个剩余量：
    // P=p1-b0-b1-b2，V=v1*T-b1-2*b2，A=a1*T²-2*b2。
    // 求解 b3+b4+b5=P、3b3+4b4+5b5=V、6b3+12b4+20b5=A，得到：
    // b3=10P-4V+A/2，b4=-15P+7V-A，b5=6P-3V+A/2。
    // 下方是将P、V、A展开后的等价公式。
    const double T = duration;
    const double delta_p = p1 - p0;
    const double b0 = p0;
    const double b1 = v0 * T;
    const double b2 = 0.5 * a0 * T * T;
    const double b3 = 10.0 * delta_p - (6.0 * v0 + 4.0 * v1) * T
                      - (1.5 * a0 - 0.5 * a1) * T * T;
    const double b4 = -15.0 * delta_p + (8.0 * v0 + 7.0 * v1) * T
                      + (1.5 * a0 - a1) * T * T;
    const double b5 = 6.0 * delta_p - 3.0 * (v0 + v1) * T
                      - 0.5 * (a0 - a1) * T * T;

    // TODO 4：按链式法则，速度除以 T，加速度除以 T 两次。
    const double s = t / T;
    TrajectoryPoint result{};
    result.position = b0 + s * (b1 + s * (b2 + s * (b3 + s * (b4 + s * b5))));
    result.velocity = (b1 + s * (2.0 * b2 + s * (3.0 * b3
                       + s * (4.0 * b4 + s * 5.0 * b5)))) / T;
    result.acceleration = (2.0 * b2 + s * (6.0 * b3
                           + s * (12.0 * b4 + s * 20.0 * b5))) / T / T;
    return result;
}


QuinticContinuousBounds boundQuinticTrajectory(
    double p0,
    double p1,
    double v0,
    double v1,
    double a0,
    double a1,
    double duration)
{
    // TODO 1:
    // 检查所有输入有限，并要求 duration > 0。
    if (!std::isfinite(p0) || !std::isfinite(p1) ||
        !std::isfinite(v0) || !std::isfinite(v1) ||
        !std::isfinite(a0) || !std::isfinite(a1) ||
        !std::isfinite(duration) || duration <= 0.0)
    {
        throw std::invalid_argument("Inputs must be finite and duration positive.");
    }

    const double T = duration;
    const double T2 = T * T;
    if (!std::isfinite(T2))
    {
        throw std::overflow_error("Squared duration is not finite.");
    }

    // 五次位置曲线对应六个控制点
    std::array<double, 6> position_control{};

    // TODO 2:
    // 根据本节公式计算 P0～P5。
    // 注意检查计算结果是否有限，防止浮点溢出。
    position_control[0] = p0;
    position_control[1] = p0 + v0 * T / 5.0;
    position_control[2] = p0 + 2.0 * v0 * T / 5.0 + a0 * T2 / 20.0;
    position_control[3] = p1 - 2.0 * v1 * T / 5.0 + a1 * T2 / 20.0;
    position_control[4] = p1 - v1 * T / 5.0;
    position_control[5] = p1;
    for (double value : position_control)
    {
        if (!std::isfinite(value))
            throw std::overflow_error("Position control point is not finite.");
    }

    // 一阶导数：速度
    std::array<double, 5> velocity_control{};

    // 二阶导数：加速度
    std::array<double, 4> acceleration_control{};

    // 三阶导数：Jerk
    std::array<double, 3> jerk_control{};

    // TODO 3:
    // 计算速度控制点：
    // V[i] = 5 * (P[i+1] - P[i]) / T
    for(std::size_t i = 0; i<5; ++ i)
    {
        velocity_control[i] = 5.0 * (position_control[i + 1] - position_control[i]) / T;
        if (!std::isfinite(velocity_control[i]))
            throw std::overflow_error("velocity control point is not finite.");
    }
    // 计算加速度控制点：
    // A[i] = 4 * (V[i+1] - V[i]) / T
    for(std::size_t i = 0; i<4; ++ i)
    {
        acceleration_control[i] = 4.0 * (velocity_control[i + 1] - velocity_control[i]) / T;
        if (!std::isfinite(acceleration_control[i]))
            throw std::overflow_error("acceleration control point is not finite.");
    }
    // 计算 Jerk 控制点：
    // J[i] = 3 * (A[i+1] - A[i]) / T
    for(std::size_t i = 0; i<3; ++ i)
    {
        jerk_control[i] = 3.0 * (acceleration_control[i + 1] - acceleration_control[i]) / T;
        if (!std::isfinite(jerk_control[i]))
            throw std::overflow_error("jerk control point is not finite.");
    }
    // 每一步都注意结果是否有限。

    QuinticContinuousBounds bounds{};

    // TODO 4:
    // min_position = P0～P5 的最小值
    // max_position = P0～P5 的最大值
    auto [min_pos, max_pos] = std::minmax_element(position_control.begin(), position_control.end(), [&](double a, double b) {
        return a < b;
    });
    bounds.min_position = *min_pos;
    bounds.max_position = *max_pos;
    // max_abs_velocity     = max(abs(V[i]))
    bounds.max_abs_velocity = std::abs(*std::max_element(velocity_control.begin(), velocity_control.end(), [&](double a, double b) {
        return std::abs(a) < std::abs(b);
    }));
    // max_abs_acceleration = max(abs(A[i]))
    bounds.max_abs_acceleration = std::abs(*std::max_element(acceleration_control.begin(), acceleration_control.end(), [&](double a, double b) {
        return std::abs(a) < std::abs(b);
    }));
    // max_abs_jerk         = max(abs(J[i]))
    bounds.max_abs_jerk = std::abs(*std::max_element(jerk_control.begin(), jerk_control.end(), [&](double a, double b) {
        return std::abs(a) < std::abs(b);
    }));

    return bounds;
}

double computeQuinticTimeScale(
    const QuinticContinuousBounds& bounds,
    const QuinticKinematicLimits& limits)
{
    // TODO 1：输入校验
    //
    // bounds:
    // - min_position、max_position 必须有限
    // - min_position <= max_position
    // - 三个 max_abs_* 必须有限且 >= 0
    //
    // limits:
    // - 三个限制必须有限且 > 0
    if (!std::isfinite(bounds.min_position) || !std::isfinite(bounds.max_position) ||
        bounds.min_position > bounds.max_position ||
        !std::isfinite(bounds.max_abs_velocity) || bounds.max_abs_velocity < 0.0 ||
        !std::isfinite(bounds.max_abs_acceleration) || bounds.max_abs_acceleration < 0.0 ||
        !std::isfinite(bounds.max_abs_jerk) || bounds.max_abs_jerk < 0.0 ||
        !std::isfinite(limits.max_velocity) || limits.max_velocity <= 0.0 ||
        !std::isfinite(limits.max_acceleration) || limits.max_acceleration <= 0.0 ||
        !std::isfinite(limits.max_jerk) || limits.max_jerk <= 0.0)
    {
        throw std::invalid_argument("Bounds must be finite and ordered with non-negative magnitudes; limits must be finite and positive.");
    }

    // TODO 2：计算速度约束所需的缩放系数
    //
    // k_velocity = ?
    const auto k_velocity = (bounds.max_abs_velocity/limits.max_velocity);

    // TODO 3：计算加速度约束所需的缩放系数
    //
    const auto k_acceleration = std::sqrt(bounds.max_abs_acceleration/limits.max_acceleration);

    // TODO 4：计算 Jerk 约束所需的缩放系数
    //
    const auto k_jerk = std::cbrt(bounds.max_abs_jerk/limits.max_jerk);

    // TODO 5：取最终缩放系数
    //
    // k = max(1, k_velocity, k_acceleration, k_jerk)
    const auto k = std::max({1.0, k_velocity, k_acceleration, k_jerk});
    // 检查计算结果是否有限
    if (!std::isfinite(k_velocity) || !std::isfinite(k_acceleration) ||
        !std::isfinite(k_jerk) || !std::isfinite(k))
    {
        throw std::overflow_error("Time scale is not finite.");
    }

    // TODO 6：返回 k
    return k;
}

}