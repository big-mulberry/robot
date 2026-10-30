#include "robot_core/trajectory/phase_synchronized_trajectory.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>
namespace robot_core::trajectory
{

PhaseSynchronizedPlan planPhaseSynchronized(
    const std::vector<AxisMotion> &axes)
{
    if (axes.empty())
    {
        throw std::invalid_argument("axes must not be empty");
    }


    // TODO 1：验证各关节输入是否有限、约束是否合法
    for (const auto &axis : axes)
    {
        if (!std::isfinite(axis.p0) || !std::isfinite(axis.p1) ||
            !std::isfinite(axis.max_velocity) || !std::isfinite(axis.max_acceleration))
        {
            throw std::invalid_argument("Axis inputs must be finite.");
        }
        if (axis.max_velocity <= 0.0 || axis.max_acceleration <= 0.0)
        {
            throw std::invalid_argument("Axis velocity and acceleration limits must be positive.");
        }
        const double delta = axis.p1 - axis.p0;
        if (!std::isfinite(delta))
        {
            throw std::overflow_error("Axis displacement is not finite.");
        }
        if (delta != 0.0)
        {
            const double vs = axis.max_velocity / std::abs(delta);
            const double as = axis.max_acceleration / std::abs(delta);
            if (!std::isfinite(vs) || !std::isfinite(as) || vs <= 0.0 || as <= 0.0)
            {
                throw std::overflow_error("Phase limits overflowed or underflowed.");
            }
        }
    }

    // TODO 2：计算每个关节的 delta_q
    std::vector<double> axes_delta_q;
    for (const auto &axis_motion : axes)
    {
        axes_delta_q.push_back(std::abs(axis_motion.p1 - axis_motion.p0));
    }

    // TODO 3：排除静止关节，计算公共相位速度上限
    // TODO 4：计算公共相位加速度上限

    double vSMax = std::numeric_limits<double>::max();
    double aSMax = std::numeric_limits<double>::max();
    bool hasValid = false;

    for (std::size_t i = 0; i < axes.size(); ++i)
    {
        auto delta_q = axes_delta_q[i];
        if (delta_q != 0)
        {
            vSMax = std::min(vSMax, axes[i].max_velocity / delta_q);
            aSMax = std::min(aSMax, axes[i].max_acceleration / delta_q);
            if (!hasValid) hasValid = true;
        }
    }

    // TODO 5：处理所有关节均静止的情况
    if (!hasValid)
    {
        TrapezoidalProfile stationary{};
        stationary.type = ProfileType::Triangular;
        return PhaseSynchronizedPlan{axes, stationary};
    }
    // TODO 6：调用 planTrapezoidal(0, 1, vs_max, as_max)
    const auto phaseProfile = planTrapezoidal(0, 1, vSMax, aSMax);

    // TODO 7：构造并返回 PhaseSynchronizedPlan
    return PhaseSynchronizedPlan{axes, phaseProfile};
}

std::vector<TrajectoryPoint> mapPhaseToAxes(
    const std::vector<AxisMotion> &axes, const TrajectoryPoint &phase)
{
    if (!std::isfinite(phase.position) || !std::isfinite(phase.velocity) ||
        !std::isfinite(phase.acceleration) || phase.position < 0.0 || phase.position > 1.0)
    {
        throw std::invalid_argument("Phase must be finite with position in [0, 1].");
    }
    if (axes.empty())
        throw std::invalid_argument("axes must not be empty");

    std::vector<TrajectoryPoint> result;
    result.reserve(axes.size());
    for (const auto &axis : axes)
    {
        if (!std::isfinite(axis.p0) || !std::isfinite(axis.p1) ||
            !std::isfinite(axis.max_velocity) || !std::isfinite(axis.max_acceleration) ||
            axis.max_velocity <= 0.0 || axis.max_acceleration <= 0.0)
            throw std::invalid_argument("Axis inputs must be finite with positive limits.");

        const double delta_q = axis.p1 - axis.p0;
        if (!std::isfinite(delta_q))
            throw std::overflow_error("Axis displacement is not finite.");
        const double position = phase.position == 0.0 ? axis.p0 :
                                phase.position == 1.0 ? axis.p1 :
                                axis.p0 + delta_q * phase.position;
        const double velocity = delta_q * phase.velocity;
        const double acceleration = delta_q * phase.acceleration;
        if (!std::isfinite(position) || !std::isfinite(velocity) || !std::isfinite(acceleration))
            throw std::overflow_error("Mapped axis state is not finite.");
        result.push_back({position, velocity, acceleration});
    }
    return result;
}

std::vector<TrajectoryPoint> samplePhaseSynchronized(
    const PhaseSynchronizedPlan &plan,
    double time)
{
    // TODO 1：检查 time 是否为有限数值
    if (!std::isfinite(time))
    {
        throw std::invalid_argument("Sample time must be finite.");
    }

    // TODO 2：调用 sampleTrapezoidal() 获取公共相位状态
    const auto &phaseProfile = plan.phase_profile;
    const auto &axes = plan.axes;
    // 所有轴静止时直接保持目标状态，不依赖零时长相位的采样行为。
    if (phaseProfile.duration == 0.0)
    {
        std::vector<TrajectoryPoint> result;
        result.reserve(axes.size());
        for (const auto &axis : axes)
        {
            result.push_back(TrajectoryPoint{axis.p1, 0.0, 0.0});
        }
        return result;
    }

    TrajectoryPoint phase = sampleTrapezoidal(phaseProfile, time);

    if (!std::isfinite(phase.position) || !std::isfinite(phase.velocity) ||
        !std::isfinite(phase.acceleration) || phase.position < 0.0 || phase.position > 1.0)
    {
        throw std::invalid_argument("Phase must be finite with position in [0, 1].");
    }

    // 到达或超过终点时，直接使用各轴终点，避免浮点误差。
    if (time >= phaseProfile.duration)
    {
        std::vector<TrajectoryPoint> result;
        result.reserve(axes.size());
        for (const auto &axis : axes)
        {
            result.push_back(TrajectoryPoint{axis.p1, 0.0, 0.0});
        }
        return result;
    }
    // TODO 3：遍历每个关节，计算 delta_q
    // TODO 4：计算 position、velocity、acceleration
    std::vector<TrajectoryPoint> result;
    result.reserve(axes.size());
    for (const auto &axis : axes)
    {
        const auto delta_q = axis.p1 - axis.p0;
        const double position = axis.p0 + delta_q * phase.position;
        result.push_back(TrajectoryPoint{
            position,
            delta_q * phase.velocity,
            delta_q * phase.acceleration});
    }
    // TODO 5：返回所有关节的 TrajectoryPoint
    return result;
}

CyclicPhaseSynchronizedPlan planCyclicPhaseSynchronized(
    const std::vector<AxisMotion>& axes,
    double cycle_time)
{
    // 1. 规划多轴公共相位轨迹
    auto original_plan = planPhaseSynchronized(axes);

    // 2. 将公共相位轨迹对齐到控制周期
    auto cyclic_phase = alignToControlCycle(
        original_plan.phase_profile,
        cycle_time);

    // 3. 组合规划结果
    return {
        std::move(original_plan.axes),
        std::move(cyclic_phase)
    };
}


std::vector<TrajectoryPoint> sampleCyclicPhaseSynchronized(
    const CyclicPhaseSynchronizedPlan& plan,
    std::uint64_t cycle_index)
{
    // TODO 1：采样当前周期的公共相位
    auto phasePoint = sampleAtCycle(plan.phase_plan, cycle_index);

    // TODO 2：将公共相位映射到所有轴
    return mapPhaseToAxes(plan.axes, phasePoint);
}

} // namespace robot_core::trajectory