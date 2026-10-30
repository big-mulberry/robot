
#pragma once
#include <vector>
#include "trapezoidal_profile.hpp"
namespace robot_core::trajectory
{
// 单轴规划参数
struct AxisMotion
{
    double p0;                // 起点
    double p1;                // 终点
    double max_velocity;      // 最大速度幅值
    double max_acceleration;  // 最大加速度幅值
};

// 已完成与控制周期对齐的多轴公共相位轨迹
struct CyclicPhaseSynchronizedPlan
{
    // 每个轴的起点、终点和运动约束
    std::vector<AxisMotion> axes;

    // 已完成周期对齐的公共相位轨迹
    CyclicTrapezoidalPlan phase_plan;
};

// 仅负责映射；要求相位有限且 0 <= position <= 1，不执行钳位。
std::vector<TrajectoryPoint> mapPhaseToAxes(
    const std::vector<AxisMotion>& axes, const TrajectoryPoint& phase);
// 轴组
struct PhaseSynchronizedPlan
{
    std::vector<AxisMotion> axes;

    // 公共相位轨迹，用一次梯形规划即可得出公共的加速段，匀速段和减速段
    TrapezoidalProfile phase_profile;
};

// 规划多轴公共相位轨迹
PhaseSynchronizedPlan planPhaseSynchronized(
    const std::vector<AxisMotion>& axes);

// 根据公共相位，计算所有轴在t时刻状态
std::vector<TrajectoryPoint> samplePhaseSynchronized(
    const PhaseSynchronizedPlan& plan,
    double t);

/**
 * @brief 规划周期对齐后的多轴公共相位轨迹
 *
 * @param axes 所有轴的运动参数
 * @param cycle_time 控制周期，单位 s
 *
 * @return 周期对齐后的多轴公共相位规划结果
 */
CyclicPhaseSynchronizedPlan planCyclicPhaseSynchronized(
    const std::vector<AxisMotion>& axes,
    double cycle_time);

/**
 * @brief 按控制周期序号采样所有轴的轨迹状态
 *
 * @param plan 周期对齐后的多轴公共相位规划结果
 * @param cycle_index 周期序号，从 0 开始
 *
 * @return 所有轴的位置、速度和加速度
 */
std::vector<TrajectoryPoint> sampleCyclicPhaseSynchronized(
    const CyclicPhaseSynchronizedPlan& plan,
    std::uint64_t cycle_index);
}