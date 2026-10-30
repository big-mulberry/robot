#pragma once
#include <cstdint>
#include <vector>
namespace robot_core::trajectory
{

enum class ProfileType
{
    Trapezoidal, // 梯形
    Triangular   // 三角形
};

// 梯形规划结果
struct TrapezoidalProfile
{
    double p0; // 起点位置
    double p1; // 终点位置

    // +1.0 表示正向运动，-1.0 表示反向运动。
    double direction;

    // 以下保存正的幅值。
    double acceleration;
    double peak_velocity;

    double acceleration_time; // 加速时段
    double cruise_time;       // 匀速
    double deceleration_time; // 减速
    double duration;          // 总时段

    ProfileType type;
};

// 采样结果
struct TrajectoryPoint
{
    double position;     // rad
    double velocity;     // rad/s
    double acceleration; // rad/s²
};

// TrapezoidalProfile 在对齐周期后的结果
struct CyclicTrapezoidalPlan
{
    // 已经完成时间缩放的轨迹。
    TrapezoidalProfile profile;

    // 控制周期，单位 s。
    double cycle_time;

    // 运动包含的时间间隔数量。
    std::uint64_t total_cycles;
};



// 梯形规划函数
TrapezoidalProfile planTrapezoidal(
    double p0,
    double p1,
    double max_velocity,
    double max_acceleration);

// 采样函数
TrajectoryPoint sampleTrapezoidal(
    const TrapezoidalProfile &profile,
    double t);



// 对齐周期后的梯形规划结果
CyclicTrapezoidalPlan alignToControlCycle(
    const TrapezoidalProfile &profile,
    double cycle_time);

TrajectoryPoint sampleAtCycle(
    const CyclicTrapezoidalPlan &plan,
    std::uint64_t cycle_index);



//* 多轴同步
// 时间缩放的重置轨迹
TrapezoidalProfile scaleToDuration(
    const TrapezoidalProfile& profile,
    double target_duration, double cycle_time);

// 多轴同步接口
std::vector<TrapezoidalProfile> synchronizeAxes(
    const std::vector<TrapezoidalProfile>& profiles, double cycle_time = 0.001);
}