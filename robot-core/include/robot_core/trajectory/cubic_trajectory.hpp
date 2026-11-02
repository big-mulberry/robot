#pragma once

namespace robot_core::trajectory
{

// 单轴轨迹采样结果
struct TrajectoryPoint
{
    double position;     // 关节位置，rad
    double velocity;     // 关节速度，rad/s
    double acceleration; // 关节加速度，rad/s²
};

// p0、p1：同一个关节的起点和终点位置，rad
// t：当前采样时间，s
// duration：运动总时间，s
TrajectoryPoint interpolateCubic(
    double p0,
    double p1,
    double t,
    double duration);

/**
 * @brief 根据起终点位置和速度进行单段三次插值，返回位置、速度和加速度。
 * @param p0 起点位置，rad。
 * @param p1 终点位置，rad。
 * @param v0 起点速度，rad/s。
 * @param v1 终点速度，rad/s。
 * @param t 当前局部采样时间，s；允许有限的越界时间。
 * @param duration 轨迹总时间 T，s，必须大于 0。
 *
 * 时间范围与返回行为（p, v, a）：
 * - 越界 t < 0：返回 (p0, 0, 0)，保持起点静止。
 * - 起点 t == 0：返回多项式起点状态 (p0, v0, 2*a2)。
 * - 区间内 0 < t < T：计算三次多项式及其一、二阶导数。
 * - 终点 t == T：返回多项式终点状态 (p1, v1, 2*a2 + 6*a3*T)。
 * - 越界 t > T：返回 (p1, 0, 0)，保持终点静止。
 *
 * 多项式 p(t) = p0 + v0*t + a2*t^2 + a3*t^3，其中：
 * a2 = (3*(p1-p0) - (2*v0+v1)*T) / T^2；
 * a3 = (-2*(p1-p0) + (v0+v1)*T) / T^3。
 * 边界属于多项式采样区间，速度和加速度不会被强制置零；
 * 终点位置、速度通过公式计算，可能有微小浮点误差。
 *
 * @throws std::invalid_argument 任一输入非有限，或 duration <= 0。
 * @note 非零边界速度与区间外静止状态之间存在速度跳变；连续运动应由相邻轨迹段接管。
 */
TrajectoryPoint interpolateCubic(
    double p0,
    double p1,
    double v0,
    double v1,
    double t,
    double duration);


 // TRAJ-11B: 两段三次轨迹边界条件；采样复用六参数 interpolateCubic。
struct CubicSegment
{
    double p0;       // rad
    double p1;       // rad
    double v0;       // rad/s
    double v1;       // rad/s
    double duration; // s
};

struct TwoSegmentCubicTrajectory
{
    CubicSegment first;
    CubicSegment second;
};

struct ContinuityTolerance
{
    double position{1e-9};     // rad
    double velocity{1e-9};     // rad/s
    double acceleration{1e-9}; // rad/s²
};

struct JunctionContinuity
{
    double position_error{0.0};
    double velocity_error{0.0};
    double acceleration_error{0.0};
    bool c0_continuous{false};
    bool c1_continuous{false};
    bool c2_continuous{false};
};

/**
 * 比较第一段终点与第二段起点，C1 包含 C0，C2 包含 C1。
 * 完成后：容差必须有限且非负，否则抛 invalid_argument。
 * 当前骨架抛 std::logic_error，待完成 TODO 后替换。
 */
JunctionContinuity checkJunctionContinuity(
    const TwoSegmentCubicTrajectory& trajectory,
    const ContinuityTolerance& tolerance = {});



// 根据前一段的终点状态，生成下一段三次轨迹的边界条件。
CubicSegment makeC1Continuation(
    const TrajectoryPoint& previous_end,
    double target_position,
    double target_velocity,
    double duration);








}