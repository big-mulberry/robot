#pragma once

#include "robot_core/trajectory/cubic_trajectory.hpp" // 轨迹采样结果TrajectoryPoint

namespace robot_core::trajectory
{

// 单关节五次多项式轨迹
//
// p0, p1: 关节位置，rad
// t:      采样时间，s
// duration: 总运动时间，s
//
// 返回:
// position: rad
// velocity: rad/s
// acceleration: rad/s²
TrajectoryPoint interpolateQuintic(
    double p0,
    double p1,
    double t,
    double duration);

/**
 * @brief 为零边界速度、零边界加速度的五次轨迹规划满足约束的持续时间。
 * @param p0 起点位置，rad。
 * @param p1 终点位置，rad。
 * @param max_velocity 最大速度幅值，rad/s，必须有限且 > 0。
 * @param max_acceleration 最大加速度幅值，rad/s²，必须有限且 > 0。
 *
 * 令 D=abs(p1-p0)、s=t/T，轨迹为 p=p0+(p1-p0)*(10s³-15s⁴+6s⁵)。
 * 该固定曲线的峰值速度为 15D/(8T)，峰值加速度幅值为 10sqrt(3)D/(3T²)。
 * 要求峰值不超限，分别得到：
 *   Tv=15D/(8*max_velocity)；
 *   Ta=sqrt(10sqrt(3)D/(3*max_acceleration))。
 * 返回 max(Tv,Ta)，同时满足两种约束；通过拉长时间降低峰值，而非截断输出。
 * 对固定的归一化曲线，时间拉长 k 倍，速度降为 1/k，加速度降为 1/k²。
 *
 * @return 非零距离的所需时长，s；零距离返回 0，表示无需运动。
 * @note 仅适用于 v0=v1=a0=a1=0，不能直接用于任意边界状态的八参数重载。
 * 返回 0 时应由调用者处理静止状态，不能将零时长直接传给 interpolateQuintic。
 * @throws std::invalid_argument 输入非有限、约束非正或位移计算非有限。
 * @throws std::overflow_error 计算出的非零运动时长非有限或非正。
 */
double planQuinticDuration(
    double p0,
    double p1,
    double max_velocity,
    double max_acceleration);

/**
 * @brief 用六个边界条件确定单段五次曲线，采样得到位置、速度和加速度。
 * @param p0 起点位置，rad。
 * @param p1 终点位置，rad。
 * @param v0 起点速度，rad/s。
 * @param v1 终点速度，rad/s。
 * @param a0 起点加速度，rad/s²。
 * @param a1 终点加速度，rad/s²。
 * @param t 局部采样时间，s，允许有限的越界值。
 * @param duration 总时长 T，s，必须 > 0；所有参数必须有限。
 *
 * 五次多项式有六个系数，满足起终点各自的位置、速度和加速度共六个条件。
 * 使用归一化时间 s=t/T（进度0到1）：
 *   p(s)=b0+b1*s+b2*s²+b3*s³+b4*s⁴+b5*s⁵。
 * 因 ds/dt=1/T，实际速度=(dp/ds)/T，加速度=(d²p/ds²)/T²。
 * 因此 b0=p0、b1=v0*T、b2=a0*T²/2；不能保持时间多项式系数不变而直接换成s。
 * b3、b4、b5由终点s=1处的三个条件求解，保证终点位置、速度、加速度匹配。
 * 嵌套乘法只是多项式的等价写法，可减少乘法次数。
 *
 * 时间行为：
 * - t<0：返回 {p0,0,0}；t>T：返回 {p1,0,0}。
 * - t=0、t=T：保留真实多项式边界状态，不强制速度、加速度为零。
 * - 0<t<T：按多项式及其时间导数采样。
 * 边界数值通过公式计算，可能有微小浮点误差。
 * 四参数重载以 v0=v1=a0=a1=0 调用本函数，此时退化为零边界五次曲线。
 *
 * @note 本函数不自动规划时长或保证中间速度、加速度不超限。
 * 非零边界状态与区间外静止状态可能不连续，连续执行应由相邻轨迹段接管。
 * @throws std::invalid_argument 任一输入非有限或 duration<=0。
 */
TrajectoryPoint interpolateQuintic(
    double p0,
    double p1,
    double v0,
    double v1,
    double a0,
    double a1,
    double t,
    double duration);


//************************贝塞尔曲线 */


struct QuinticContinuousBounds
{
    double min_position;         // rad
    double max_position;         // rad
    double max_abs_velocity;     // rad/s
    double max_abs_acceleration; // rad/s²
    double max_abs_jerk;         // rad/s³
};

/**
 * @brief 计算五次轨迹在整个 [0, duration] 上的保守约束界。
 *
 * @note 返回的是连续时间保守上界，不一定是实际精确极值。
 */
QuinticContinuousBounds boundQuinticTrajectory(
    double p0,
    double p1,
    double v0,
    double v1,
    double a0,
    double a1,
    double duration);

//*********************五次轨迹的时间缩放 */
// 约束上限
struct QuinticKinematicLimits
{
    double max_velocity;     // rad/s
    double max_acceleration; // rad/s²
    double max_jerk;         // rad/s³
};

/**
 * @brief 计算五次轨迹满足运动学约束所需的时间缩放系数。
 *
 * @param bounds 原始轨迹的连续时间保守界。
 * @param limits 设备允许的速度、加速度、Jerk 上限。
 *
 * @return 时间缩放系数 k，满足 k >= 1。
 *
 * @throws std::invalid_argument
 *         输入非有限、上界为负、限制不大于零，
 *         或位置上下界不合法。
 *
 * @throws std::overflow_error
 *         缩放系数计算结果非有限。
 *
 * @note 本函数不修改轨迹，不负责位置软限位检查。
 * @note 返回值基于保守界，不一定是实际最优缩放系数。
 */
double computeQuinticTimeScale(
    const QuinticContinuousBounds& bounds,
    const QuinticKinematicLimits& limits);
}
