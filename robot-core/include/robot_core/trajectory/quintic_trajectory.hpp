#pragma once

#include "robot_core/trajectory/cubic_trajectory.hpp" // 轨迹采样结果TrajectoryPoint

namespace robot_core::trajectory {

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

// 在速度和加速度约束的情况下规划五次多项式轨迹的持续时间
double planQuinticDuration(
    double p0,
    double p1,
    double max_velocity,
    double max_acceleration);

}

