#pragma once

namespace robot_core::trajectory {

// 单轴轨迹采样结果
struct TrajectoryPoint
{
    double position;      // 关节位置，rad
    double velocity;      // 关节速度，rad/s
    double acceleration;  // 关节加速度，rad/s²
};

// p0、p1：同一个关节的起点和终点位置，rad
// t：当前采样时间，s
// duration：运动总时间，s
TrajectoryPoint interpolateCubic(
    double p0,
    double p1,
    double t,
    double duration);

}