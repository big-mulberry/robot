#pragma once
#include <Eigen/Core>

namespace robot_core::trajectory {


// 线性插值
Eigen::Vector3d interpolateLinear(
    const Eigen::Vector3d& p_start,
    const Eigen::Vector3d& p_goal,
    double t,
    double duration);

}