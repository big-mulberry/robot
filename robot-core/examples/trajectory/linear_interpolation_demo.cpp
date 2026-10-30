#include "robot_core/trajectory/linear_interpolation.hpp"
#include <Eigen/Core>
#include <cassert>
#include <iostream>
using namespace robot_core::trajectory;
int main()
{
    // 基坐标系 {B} 下的起点和终点，单位：m
    Eigen::Vector3d p_start(0.0, 0.0, 0.0);
    Eigen::Vector3d p_goal(1.0, 2.0, 3.0);

    // 运动总时间，单位：s
    double duration = 2.0;

    // 测试运动中点
    auto result = robot_core::trajectory::interpolateLinear(
        p_start, p_goal, 1.0, duration);

    std::cout << "Interpolated position at t=1s: " << result.transpose() << std::endl;

    // t=1s 时，期望位置为 (0.5, 1.0, 1.5)m
    Eigen::Vector3d expected(0.5, 1.0, 1.5);

    assert(result.isApprox(expected, 1e-12));

    assert(interpolateLinear(p_start, p_goal, -1.0, duration).isApprox(p_start, 1e-12));
    assert(interpolateLinear(p_start, p_goal, 3.0, duration).isApprox(p_goal, 1e-12));
    bool caught = false;

    try
    {
        interpolateLinear(p_start, p_goal, 1.0, 0.0);
    }
    catch (const std::invalid_argument &)
    {
        caught = true;
    }

    assert(caught);

    return 0;
}