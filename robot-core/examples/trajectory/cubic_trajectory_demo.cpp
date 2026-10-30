#include "robot_core/trajectory/cubic_trajectory.hpp"
#include <Eigen/Core>
#include <cassert>
#include <iostream>
using namespace robot_core::trajectory;
int main()
{
    const double  p0 = 0.0;
    const double  p1 = 1.0;
    const double duration = 2.0;

    const double t = 1;
    // 测试运动中点
    auto result = robot_core::trajectory::interpolateCubic(
        p0, p1, t, duration);

    std::cout << "Interpolated position at t=" << t << "s: " << result.position << std::endl;
    std::cout << "Interpolated velocity at t=" << t << "s: " << result.velocity << std::endl;
    std::cout << "Interpolated acceleration at t=" << t << "s: " << result.acceleration << std::endl;

    return 0;
}