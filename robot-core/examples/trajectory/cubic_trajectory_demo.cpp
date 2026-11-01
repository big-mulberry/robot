#include "robot_core/trajectory/cubic_trajectory.hpp"
#include <Eigen/Core>
#include <cassert>
#include <cmath>
#include <limits>
#include <stdexcept>
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

    const auto require = [](bool ok, const char *message) {
        if (!ok) throw std::runtime_error(message);
    };
    const auto near = [](double a, double b) { return std::abs(a-b) < 1e-9; };
    const auto checkBoundary = [&](const char *label, double v0, double v1) {
        const auto start = interpolateCubic(0,1,v0,v1,0,1);
        const auto end = interpolateCubic(0,1,v0,v1,1,1);
        require(near(start.position,0) && near(end.position,1) &&
                near(start.velocity,v0) && near(end.velocity,v1), "Boundary mismatch");
        std::cout << "[PASS] " << label << " 起点速度=" << start.velocity
                  << " 终点速度=" << end.velocity << '\n';
    };
    checkBoundary("零边界速度",0,0);
    checkBoundary("非零终点速度",0,1.5);
    checkBoundary("非零起点速度",1.5,0);
    for (int k=0;k<=100;++k) {
        const double time=k/100.0;
        const auto point=interpolateCubic(0,1,1,1,time,1);
        require(near(point.position,time) && near(point.velocity,1) && near(point.acceleration,0),
                "Constant-speed case mismatch");
        const auto old=interpolateCubic(0,1,time,1);
        const auto zero=interpolateCubic(0,1,0,0,time,1);
        require(near(old.position,zero.position) && near(old.velocity,zero.velocity) &&
                near(old.acceleration,zero.acceleration), "Overload mismatch");
    }
    std::cout << "[PASS] 匀速特例（101个采样点）、旧接口复用\n";
    const auto expectInvalid = [&](const char *label, auto action) {
        bool rejected=false;
        try { action(); } catch(const std::invalid_argument&) { rejected=true; }
        require(rejected,"Expected invalid_argument");
        std::cout << "[PASS] " << label << '\n';
    };
    expectInvalid("duration=0",[]{(void)interpolateCubic(0,1,0,0,0,0);});
    expectInvalid("v0=NaN",[]{(void)interpolateCubic(0,1,std::numeric_limits<double>::quiet_NaN(),0,0,1);});
    const auto before=interpolateCubic(0,1,-0.1,1);
    const auto after=interpolateCubic(0,1,1.1,1);
    require(before.position==0 && before.velocity==0 && before.acceleration==0 &&
            after.position==1 && after.velocity==0 && after.acceleration==0,
            "Legacy out-of-range behavior changed");
    std::cout << "[PASS] 旧接口越界静止行为保留\n";
    const auto checkState = [&](const char *label, const TrajectoryPoint &point,
                                double p, double v, double a) {
        std::cout << label << " 实际(p,v,a)=(" << point.position << ", "
                  << point.velocity << ", " << point.acceleration << ") 预期=("
                  << p << ", " << v << ", " << a << ")\n";
        require(near(point.position,p) && near(point.velocity,v) &&
                near(point.acceleration,a), "Boundary state mismatch");
        std::cout << "[PASS] " << label << '\n';
    };
    checkState("起点之前",interpolateCubic(0,1,1,1,-0.1,1),0,0,0);
    checkState("终点之后",interpolateCubic(0,1,0,1.5,1.1,1),1,0,0);
    // a2=1.5、a3=-0.5，终点加速度为 2*a2+6*a3=0。
    checkState("恰好终点",interpolateCubic(0,1,0,1.5,1,1),1,1.5,0);
    return 0;
}