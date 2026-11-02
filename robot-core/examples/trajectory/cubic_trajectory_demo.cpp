#include "robot_core/trajectory/cubic_trajectory.hpp"
#include <Eigen/Core>
#include <cassert>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <iostream>
using namespace robot_core::trajectory;
void testC1Continuation()
{
    using namespace robot_core::trajectory;

    // 第一段：0 -> 1，终点速度 1.5 rad/s
    const CubicSegment first{
        0.0, 1.0, 0.0, 1.5, 1.0
    };

    const auto first_end = interpolateCubic(
        first.p0, first.p1,
        first.v0, first.v1,
        first.duration, first.duration);

    // 根据第一段终点状态，自动生成第二段
    const auto second = makeC1Continuation(
        first_end,
        3.0,  // 目标位置
        0.0,  // 目标速度
        1.0   // 第二段持续时间
    );

    const auto second_start = interpolateCubic(
        second.p0, second.p1,
        second.v0, second.v1,
        0.0, second.duration);

    const auto second_end = interpolateCubic(
        second.p0, second.p1,
        second.v0, second.v1,
        second.duration, second.duration);

    constexpr double eps = 1e-9;

    assert(std::abs(first_end.position - second_start.position) < eps);
    assert(std::abs(first_end.velocity - second_start.velocity) < eps);
    assert(std::abs(second_end.position - 3.0) < eps);
    assert(std::abs(second_end.velocity) < eps);

    std::cout << "First end:  "
              << first_end.position << ", "
              << first_end.velocity << ", "
              << first_end.acceleration << '\n';

    std::cout << "Second start: "
              << second_start.position << ", "
              << second_start.velocity << ", "
              << second_start.acceleration << '\n';

    std::cout << "[PASS] C1 continuation\n";
}

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

    const TwoSegmentCubicTrajectory trajectory{
        .first = {.p0=0, .p1=1, .v0=0, .v1=1.5, .duration=1},
        .second = {.p0=1, .p1=3, .v0=1.5, .v1=0, .duration=1}
    };
    const auto printReport = [](const char *label, const JunctionContinuity &report) {
        std::cout << label << " 误差(p,v,a)=(" << report.position_error << ", "
                  << report.velocity_error << ", " << report.acceleration_error << ")"
                  << " C0=" << (report.c0_continuous ? "PASS" : "FAIL")
                  << " C1=" << (report.c1_continuous ? "PASS" : "FAIL")
                  << " C2=" << (report.c2_continuous ? "PASS" : "FAIL") << '\n';
    };
    auto report=checkJunctionContinuity(trajectory);
    printReport("测试一：C1成立、C2不成立",report);
    require(near(report.position_error,0) && near(report.velocity_error,0) &&
            near(report.acceleration_error,6) && report.c0_continuous &&
            report.c1_continuous && !report.c2_continuous,"C1/C2 classification mismatch");

auto changed=trajectory;
    changed.second.p0=1.01;
    report=checkJunctionContinuity(changed);
    printReport("测试三：位置不连续",report);
    require(near(report.position_error,0.01) && !report.c0_continuous &&
            !report.c1_continuous && !report.c2_continuous,"Position discontinuity mismatch");
    changed=trajectory;
    changed.second.v0=1.2;
    report=checkJunctionContinuity(changed);
    printReport("测试四：速度不连续",report);
    require(near(report.velocity_error,0.3) && report.c0_continuous &&
            !report.c1_continuous && !report.c2_continuous,"Velocity discontinuity mismatch");

    // 真正C2连续的匀速两段，防止实现把所有案例都判断成C2失败。
    auto uniform=trajectory;
    uniform.first={0,1,1,1,1}; uniform.second={1,2,1,1,1};
    report=checkJunctionContinuity(uniform);
    require(report.c0_continuous && report.c1_continuous && report.c2_continuous,
            "C2 continuous case rejected");
    std::cout << "[PASS] 匀速拼接C2成立\n";

    const double nan=std::numeric_limits<double>::quiet_NaN();
    const double inf=std::numeric_limits<double>::infinity();
    for(double bad:{0.0,-1.0,nan,inf})
    {
        for(int segment=0;segment<2;++segment)
        {
            auto invalid=trajectory;
            if(segment==0) invalid.first.duration=bad;
            else invalid.second.duration=bad;
            expectInvalid("非法分段时长：连续性",[&]{(void)checkJunctionContinuity(invalid);});
        }
    }
    for(double bad:{-1.0,nan,inf})
    {
        for(int field=0;field<3;++field)
        {
            ContinuityTolerance tolerance;
            if(field==0) tolerance.position=bad;
            else if(field==1) tolerance.velocity=bad;
            else tolerance.acceleration=bad;
            expectInvalid("非法连续性容差",[&]{(void)checkJunctionContinuity(trajectory,tolerance);});
        }
    }
    report=checkJunctionContinuity(trajectory,{0,0,0});
    require(report.c0_continuous && report.c1_continuous && !report.c2_continuous,
            "Zero tolerances should be valid");
    std::cout << "[PASS] 零容差合法\n两段三次轨迹测试全部通过。\n";
    testC1Continuation();
    return 0;
}