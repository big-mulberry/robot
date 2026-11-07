#include "robot_core/trajectory/quintic_trajectory.hpp"
#include <Eigen/Core>
#include <cassert>
#include <iostream>
#include <string>
#include <iomanip>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <array>
#include <algorithm>
using namespace robot_core::trajectory;
int main()
{
    const double  p0 = 0.0;
    const double  p1 = 1.0;
    const double duration = 2.0;

    const double t = 1;
    // 测试运动中点
    auto result = robot_core::trajectory::interpolateQuintic(
        p0, p1, t, duration);

    std::cout << "Interpolated position at t=" << t << "s: " << result.position << std::endl;
    std::cout << "Interpolated velocity at t=" << t << "s: " << result.velocity << std::endl;
    std::cout << "Interpolated acceleration at t=" << t << "s: " << result.acceleration << std::endl;

    const auto require = [](bool ok, const char *message) {
        if (!ok) throw std::runtime_error(message);
    };
    const auto near = [](double a, double b) { return std::abs(a-b) < 1e-9; };
    const auto checkState = [&](const char *label, const TrajectoryPoint &actual,
                                double p, double v, double a) {
        std::cout << label << " 实际(p,v,a)=(" << actual.position << ", "
                  << actual.velocity << ", " << actual.acceleration << ") 预期=("
                  << p << ", " << v << ", " << a << ")\n";
        require(near(actual.position,p) && near(actual.velocity,v) && near(actual.acceleration,a),
                "Quintic state mismatch");
        std::cout << "[PASS] " << label << '\n';
    };
    // 四参数目前是委托调用，同时用原始闭式公式检查，避免只比较同一实现。
    for (int k=0;k<=100;++k)
    {
        const double time=2.0*k/100.0;
        const double s=time/2.0;
        const auto old=interpolateQuintic(0,1,time,2);
        const auto full=interpolateQuintic(0,1,0,0,0,0,time,2);
        require(near(old.position,full.position) && near(old.velocity,full.velocity) &&
                near(old.acceleration,full.acceleration), "Overload mismatch");
        require(near(full.position,10*std::pow(s,3)-15*std::pow(s,4)+6*std::pow(s,5)) &&
                near(full.velocity,(30*s*s-60*std::pow(s,3)+30*std::pow(s,4))/2) &&
                near(full.acceleration,(60*s-180*s*s+120*std::pow(s,3))/4),
                "Zero-boundary analytic regression failed");
    }
    std::cout << "[PASS] 零边界速度、加速度：101点重载对比及原公式验证\n";
    checkState("非零起点速度",interpolateQuintic(0,1,0.7,0,0,0,0,2),0,0.7,0);
    checkState("非零终点速度",interpolateQuintic(0,1,0,0.8,0,0,2,2),1,0.8,0);
    checkState("非零起点加速度",interpolateQuintic(0,1,0,0,0.4,0,0,2),0,0,0.4);
    checkState("非零终点加速度",interpolateQuintic(0,1,0,0,0,-0.3,2,2),1,0,-0.3);

    // 第一段0->1；第二段继承真实终点的全部三种状态，时长不同。
    const auto first_end=interpolateQuintic(0,1,0,0.8,0,0.4,1,1);
    const auto second_start=interpolateQuintic(first_end.position,3,
        first_end.velocity,0,first_end.acceleration,0,0,2);
    checkState("C2拼接：第一段终点",first_end,1,0.8,0.4);
    checkState("C2拼接：第二段起点",second_start,
               first_end.position,first_end.velocity,first_end.acceleration);
    checkState("第二段目标状态",interpolateQuintic(first_end.position,3,
        first_end.velocity,0,first_end.acceleration,0,2,2),3,0,0);

    const auto expectInvalid = [&](const char *label, auto action) {
        bool rejected=false;
        try { action(); } catch (const std::invalid_argument &) { rejected=true; }
        require(rejected,"Expected invalid_argument was not thrown");
        std::cout << "[PASS] " << label << '\n';
    };
    const double nan=std::numeric_limits<double>::quiet_NaN();
    const double inf=std::numeric_limits<double>::infinity();
    for (double bad:{0.0,-1.0,nan,inf})
    {
        expectInvalid("非法duration：八参数",[&]{(void)interpolateQuintic(0,1,0,0,0,0,0,bad);});
        expectInvalid("非法duration：四参数",[&]{(void)interpolateQuintic(0,1,0,bad);});
    }
    // 遍历全部八个参数，逐个验证NaN与无穷大被拒绝。
    for (double bad:{nan,inf})
    {
        for (std::size_t field=0;field<8;++field)
        {
            std::array<double,8> input{0,1,0,0,0,0,0.5,1};
            input[field]=bad;
            expectInvalid("非有限参数",[&]{(void)interpolateQuintic(
                input[0],input[1],input[2],input[3],input[4],input[5],input[6],input[7]);});
        }
    }
    checkState("起点之前静止",interpolateQuintic(2,5,0.7,0.8,0.4,-0.3,-0.1,2),2,0,0);
    checkState("终点之后静止",interpolateQuintic(2,5,0.7,0.8,0.4,-0.3,2.1,2),5,0,0);
    checkState("恰好起点保留边界状态",interpolateQuintic(2,5,0.7,0.8,0.4,-0.3,0,2),2,0.7,0.4);
    checkState("恰好终点保留边界状态",interpolateQuintic(2,5,0.7,0.8,0.4,-0.3,2,2),5,0.8,-0.3);
    std::cout << "五次插值测试全部通过。\n";
    // 观察案例：净位移仅0.01rad，但起终点速度均为1rad/s。
    // 固定时长1s、边界加速度为零；插值满足边界，但不保证位置单调。
    {
        const double small_p0 = 0.0;
        const double small_p1 = 0.01;
        const double boundary_velocity = 1.0;
        const double T = 1.0;
        double min_position = small_p0;
        double max_position = small_p0;
        double min_velocity = boundary_velocity;
        double max_acceleration_magnitude = 0.0;
        std::cout << "\n=== 观察：小位移、大边界速度 ===\n"
                  << "p0=0rad, p1=0.01rad, v0=v1=1rad/s, a0=a1=0, T=1s\n"
                  << "所需平均速度只有0.01rad/s；观察曲线如何满足两端1rad/s。\n"
                  << std::fixed << std::setprecision(6)
                  << std::setw(10) << "t(s)"
                  << std::setw(16) << "p(rad)"
                  << std::setw(16) << "v(rad/s)"
                  << std::setw(16) << "a(rad/s^2)"
                  << "  现象\n";
        for (int k = 0; k <= 40; ++k)
        {
            const double time = T * k / 40.0;
            const auto point = interpolateQuintic(small_p0, small_p1,
                boundary_velocity, boundary_velocity, 0.0, 0.0, time, T);
            min_position = std::min(min_position, point.position);
            max_position = std::max(max_position, point.position);
            min_velocity = std::min(min_velocity, point.velocity);
            max_acceleration_magnitude = std::max(max_acceleration_magnitude,
                                                  std::abs(point.acceleration));
            std::cout << std::setw(10) << time
                      << std::setw(16) << point.position
                      << std::setw(16) << point.velocity
                      << std::setw(16) << point.acceleration;
            if (point.position < small_p0 || point.position > small_p1)
                std::cout << "  超出起终点区间";
            if (point.velocity < 0.0)
                std::cout << "  反向运动";
            std::cout << '\n';
        }
        std::cout << "采样位置范围：[" << min_position << ", " << max_position << "]rad\n"
                  << "采样最小速度：" << min_velocity << "rad/s\n"
                  << "采样最大加速度幅值：" << max_acceleration_magnitude << "rad/s²\n"
                  << "以上是41个采样点的统计，不代表连续曲线的精确极值。\n";
    }
    // 连续时间保守上界：各项独立记录，失败后继续，便于查看全部问题。
    int bounds_failures = 0;
    const auto checkBound = [&](const char *name, double actual, double expected) {
        const bool passed = std::isfinite(actual) && std::abs(actual-expected) < 1e-9;
        std::cout << (passed ? "[PASS] " : "[FAIL] ") << name
                  << " 实际=" << actual << " 预期=" << expected << '\n';
        if (!passed) ++bounds_failures;
    };
    const auto checkBounds = [&](const char *label, const QuinticContinuousBounds &actual,
                                 const QuinticContinuousBounds &expected) {
        std::cout << "\n=== " << label << " ===\n";
        checkBound("min_position",actual.min_position,expected.min_position);
        checkBound("max_position",actual.max_position,expected.max_position);
        checkBound("max_abs_velocity",actual.max_abs_velocity,expected.max_abs_velocity);
        checkBound("max_abs_acceleration",actual.max_abs_acceleration,expected.max_abs_acceleration);
        checkBound("max_abs_jerk",actual.max_abs_jerk,expected.max_abs_jerk);
    };
    checkBounds("上界测试1：匀速运动",boundQuinticTrajectory(0,1,1,1,0,0,1),
                {0,1,1,0,0});
    checkBounds("上界测试2：标准零边界轨迹",boundQuinticTrajectory(0,1,0,0,0,0,1),
                {0,1,5,20,120});
    std::cout << "实际峰值速度为1.875rad/s，保守上界应为5rad/s。\n";
    checkBounds("上界测试3：小位移、大边界速度",boundQuinticTrajectory(0,0.01,1,1,0,0,1),
                {-0.39,0.4,3.95,19.8,118.8});

    std::cout << "\n=== 上界测试4：异常输入 ===\n";
    const auto rejectBounds = [&](const char *label, const std::array<double,7> &values,
                                  bool expect_overflow) {
        bool passed = false;
        try {
            (void)boundQuinticTrajectory(values[0],values[1],values[2],values[3],
                                         values[4],values[5],values[6]);
        }
        catch (const std::invalid_argument &) { passed = !expect_overflow; }
        catch (const std::overflow_error &) { passed = expect_overflow; }
        catch (const std::exception &e) { std::cout << "异常类型不符：" << e.what() << '\n'; }
        std::cout << (passed ? "[PASS] " : "[FAIL] ") << label << '\n';
        if (!passed) ++bounds_failures;
    };
    rejectBounds("duration=0",{0,1,0,0,0,0,0},false);
    rejectBounds("duration<0",{0,1,0,0,0,0,-1},false);
    const std::array<const char*,7> names{"p0","p1","v0","v1","a0","a1","duration"};
    for (double bad : {std::numeric_limits<double>::quiet_NaN(),
                       std::numeric_limits<double>::infinity(),
                       -std::numeric_limits<double>::infinity()})
    {
        for (std::size_t i=0;i<names.size();++i)
        {
            std::array<double,7> values{0,1,0,0,0,0,1};
            values[i]=bad;
            const std::string label=std::string("拒绝非有限参数 ")+names[i];
            rejectBounds(label.c_str(),values,false);
        }
    }
    const double huge=std::numeric_limits<double>::max();
    rejectBounds("时长平方溢出",{0,1,0,0,0,0,huge},true);
    rejectBounds("位置控制点计算溢出",{0,1,huge,0,0,0,2},true);
    rejectBounds("速度控制点计算溢出",{-huge,huge,0,0,0,0,1},true);
    rejectBounds("加速度控制点计算溢出",{0,1e306,0,0,0,0,0.1},true);
    rejectBounds("jerk控制点计算溢出",{0,1e304,0,0,0,0,0.1},true);
    std::cout << "\n上界测试失败项数：" << bounds_failures << '\n';
    return bounds_failures == 0 ? 0 : 1;
}