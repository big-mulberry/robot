#include "robot_core/trajectory/trapezoidal_profile.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <iomanip>
#include <iostream>
#include <vector>

namespace
{
using namespace robot_core::trajectory;
TrapezoidalProfile printPlan(const char *label, double p0, double p1,
                             double max_velocity, double max_acceleration)
{
    using namespace robot_core::trajectory;
    const auto profile = planTrapezoidal(p0, p1, max_velocity, max_acceleration);

    std::cout << "\n=== " << label << " ===\n"
              << "起点 / 终点: " << profile.p0 << " / " << profile.p1 << '\n'
              << "方向 (+1 正向 / -1 反向 / 0 静止): " << profile.direction << '\n'
              << "规划类型: "
              << (profile.type == ProfileType::Trapezoidal ? "梯形" : "三角形")
              << (profile.duration == 0.0 ? "（退化为静止）" : "") << '\n'
              << "速度上限 / 加速度上限: " << max_velocity << " / " << max_acceleration << '\n'
              << "峰值速度幅值: " << profile.peak_velocity << '\n'
              << "加速度幅值: " << profile.acceleration << '\n'
              << "加速时间: " << profile.acceleration_time << '\n'
              << "匀速时间: " << profile.cruise_time << '\n'
              << "减速时间: " << profile.deceleration_time << '\n'
              << "总时间: " << profile.duration << '\n';
    return profile;
}

// 打印各段中点、分段边界以及运动前后的采样状态。
void printSamples(const TrapezoidalProfile &profile)
{
    const double ta = profile.acceleration_time;
    const double t2 = ta + profile.cruise_time;
    std::vector<double> times{
        -1.0,
        0.0,
        ta / 2.0,
        ta,
        ta + profile.cruise_time / 2.0,
        t2,
        t2 + profile.deceleration_time / 2.0,
        profile.duration,
        profile.duration + 1.0};
    // 三角形和静止轨迹存在重合的边界，只打印一次。
    std::sort(times.begin(), times.end());
    times.erase(std::unique(times.begin(), times.end()), times.end());

    std::cout << "\n采样结果（t / position / velocity / acceleration）\n"
              << std::setw(12) << "t"
              << std::setw(14) << "position"
              << std::setw(14) << "velocity"
              << std::setw(16) << "acceleration" << '\n';
    for (double t : times)
    {
        const auto point = sampleTrapezoidal(profile, t);
        std::cout << std::setw(12) << t
                  << std::setw(14) << point.position
                  << std::setw(14) << point.velocity
                  << std::setw(16) << point.acceleration << '\n';
    }

    std::cout << "------------------\n";
    const double epsilon = 1e-6;
    auto point = sampleTrapezoidal(profile, 2.0 - epsilon);
    std::cout << std::setw(12) << 2.0 - epsilon
              << std::setw(14) << point.position
              << std::setw(14) << point.velocity
              << std::setw(16) << point.acceleration << '\n';

    point = sampleTrapezoidal(profile, 2.0);
    std::cout << std::setw(12) << 2.0 - epsilon
              << std::setw(14) << point.position
              << std::setw(14) << point.velocity
              << std::setw(16) << point.acceleration << '\n';
    
    point = sampleTrapezoidal(profile, 2.0 + epsilon);
    std::cout << std::setw(12) << 2.0 - epsilon
              << std::setw(14) << point.position
              << std::setw(14) << point.velocity
              << std::setw(16) << point.acceleration << '\n';
}

void require(bool condition, const char *message)
{
    if (!condition)
        throw std::runtime_error(message);
}

bool near(double a, double b)
{
    return std::abs(a - b) <= 1e-9 * std::max({1.0, std::abs(a), std::abs(b)});
}

void testCyclePlan(const char *label, const TrapezoidalProfile &original,
                   double cycle_time, std::uint64_t expected_cycles,
                   double expected_duration)
{
    const auto plan = alignToControlCycle(original, cycle_time);
    require(plan.total_cycles == expected_cycles, "Unexpected total_cycles");
    require(near(plan.profile.duration, expected_duration), "Unexpected aligned duration");
    require(plan.profile.duration >= original.duration, "Alignment shortened the motion");
    require(near(plan.profile.p0, original.p0) && near(plan.profile.p1, original.p1),
            "Alignment changed endpoints");

    const double scale = original.duration > 0.0
        ? plan.profile.duration / original.duration : 1.0;
    require(near(plan.profile.acceleration_time, original.acceleration_time * scale) &&
            near(plan.profile.cruise_time, original.cruise_time * scale) &&
            near(plan.profile.deceleration_time, original.deceleration_time * scale),
            "Phase time scaling mismatch");
    require(near(plan.profile.peak_velocity, original.peak_velocity / scale) &&
            near(plan.profile.acceleration, original.acceleration / (scale * scale)),
            "Velocity or acceleration scaling mismatch");

    std::cout << "\n=== " << label << " ===\n"
              << "原时长 / 周期 / 周期数 / 对齐时长: "
              << original.duration << " / " << cycle_time << " / "
              << plan.total_cycles << " / " << plan.profile.duration << '\n'
              << std::setw(8) << "cycle" << std::setw(12) << "t"
              << std::setw(14) << "position" << std::setw(14) << "velocity"
              << std::setw(16) << "acceleration" << '\n';

    // 包含 k=0 和 k=N，因此 N 个运动间隔有 N+1 个采样点。
    for (std::uint64_t k = 0; k <= plan.total_cycles; ++k)
    {
        const auto point = sampleAtCycle(plan, k);
        const auto reference = sampleTrapezoidal(original, k * cycle_time / scale);
        require(near(point.position, reference.position) &&
                near(point.velocity, reference.velocity / scale) &&
                near(point.acceleration, reference.acceleration / (scale * scale)),
                "Cycle sample does not match time-scaled trajectory");
        std::cout << std::setw(8) << k << std::setw(12) << k * cycle_time
                  << std::setw(14) << point.position << std::setw(14) << point.velocity
                  << std::setw(16) << point.acceleration << '\n';
    }

    const auto start = sampleAtCycle(plan, 0);
    require(near(start.position, original.p0) && near(start.velocity, 0.0),
            "Incorrect start state");
    // 验证终点及超出范围时的钳制，包括最大的 uint64_t。
    for (auto k : {plan.total_cycles, plan.total_cycles + 1,
                   std::numeric_limits<std::uint64_t>::max()})
    {
        const auto end = sampleAtCycle(plan, k);
        require(near(end.position, original.p1) && end.velocity == 0.0 &&
                end.acceleration == 0.0, "Incorrect final or clamped state");
    }
    std::cout << "[PASS] 周期数、时间缩放、逐周期采样和末尾钳制\n";
}

void testInvalidCycles(const TrapezoidalProfile &profile)
{
    std::cout << "\n=== 非法周期 ===\n";
    for (double cycle : {0.0, -0.1,
                         std::numeric_limits<double>::quiet_NaN(),
                         std::numeric_limits<double>::infinity(),
                         -std::numeric_limits<double>::infinity()})
    {
        bool rejected = false;
        try
        {
            (void)alignToControlCycle(profile, cycle);
        }
        catch (const std::invalid_argument &e)
        {
            rejected = true;
            std::cout << "[PASS] cycle_time=" << cycle << ": " << e.what() << '\n';
        }
        require(rejected, "Invalid cycle_time was accepted");
    }
}
} // namespace

// int main()
// {
//     std::cout << std::fixed << std::setprecision(3);
// 
//     // 临界距离为 2^2 / 1 = 4；位移长度 10，生成梯形规划。
//     // printSamples(printPlan("正向梯形运动", 0.0, 10.0, 2.0, 1.0));
//     // // 三角形
//     // printSamples(printPlan("三角形运动", 0.0, 1.0, 2.0, 1.0));
// 
//     // // 同样的距离和限制，反向运动只有方向改变，各段时间相同。
//     // printSamples(printPlan("反向运动", 10.0, 0.0, 2.0, 1.0));
// 
//     // // 起终点重合，所有时间和运动幅值均为零。
//     // printSamples(printPlan("静止", 5.0, 5.0, 2.0, 1.0));
// 
//     // 测试固定周期对齐：同一条 7 秒轨迹分别使用整除和非整除周期。
//     try
//     {
//         const auto moving = planTrapezoidal(0.0, 10.0, 2.0, 1.0);
//         testCyclePlan("整除周期", moving, 0.5, 14, 7.0);
//         testCyclePlan("非整除周期", moving, 0.3, 24, 7.2);
//         const auto stationary = planTrapezoidal(5.0, 5.0, 2.0, 1.0);
//         testCyclePlan("静止轨迹", stationary, 0.1, 0, 0.0);
//         testInvalidCycles(moving);
//         // 静止轨迹也必须拒绝非法周期，不能绕过输入校验。
//         testInvalidCycles(stationary);
//         std::cout << "\n四类周期测试全部通过。\n";
//     }
//     catch (const std::exception &e)
//     {
//         std::cerr << "[FAIL] " << e.what() << '\n';
//         return 1;
//     }
//     
// 
//     return 0;
// }
// 

int main()
{
    std::cout << std::fixed << std::setprecision(6);
    try
    {
        const auto j1 = planTrapezoidal(0.0, 10.0, 2.0, 1.0);
        const auto j2 = planTrapezoidal(0.0, 2.0, 1.0, 1.0);
        const auto j3 = planTrapezoidal(5.0, 5.0, 1.0, 1.0);
        const auto j4 = planTrapezoidal(3.0, -1.0, 2.0, 1.0);
        const auto synchronized = synchronizeAxes({j1, j2});
        require(synchronized.size() == 2, "Wrong axis count");
        const auto &s1 = synchronized[0];
        const auto &s2 = synchronized[1];
        std::cout << "\n=== 两根运动轴 ===\n"
                  << "项目\tJ1\tJ2\n"
                  << "原时间(s)\t" << j1.duration << '\t' << j2.duration << '\n'
                  << "同步后时间(s)\t" << s1.duration << '\t' << s2.duration << '\n'
                  << "缩放倍数\t" << s1.duration / j1.duration << '\t' << s2.duration / j2.duration << '\n'
                  << "同步后峰值速度\t" << s1.peak_velocity << '\t' << s2.peak_velocity << '\n'
                  << "同步后加速度\t" << s1.acceleration << '\t' << s2.acceleration << '\n';
        require(near(j1.duration, 7) && near(j2.duration, 3), "Unexpected original durations");
        require(near(s1.duration, 7) && near(s2.duration, 7), "Wrong synchronized duration");
        require(near(s1.peak_velocity, 2) && near(s1.acceleration, 1) &&
                near(s2.peak_velocity, 3.0 / 7.0) && near(s2.acceleration, 9.0 / 49.0),
                "Incorrect scaled velocity or acceleration");

        const auto printPoint = [](const char *label, const TrapezoidalProfile &profile,
                                   double t, double expected_position, bool check_rest) {
            const auto point = sampleTrapezoidal(profile, t);
            std::cout << label << " t=" << t << " 实际(p,v,a)=("
                      << point.position << ", " << point.velocity << ", " << point.acceleration << ")";
            if (check_rest)
            {
                std::cout << " 预期=(" << expected_position << ", 0, 0)";
                require(near(point.position, expected_position) && near(point.velocity, 0) &&
                        near(point.acceleration, 0), "Incorrect resting sample");
            }
            std::cout << '\n';
        };
        printPoint("J1 终点", s1, 7.0, 10.0, true);
        printPoint("J2 终点", s2, 7.0, 2.0, true);

        std::cout << "\n=== 加入第三根静止轴 ===\n";
        const auto three = synchronizeAxes({j1, j2, j3});
        require(three.size() == 3 && three[2].duration == 0, "Stationary axis changed");
        for (double t : {-1.0, 0.0, 1.5, 3.5, 7.0, 9.0})
            printPoint("J3", three[2], t, 5.0, true);

        std::cout << "\n=== 加入第四根反向轴 ===\n";
        const auto four = synchronizeAxes({j1, j2, j3, j4});
        require(four.size() == 4 && near(four[3].duration, 7) && four[3].direction == -1,
                "Incorrect reverse synchronization");
        require(near(four[0].duration, 7) && near(four[1].duration, 7) && four[2].duration == 0,
                "Four-axis durations mismatch");
        std::cout << "J4 原时间=" << j4.duration << " 同步后时间=" << four[3].duration << '\n';
        printPoint("J4 起点", four[3], -1.0, 3.0, true);
        printPoint("J4 中点", four[3], 3.5, 0, false);
        const auto mid = sampleTrapezoidal(four[3], 3.5);
        require(near(mid.position, 1.0) && mid.velocity < 0, "Reverse midpoint mismatch");
        printPoint("J4 终点", four[3], 7.0, -1.0, true);
        for (std::size_t i = 0; i < four.size(); ++i)
            printPoint("四轴终点", four[i], 7.0, four[i].p1, true);

        std::cout << "\n=== 异常测试 ===\n";
        const auto expectInvalid = [](const char *label, auto action) {
            bool rejected = false;
            try { action(); }
            catch (const std::invalid_argument &e) {
                rejected = true;
                std::cout << "[PASS] " << label << ": " << e.what() << '\n';
            }
            require(rejected, "Expected invalid_argument was not thrown");
        };
        auto invalid = j2;
        invalid.duration = -1.0;
        expectInvalid("负 duration", [&] { (void)synchronizeAxes({j1, invalid}); });
        invalid.duration = std::numeric_limits<double>::quiet_NaN();
        expectInvalid("NaN duration", [&] { (void)synchronizeAxes({j1, invalid}); });
        expectInvalid("目标时间短于原时间", [&] { (void)scaleToDuration(j1, 6.0, 0.001); });

        // 有限的 scale=2，但阶段时间 1e308*2 溢出，验证结果检查。
        auto extreme = j1;
        extreme.acceleration_time = 1e308;
        bool overflow_rejected = false;
        try { (void)scaleToDuration(extreme, 14.0, 0.001); }
        catch (const std::overflow_error &e) {
            overflow_rejected = true;
            std::cout << "[PASS] 缩放结果无穷大: " << e.what() << '\n';
        }
        require(overflow_rejected, "Expected overflow_error was not thrown");
        const auto cycle_aligned = scaleToDuration(j2, 7.0, 0.3);
        require(near(cycle_aligned.duration, 7.2), "Cycle ceiling failed");
        require(near(scaleToDuration(j1, 7.0, 0.5).duration, 7.0), "Exact cycle failed");
        require(near(scaleToDuration(j1, 7.0, 0.3).duration, 7.2), "Equal duration must still align");
        const auto cycle_axes = synchronizeAxes({j1, j2, j3}, 0.3);
        require(near(cycle_axes[0].duration, 7.2) && near(cycle_axes[1].duration, 7.2) &&
                cycle_axes[2].duration == 0.0, "Synchronized cycle alignment failed");
        (void)sampleTrapezoidal(cycle_axes[1], 3.6);
        expectInvalid("非法缩放周期", [&] { (void)scaleToDuration(j1, 7.0, 0.0); });
        std::cout << "[PASS] 周期向上对齐：7s / 0.3s -> 7.2s\n";
        require(synchronizeAxes({}).empty(), "Empty synchronization failed");
        std::cout << "\n全部多轴同步测试通过。\n";
    }
    catch (const std::exception &e)
    {
        std::cerr << "[FAIL] " << e.what() << '\n';
        return 1;
    }
    return 0;
}
