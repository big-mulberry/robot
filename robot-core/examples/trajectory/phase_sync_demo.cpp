#include "robot_core/trajectory/phase_synchronized_trajectory.hpp"

#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>

using namespace robot_core::trajectory;

namespace
{
void require(bool condition, const char *message)
{
    if (!condition)
    {
        throw std::runtime_error(message);
    }
}

bool near(double actual, double expected)
{
    return std::abs(actual - expected) <= 1e-9;
}

void checkPoint(const char *label, const TrajectoryPoint &point,
                double position, double velocity, double acceleration)
{
    std::cout << label << " 实际(p,v,a)=(" << point.position << ", "
              << point.velocity << ", " << point.acceleration << ") 预期=("
              << position << ", " << velocity << ", " << acceleration << ")\n";
    require(near(point.position, position) && near(point.velocity, velocity) &&
            near(point.acceleration, acceleration), "Sample state mismatch");
}

template <typename Action>
void expectInvalid(const char *label, Action action)
{
    bool rejected = false;
    try
    {
        action();
    }
    catch (const std::invalid_argument &e)
    {
        rejected = true;
        std::cout << "[PASS] " << label << ": " << e.what() << '\n';
    }
    require(rejected, "Expected invalid_argument was not thrown");
}
} // namespace

// 以下为旧测试，注释保留。
// int main()
// {
//     std::cout << std::fixed << std::setprecision(6);
//     try
//     {
//         std::cout << "\n=== 1. 正常双轴 ===\n";
//         const std::vector<AxisMotion> axes{{0.0, 10.0, 2.0, 1.0},
//                                            {0.0, 2.0, 1.0, 1.0}};
//         const auto plan = planPhaseSynchronized(axes);
//         std::cout << "总时长：实际=" << plan.phase_profile.duration << "s，预期=7s\n";
//         require(near(plan.phase_profile.duration, 7.0), "Expected duration 7s");
//         std::cout << "[PASS] 正常双轴\n";
// 
//         std::cout << "\n=== 2. 中间采样 t=3.5s ===\n";
//         const auto middle = samplePhaseSynchronized(plan, 3.5);
//         require(middle.size() == 2, "Expected two axis states");
//         checkPoint("J1", middle[0], 5.0, 2.0, 0.0);
//         checkPoint("J2", middle[1], 1.0, 0.4, 0.0);
//         std::cout << "[PASS] 中间采样\n";
// 
//         std::cout << "\n=== 3. 反向运动 J1:10->0 ===\n";
//         const auto reverse = planPhaseSynchronized({{10.0, 0.0, 2.0, 1.0}});
//         const auto reverse_middle = samplePhaseSynchronized(reverse, 3.5);
//         checkPoint("反向 J1", reverse_middle[0], 5.0, -2.0, 0.0);
//         require(reverse_middle[0].velocity < 0.0, "Reverse velocity must be negative");
//         std::cout << "[PASS] 反向运动\n";
// 
//         std::cout << "\n=== 4. 混合静止轴 J3:5->5 ===\n";
//         auto mixed_axes = axes;
//         mixed_axes.push_back({5.0, 5.0, 1.0, 1.0});
//         const auto mixed = planPhaseSynchronized(mixed_axes);
//         require(near(mixed.phase_profile.duration, 7.0), "Stationary axis changed duration");
//         for (double t : {-1.0, 0.0, 1.0, 3.5, 6.0, 7.0, 8.0})
//         {
//             std::cout << "t=" << t << " ";
//             checkPoint("J3", samplePhaseSynchronized(mixed, t)[2], 5.0, 0.0, 0.0);
//         }
//         std::cout << "[PASS] 静止轴保持目标位置\n";
// 
//         std::cout << "\n=== 5. 全部静止 ===\n";
//         const auto stationary = planPhaseSynchronized({{5.0, 5.0, 1.0, 1.0},
//                                                         {-2.0, -2.0, 2.0, 1.0}});
//         std::cout << "duration=" << stationary.phase_profile.duration << "，预期=0\n";
//         require(stationary.phase_profile.duration == 0.0, "Stationary duration must be zero");
//         for (double t : {-1.0, 0.0, 3.5})
//         {
//             const auto states = samplePhaseSynchronized(stationary, t);
//             std::cout << "t=" << t << '\n';
//             checkPoint("静止 J1", states[0], 5.0, 0.0, 0.0);
//             checkPoint("静止 J2", states[1], -2.0, 0.0, 0.0);
//         }
//         std::cout << "[PASS] 全部静止\n";
// 
//         std::cout << "\n=== 6. 非法速度 vmax=0 ===\n";
//         expectInvalid("零速度上限", [] {
//             (void)planPhaseSynchronized({{0.0, 10.0, 0.0, 1.0}});
//         });
// 
//         std::cout << "\n=== 7. 非法时间 time=NaN ===\n";
//         const double nan = std::numeric_limits<double>::quiet_NaN();
//         expectInvalid("运动轨迹 NaN 时间", [&] { (void)samplePhaseSynchronized(plan, nan); });
//         expectInvalid("静止轨迹 NaN 时间", [&] { (void)samplePhaseSynchronized(stationary, nan); });
// 
//         std::cout << "\n=== 8. 终点采样：精确相等检查 ===\n";
//         // 同时覆盖正向、反向及静止关节，检查终点时刻和超过终点的时刻。
//         auto end_axes = mixed_axes;
//         end_axes.push_back({3.0, -1.0, 2.0, 1.0});
//         const auto end_plan = planPhaseSynchronized(end_axes);
//         for (double t : {end_plan.phase_profile.duration, end_plan.phase_profile.duration + 1.0})
//         {
//             const auto states = samplePhaseSynchronized(end_plan, t);
//             require(states.size() == end_axes.size(), "Endpoint axis count mismatch");
//             for (std::size_t i = 0; i < states.size(); ++i)
//             {
//                 std::cout << "t=" << t << " J" << i + 1 << " 实际终点="
//                           << states[i].position << " 预期=" << end_axes[i].p1 << '\n';
//                 require(states[i].position == end_axes[i].p1 &&
//                         states[i].velocity == 0.0 && states[i].acceleration == 0.0,
//                         "Endpoint must be exact with zero velocity and acceleration");
//             }
//         }
//         std::cout << "[PASS] 精确终点\n\n八类测试全部通过。\n";
//     }
//     catch (const std::exception &e)
//     {
//         std::cerr << "[FAIL] " << e.what() << '\n';
//         return 1;
//     }
//     return 0;
// }

int main()
{
    std::cout << std::fixed << std::setprecision(6);
    try
    {
        const std::vector<AxisMotion> axes{{0,10,2,1},{0,2,1,1}};
        const auto original = planPhaseSynchronized(axes);
        const auto exact = planCyclicPhaseSynchronized(axes, 0.5);
        const auto rounded = planCyclicPhaseSynchronized(axes, 0.3);
        require(near(original.phase_profile.duration,7), "Original duration mismatch");
        require(exact.phase_plan.total_cycles==14 && near(exact.phase_plan.profile.duration,7),
                "Exact cycle alignment failed");
        require(rounded.phase_plan.total_cycles==24 && near(rounded.phase_plan.profile.duration,7.2),
                "Non-exact cycle alignment failed");
        std::cout << "[PASS] 整除：7s / 0.5s -> 14周期、7s\n"
                  << "[PASS] 非整除：7s / 0.3s -> 24周期、7.2s\n";
        const double scale=7.2/7.0;
        const auto &aligned=rounded.phase_plan.profile;
        require(near(aligned.peak_velocity,original.phase_profile.peak_velocity/scale) &&
                near(aligned.acceleration,original.phase_profile.acceleration/(scale*scale)) &&
                near(aligned.acceleration_time,original.phase_profile.acceleration_time*scale) &&
                near(aligned.cruise_time,original.phase_profile.cruise_time*scale) &&
                near(aligned.deceleration_time,original.phase_profile.deceleration_time*scale),
                "Phase scaling mismatch");

        checkPoint("整除中点 J1",sampleCyclicPhaseSynchronized(exact,7)[0],5,2,0);
        checkPoint("整除中点 J2",sampleCyclicPhaseSynchronized(exact,7)[1],1,0.4,0);
        checkPoint("非整除中点 J1",sampleCyclicPhaseSynchronized(rounded,12)[0],5,2/scale,0);
        checkPoint("非整除中点 J2",sampleCyclicPhaseSynchronized(rounded,12)[1],1,0.4/scale,0);

        const std::vector<AxisMotion> mixed{{0,10,2,1},{0,2,1,1},{10,0,2,1},{5,5,1,1}};
        const auto plan=planCyclicPhaseSynchronized(mixed,0.3);
        std::cout << "\n周期采样：k / t / J1位置 / J2位置 / 反向轴位置 / 静止轴位置\n";
        for(std::uint64_t k=0;k<=plan.phase_plan.total_cycles;++k)
        {
            const auto points=sampleCyclicPhaseSynchronized(plan,k);
            require(points.size()==mixed.size(),"Axis count mismatch");
            // 使用原始连续轨迹与时间缩放关系作为参考。
            const auto reference=samplePhaseSynchronized(
                planPhaseSynchronized(mixed), k == plan.phase_plan.total_cycles ? original.phase_profile.duration : k*0.3/scale);
            std::cout<<k<<" / "<<k*0.3;
            for(std::size_t j=0;j<points.size();++j)
            {
                const auto &point=points[j];
                require(near(point.position,reference[j].position) &&
                        near(point.velocity,reference[j].velocity/scale) &&
                        near(point.acceleration,reference[j].acceleration/(scale*scale)),
                        "Cyclic sample differs from scaled reference");
                require(std::abs(point.velocity)<=mixed[j].max_velocity+1e-9 &&
                        std::abs(point.acceleration)<=mixed[j].max_acceleration+1e-9,
                        "Axis constraint exceeded");
                std::cout<<" / "<<point.position;
            }
            require(points[3].position==5 && points[3].velocity==0 && points[3].acceleration==0,
                    "Stationary axis moved");
            if(k>0 && k<plan.phase_plan.total_cycles)
                require(points[2].velocity<0,"Reverse velocity must be negative");
            require(near(points[0].position/10,points[1].position/2) &&
                    near(points[0].position/10,(10-points[2].position)/10),
                    "Axes are not phase synchronized");
            std::cout<<'\n';
        }
        std::cout<<"[PASS] 逐周期映射、反向、混合静止轴及运动约束\n";
        const auto start=sampleCyclicPhaseSynchronized(plan,0);
        for(std::size_t j=0;j<mixed.size();++j)
            require(start[j].position==mixed[j].p0 && start[j].velocity==0,"Start mismatch");
        for(auto k:{plan.phase_plan.total_cycles,plan.phase_plan.total_cycles+1,
                    std::numeric_limits<std::uint64_t>::max()})
        {
            const auto end=sampleCyclicPhaseSynchronized(plan,k);
            for(std::size_t j=0;j<mixed.size();++j)
                require(end[j].position==mixed[j].p1 && end[j].velocity==0 && end[j].acceleration==0,
                        "Exact endpoint or cycle clamping failed");
        }
        std::cout<<"[PASS] 起点、精确终点及超出末尾周期钳制\n";

        const auto stationary=planCyclicPhaseSynchronized({{5,5,1,1},{-2,-2,1,1}},0.1);
        require(stationary.phase_plan.total_cycles==0 && stationary.phase_plan.profile.duration==0,
                "All-stationary plan must have zero duration and cycles");
        for(std::uint64_t k:{std::uint64_t{0},std::uint64_t{10},std::numeric_limits<std::uint64_t>::max()})
        {
            const auto points=sampleCyclicPhaseSynchronized(stationary,k);
            checkPoint("全部静止 J1",points[0],5,0,0);
            checkPoint("全部静止 J2",points[1],-2,0,0);
        }
        const double nan=std::numeric_limits<double>::quiet_NaN();
        const double inf=std::numeric_limits<double>::infinity();
        for(double cycle:{0.0,-0.1,nan,inf})
        {
            expectInvalid("非法周期（运动）",[&]{(void)planCyclicPhaseSynchronized(axes,cycle);});
            expectInvalid("非法周期（静止）",[&]{(void)planCyclicPhaseSynchronized({{5,5,1,1}},cycle);});
        }
        expectInvalid("空轴组",[]{(void)planCyclicPhaseSynchronized({},0.1);});
        expectInvalid("零速度上限",[]{(void)planCyclicPhaseSynchronized({{0,1,0,1}},0.1);});
        expectInvalid("负加速度上限",[]{(void)planCyclicPhaseSynchronized({{0,1,1,-1}},0.1);});
        expectInvalid("NaN 起点",[&]{(void)planCyclicPhaseSynchronized({{nan,1,1,1}},0.1);});
        bool overflow=false;
        try{(void)planCyclicPhaseSynchronized(axes,std::numeric_limits<double>::min());}
        catch(const std::out_of_range& e){overflow=true;std::cout<<"[PASS] 周期数溢出: "<<e.what()<<'\n';}
        require(overflow,"Cycle overflow was not rejected");
        std::cout<<"\n周期相位同步测试全部通过。\n";
    }
    catch(const std::exception &e)
    {
        std::cerr<<"[FAIL] "<<e.what()<<'\n';
        return 1;
    }
    return 0;
}