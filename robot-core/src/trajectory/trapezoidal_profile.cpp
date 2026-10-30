#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <algorithm>
#include "robot_core/trajectory/trapezoidal_profile.hpp"
namespace robot_core::trajectory
{
TrapezoidalProfile planTrapezoidal(
    double p0,
    double p1,
    double max_velocity,
    double max_acceleration)
{
    // TODO 1：
    // 检查四个输入均为有限数。
    // max_velocity 和 max_acceleration 必须大于零。
    if (!std::isfinite(p0) || !std::isfinite(p1) ||
        !std::isfinite(max_velocity) || !std::isfinite(max_acceleration))
    {
        throw std::invalid_argument("All inputs must be finite.");
    }

    if (max_velocity <= 0.0 || max_acceleration <= 0.0)
    {
        throw std::invalid_argument(
            "max_velocity and max_acceleration must be greater than zero.");
    }

    // TODO 2：
    // 计算 displacement、distance 和 direction。
    const double displacement = p1 - p0;
    const double distance = std::abs(p1 - p0);
    const double direction = p1 > p0 ? 1.0 : -1.0;

    // TODO 3：
    // 处理零位移。
    // 本课约定返回 duration == 0 的静止规划。
    if (distance == 0.0)
    {
        TrapezoidalProfile profile{};
        profile.p0 = p0;
        profile.p1 = p1;
        profile.duration = 0.0;
        profile.direction = 0.0;
        profile.type = ProfileType::Triangular;
        return profile;
    }

    // TODO 4：
    // 计算达到最大速度并停止所需的临界距离。
    const double threshold_distance = max_velocity * max_velocity / max_acceleration;

    TrapezoidalProfile profile{};

    if (distance >= threshold_distance)
    {
        // TODO 5：梯形速度规划。
        //
        profile.peak_velocity = max_velocity;
        profile.acceleration_time = max_velocity / max_acceleration;
        profile.cruise_time = (distance - threshold_distance) / max_velocity;
        profile.deceleration_time = profile.acceleration_time;
        profile.duration = profile.acceleration_time + profile.cruise_time + profile.deceleration_time;
        profile.type = ProfileType::Trapezoidal;
    }
    else
    {
        // TODO 6：三角形速度规划。
        profile.acceleration_time = std::sqrt(distance / max_acceleration);
        profile.peak_velocity = max_acceleration * profile.acceleration_time;
        profile.cruise_time = 0;
        profile.deceleration_time = profile.acceleration_time;
        profile.duration = profile.acceleration_time + profile.cruise_time + profile.deceleration_time;
        profile.type = ProfileType::Triangular;
    }

    // TODO 7：
    // 填入 p0、p1、direction、acceleration 等公共字段。
    profile.p0 = p0;
    profile.p1 = p1;
    profile.direction = direction;
    profile.acceleration = max_acceleration;
    return profile;
}

TrajectoryPoint sampleTrapezoidal(const TrapezoidalProfile &profile, double t)
{

    // struct TrapezoidalProfile
    // {
    //     double p0; // 起点位置
    //     double p1; // 终点位置
    //     // +1.0 表示正向运动，-1.0 表示反向运动。
    //     double direction;
    //     // 以下保存正的幅值。
    //     double acceleration;
    //     double peak_velocity;
    //     double acceleration_time; // 加速时段
    //     double cruise_time;       // 匀速
    //     double deceleration_time; // 减速
    //     double duration;          // 总时段
    //     ProfileType type;
    // };

    // 判断t的位置是处于哪一段
    // 然后知道a，推出位置和速度

    // TODO 1：
    // 检查 t 是否为有限数。
    // 必要时也检查 profile 中使用到的参数。
    if (!std::isfinite(t))
    {
        throw std::invalid_argument("t must be finite.");
    }

    if (!std::isfinite(profile.p0) || !std::isfinite(profile.p1) ||
        !std::isfinite(profile.direction) || !std::isfinite(profile.acceleration) ||
        !std::isfinite(profile.peak_velocity) ||
        !std::isfinite(profile.acceleration_time) ||
        !std::isfinite(profile.cruise_time) ||
        !std::isfinite(profile.deceleration_time) ||
        !std::isfinite(profile.duration))
    {
        throw std::invalid_argument("Profile parameters must be finite.");
    }

    if (profile.acceleration < 0.0 || profile.peak_velocity < 0.0 ||
        profile.acceleration_time < 0.0 || profile.cruise_time < 0.0 ||
        profile.deceleration_time < 0.0 || profile.duration < 0.0)
    {
        throw std::invalid_argument("Profile magnitudes and times must be non-negative.");
    }

    // 用相对容差比较计算关系，避免将很小的非零轨迹误判为零。
    const auto approximatelyEqual = [](long double lhs, long double rhs) {
        return std::isfinite(lhs) && std::isfinite(rhs) &&
               std::abs(lhs - rhs) <= 1e-9L * std::max(std::abs(lhs), std::abs(rhs));
    };

    if (profile.type != ProfileType::Trapezoidal &&
        profile.type != ProfileType::Triangular)
    {
        throw std::invalid_argument("Unknown profile type.");
    }

    // 静止规划必须全部静止，不能仅靠 duration == 0 提前返回。
    if (profile.duration == 0.0)
    {
        if (profile.p0 != profile.p1 || profile.direction != 0.0 ||
            profile.acceleration != 0.0 || profile.peak_velocity != 0.0 ||
            profile.acceleration_time != 0.0 || profile.cruise_time != 0.0 ||
            profile.deceleration_time != 0.0)
        {
            throw std::invalid_argument("Stationary profile parameters are inconsistent.");
        }
    }
    else
    {
        const double expected_direction = profile.p1 > profile.p0 ? 1.0 : -1.0;
        if (profile.p0 == profile.p1 || profile.direction != expected_direction)
        {
            throw std::invalid_argument("Profile direction must match the start and end positions.");
        }
        if (profile.acceleration <= 0.0 || profile.peak_velocity <= 0.0 ||
            profile.acceleration_time <= 0.0 || profile.deceleration_time <= 0.0)
        {
            throw std::invalid_argument("Moving profiles require positive acceleration, peak velocity and ramp times.");
        }

        // 使用更宽的中间类型，减少校验计算本身溢出的风险。
        const long double ta = profile.acceleration_time;
        const long double tc = profile.cruise_time;
        const long double td = profile.deceleration_time;
        const long double a = profile.acceleration;
        const long double vp = profile.peak_velocity;
        if (!approximatelyEqual(profile.duration, ta + tc + td))
        {
            throw std::invalid_argument("Profile duration must equal the sum of phase times.");
        }
        if (!approximatelyEqual(vp, a * ta) || !approximatelyEqual(vp, a * td))
        {
            throw std::invalid_argument("Peak velocity must match acceleration times both ramp durations.");
        }
        if (profile.type == ProfileType::Triangular && profile.cruise_time != 0.0)
        {
            throw std::invalid_argument("Triangular profiles must have zero cruise time.");
        }
        const long double distance = std::abs(
            static_cast<long double>(profile.p1) - profile.p0);
        const long double integrated_distance = vp * (ta / 2.0L + tc + td / 2.0L);
        if (!approximatelyEqual(distance, integrated_distance))
        {
            throw std::invalid_argument("Profile displacement must equal the area under the velocity curve.");
        }
    }
    // TODO 2：
    // 处理静止轨迹：
    if (profile.duration == 0)
    {
        return TrajectoryPoint{profile.p1, 0.0, 0.0};
    }

    // TODO 3：
    // t < 0 时，返回运动开始前的静止状态：
    // position = p0，velocity = 0，acceleration = 0。
    if (t < 0)
    {
        return TrajectoryPoint{profile.p0, 0.0, 0.0};
    }

    // TODO 4：
    // t >= duration 时，显式返回最终状态：
    // position = p1，velocity = 0，acceleration = 0。
    //
    // 直接使用 p1，避免浮点计算使终点出现微小误差。
    if (t >= profile.duration)
    {
        return TrajectoryPoint{profile.p1, 0.0, 0.0};
    }

    const double a = profile.acceleration;
    const double vp = profile.peak_velocity;
    const double ta = profile.acceleration_time;
    const double tc = profile.cruise_time;

    const double t1 = ta; // 匀速起点
    const double t2 = ta + tc; // 减速起点

    double distance = 0.0;
    double velocity_magnitude = 0.0;
    double acceleration_along_motion = 0.0;

    if (t < t1)
    {
        // TODO 5：加速段。
        distance = 0.5 * a * t*t;
        velocity_magnitude =a*t;
        acceleration_along_motion = a;
    }
    else if (t < t2)
    {
        // TODO 6：匀速段。
        const auto tau = t - t1;
        distance = 0.5 * a * t1*t1 + vp*tau;
        velocity_magnitude = vp;
        acceleration_along_motion = 0;
    }
    else
    {
        // TODO 7：减速段。
        const auto tau = t - t2;
        distance = 0.5 * a * t1*t1 + vp*tc + vp * tau - 0.5 * a *tau*tau;
        velocity_magnitude = vp - a *tau;
        acceleration_along_motion = -a;
    }

    // TODO 8：
    // 使用 p0、direction 和上面的三个中间结果，
    // 得到有符号的位置、速度和加速度。
    const auto dir = profile.direction;
    return TrajectoryPoint{profile.p0 + distance * dir, velocity_magnitude*dir,acceleration_along_motion*dir};
}

CyclicTrapezoidalPlan alignToControlCycle(const TrapezoidalProfile &profile, double cycle_time)
{
    // TODO 1：
    // cycle_time 必须有限且大于 0。
    // profile.duration 必须有限且不小于 0。
    if (!std::isfinite(cycle_time) || cycle_time <= 0.0)
    {
        throw std::invalid_argument("cycle_time must be finite and greater than zero.");
    }
    if (!std::isfinite(profile.duration) || profile.duration < 0.0)
    {
        throw std::invalid_argument("Profile duration must be finite and non-negative.");
    }

    // TODO 2：
    // 静止轨迹不需要执行运动周期。
    // 返回 total_cycles == 0。
    if(profile.duration == 0)
    {
        return CyclicTrapezoidalPlan{profile, cycle_time, 0};
    }

    // TODO 3：
    // 计算所需周期数，必须向上取整。
    const double raw_cycles =
        profile.duration / cycle_time;

    const double rounded_cycles =
        std::ceil(raw_cycles);

    // TODO 4：
    // 转成 uint64_t 前检查是否超过其表示范围。
     if (!std::isfinite(rounded_cycles) || rounded_cycles < 0.0 || rounded_cycles >= 0x1p64)
    {
        throw std::out_of_range("rounded_cycles is outside uint64_t range.");
    }
    const std::uint64_t total_cycles = static_cast<std::uint64_t>(rounded_cycles);

    // TODO 5：
    // 计算周期对齐后的总时间和缩放系数。
    const double execution_duration = total_cycles*cycle_time;
        /* total_cycles × cycle_time */;

    const double scale = execution_duration / profile.duration;
        /* execution_duration / 原 duration */;

    // TODO 6：
    // 复制原规划，然后统一缩放三个阶段时间。
    TrapezoidalProfile aligned = profile;

    aligned.acceleration_time *= scale;
    aligned.cruise_time *= scale;
    aligned.deceleration_time *= scale;
    aligned.duration = execution_duration;

    // TODO 7：
    // 缩放峰值速度和加速度。
    aligned.peak_velocity /= scale;
    aligned.acceleration /= scale * scale;

    // TODO 8：
    // 检查缩放后的关键结果仍然有限。
    if (!std::isfinite(execution_duration) || !std::isfinite(scale) || scale <= 0.0 ||
        !std::isfinite(scale * scale) ||
        !std::isfinite(aligned.acceleration_time) ||
        !std::isfinite(aligned.cruise_time) ||
        !std::isfinite(aligned.deceleration_time) ||
        !std::isfinite(aligned.duration) ||
        !std::isfinite(aligned.peak_velocity) ||
        !std::isfinite(aligned.acceleration))
    {
        throw std::overflow_error("Control-cycle alignment produced non-finite results or an invalid scale.");
    }

    return CyclicTrapezoidalPlan{
        aligned,
        cycle_time,
        total_cycles
    };
}

TrajectoryPoint sampleAtCycle(
    const CyclicTrapezoidalPlan &plan,
    std::uint64_t cycle_index)
{
    // TODO 1：
    // 对超过末尾的周期序号进行钳制。
    const std::uint64_t clamped_cycle = std::min(cycle_index, plan.total_cycles) ;

    // TODO 2：
    // 使用整数周期序号计算时间。
    const double t =
        static_cast<double>(clamped_cycle) * plan.cycle_time;

    // TODO 3：
    // 复用已经完成的连续时间插补器。
    return  sampleTrapezoidal(plan.profile, t);
}

TrapezoidalProfile scaleToDuration(
    const TrapezoidalProfile &profile,
    double target_duration, double cycle_time)
{
    // 将profile的duration拉长到target_duration
    // TODO 1：
    // 检查 target_duration 有限且不小于 0。
    // 检查 profile.duration 有限且不小于 0。
    if (!std::isfinite(target_duration) || target_duration < 0.0 ||
        !std::isfinite(profile.duration) || profile.duration < 0.0)
    {
        throw std::invalid_argument("Target and original durations must be finite and non-negative.");
    }

    if (!std::isfinite(cycle_time) || cycle_time <= 0.0)
    {
        throw std::invalid_argument("cycle_time must be finite and positive.");
    }
    // 保留原约定：不接受比原运动更短的目标时间。
    if (target_duration < profile.duration)
    {
        throw std::invalid_argument("target_duration must not be shorter than profile.duration");
    }
    const double cycles = std::ceil(target_duration / cycle_time);
    if (!std::isfinite(cycles) || cycles >= 0x1p64)
    {
        throw std::overflow_error("Aligned cycle count is outside uint64_t range.");
    }
    const double aligned_duration = cycles * cycle_time;
    if (!std::isfinite(aligned_duration) || aligned_duration < target_duration)
    {
        throw std::overflow_error("Aligned duration overflowed or rounded below the target.");
    }
    target_duration = aligned_duration;
    // TODO 2：
    // 静止轴的 duration == 0。
    // 保持该轨迹不变，不进行除法。
    if (profile.duration == 0)
    {
        return profile;
    }

    // TODO 3：
    // 不允许 target_duration 缩短原轨迹。
    // 否则可能超过原速度和加速度限制。
    if (target_duration < profile.duration)
    {
        throw std::invalid_argument(
            "target_duration must not be shorter than profile.duration");
    }

    if (target_duration == profile.duration)
    {
        return profile;
    }
    const double scale =
        target_duration / profile.duration;
    if (!std::isfinite(scale) || scale < 1.0)
    {
        throw std::overflow_error("Invalid duration scale");
    }

    TrapezoidalProfile result = profile;

    // TODO 4：
    // 缩放三个阶段时间和总时间。
    result.acceleration_time *= scale;
    result.cruise_time *= scale;
    result.deceleration_time *= scale;
    result.duration = target_duration;

    // TODO 5：
    // 缩放峰值速度与加速度。
    result.peak_velocity /= scale;
    result.acceleration /= scale;
    result.acceleration /= scale;
    // TODO 6：
    // 验证结果有限。
    if (!std::isfinite(scale) || scale <= 0.0 ||
        !std::isfinite(result.p0) || !std::isfinite(result.p1) ||
        !std::isfinite(result.direction) ||
        !std::isfinite(result.acceleration_time) ||
        !std::isfinite(result.cruise_time) ||
        !std::isfinite(result.deceleration_time) ||
        !std::isfinite(result.duration) ||
        !std::isfinite(result.peak_velocity) ||
        !std::isfinite(result.acceleration))
    {
        throw std::overflow_error("Duration scaling produced non-finite results or an invalid scale.");
    }

    return result;
}

std::vector<TrapezoidalProfile> synchronizeAxes(
    const std::vector<TrapezoidalProfile>& profiles, double cycle_time)
{
    if (!std::isfinite(cycle_time) || cycle_time <= 0.0)
        throw std::invalid_argument("cycle_time must be finite and positive.");
    // TODO 1：
    // 空输入直接返回空 vector。
    if(profiles.empty()) return std::vector<TrapezoidalProfile>{};

    // TODO 2：
    // 遍历所有轴，检查 duration 合法，
    // 找出最大的 duration。
    double synchronized_duration = 0.0;

    for (const auto &profile : profiles)
    {
        if (!std::isfinite(profile.duration) || profile.duration < 0.0)
        {
            throw std::invalid_argument(
                "Axis duration must be finite and non-negative.");
        }
        synchronized_duration =
            std::max(synchronized_duration, profile.duration);
    }

    // TODO 3：
    // 为结果预留空间，避免反复扩容。
    std::vector<TrapezoidalProfile> synchronized;
    synchronized.reserve(profiles.size());

    // TODO 4：
    // 逐轴调用 scaleToDuration()。
    //
    // 静止轴保持静止；
    // 运动轴统一延长到 synchronized_duration。
    for (const auto& profile : profiles)
    {
        synchronized.push_back(scaleToDuration(profile, synchronized_duration, cycle_time));
    }

    return synchronized;
}

}