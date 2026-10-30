#include "robot_core/trajectory/linear_interpolation.hpp"
#include <algorithm>   // std::clamp
#include <cmath>       // std::isfinite
#include <stdexcept>   // std::invalid_argument
namespace robot_core::trajectory
{

Eigen::Vector3d interpolateLinear(
    const Eigen::Vector3d &p_start,
    const Eigen::Vector3d &p_goal,
    double t,
    double duration)
{
    // TODO 1：检查参数有效性
    if (!std::isfinite(duration) || duration <= 0.0)
    {
        throw std::invalid_argument("Invalid duration.");
    }

    if (!std::isfinite(t))
    {
        throw std::invalid_argument("Invalid sampling time.");
    }
    // TODO 2：限制时间范围
    const double clamp_t = std::clamp(t, 0.0, duration);

    // TODO 3：计算归一化进度
    const double normalized_progress = clamp_t / duration;

    // TODO 4：计算插值位置
    return p_start + normalized_progress * (p_goal - p_start);
}

}