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


Eigen::VectorXd sampleJointLinearPath(
    const Eigen::VectorXd& q0,
    const Eigen::VectorXd& q1,
    double s)
{
    // TODO 1：检查关节向量维度
    // - q0 和 q1 的维度必须一致
    // - 关节数量必须大于 0
    // - 不满足条件时抛出 std::invalid_argument
    if (q0.size() == 0 || q0.size() != q1.size())
    {
        throw std::invalid_argument("Joint vectors must have equal non-zero dimensions.");
    }


    // TODO 2：检查输入数据的有效性
    // - q0 和 q1 的所有元素必须是有限值
    // - s 必须是有限值
    // - NaN 和正负无穷大都属于非法输入
    if (!q0.allFinite() || !q1.allFinite() || !std::isfinite(s))
    {
        throw std::invalid_argument("Joint positions and path parameter must be finite.");
    }


    // TODO 3：检查路径参数范围
    // - 要求 0 <= s <= 1
    // - 本次不使用 clamp，超出范围直接抛出异常
    // - s == 0 和 s == 1 都是合法边界
    if (s < 0.0 || s > 1.0)
    {
        throw std::invalid_argument("Path parameter must be within [0, 1].");
    }


    // TODO 4：计算关节空间直线路径上的采样位置
    // - 使用 q(s) = q0 + s * (q1 - q0)
    // - 返回向量维度应与输入关节向量一致
    const Eigen::VectorXd result = q0 + s * (q1 - q0);


    // TODO 5：检查计算结果
    // - 检查输出向量的所有元素是否有限
    // - 防止有限输入在中间计算中发生浮点溢出
    // - 若结果非法，抛出 std::invalid_argument
    if (!result.allFinite())
    {
        throw std::invalid_argument("Joint path calculation produced non-finite values.");
    }


    // TODO 6：返回采样结果
    return result;

}



}