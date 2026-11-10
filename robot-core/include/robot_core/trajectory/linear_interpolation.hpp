#pragma once
#include <Eigen/Core>

namespace robot_core::trajectory {


// 线性插值
Eigen::Vector3d interpolateLinear(
    const Eigen::Vector3d& p_start,
    const Eigen::Vector3d& p_goal,
    double t,
    double duration);


/**
 * @brief 关节空间直线路径采样
 *
 * 几何路径定义：
 *     q(s) = q0 + s * (q1 - q0)
 *
 * 注意：
 * 1. 本函数只计算几何路径上的关节位置，不涉及时间参数化。
 * 2. s 是无量纲路径参数，不是时间 t。
 * 3. 本函数不计算关节速度、加速度和 Jerk。
 * 4. 所有关节共用同一个路径参数 s。
 *
 * @param q0 起点关节位置向量，单位通常为 rad
 * @param q1 终点关节位置向量，单位通常为 rad
 * @param s  路径进度，要求 0 <= s <= 1
 *
 * @return Eigen::VectorXd 路径参数 s 对应的关节位置
 *
 * @throws std::invalid_argument 输入维度不合法、存在非有限值或 s 超出范围
 */
Eigen::VectorXd sampleJointLinearPath(
    const Eigen::VectorXd& q0,
    const Eigen::VectorXd& q1,
    double s);





}