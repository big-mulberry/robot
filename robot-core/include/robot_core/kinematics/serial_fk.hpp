#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <vector>

namespace robot_core::kinematics {

/**
 * @brief 计算串联旋转关节机器人的正运动学。
 *
 * 运动学模型：
 * T_BT = M1 * Ry(q1) * M2 * Ry(q2) * ... * Mn * Ry(qn) * M_tool
 *
 * 其中：
 * - Mi：上一关节输出坐标系到当前关节零位安装坐标系的固定变换。
 * - Ry(qi)：当前关节绕局部 +Y 轴的旋转。
 * - M_tool：最后一个关节输出坐标系到 TCP 的固定变换。
 *
 * @param mounts 固定安装变换数组，长度为关节数量。
 * @param q 关节角度向量，单位 rad。
 * @param tool 最后一个关节输出坐标系到 TCP 的固定变换。
 *
 * @return TCP 相对于基坐标系 B 的齐次变换。
 *
 * @throws std::invalid_argument 参数数量不匹配、关节角非有限
 *         或输入固定变换不满足刚体变换要求。
 *
 * @note 所有关节均为旋转关节，旋转轴统一定义为各自局部 +Y。
 * @note 允许零关节，此时返回 tool。
 */
Eigen::Isometry3d forwardKinematicsSerial(
    const std::vector<Eigen::Isometry3d>& mounts,
    const Eigen::VectorXd& q,
    const Eigen::Isometry3d& tool);

}  // namespace robot_core::kinematics