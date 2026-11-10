#pragma once

#include <Eigen/Geometry>

namespace robot_core::kinematics {

/**
 * @brief 根据平移向量和轴角旋转构造三维刚体变换。
 *
 * 构造齐次变换矩阵：
 *
 *     T = [ R  p ]
 *         [ 0  1 ]
 *
 * 其中：
 * - R ∈ SO(3)，表示三维旋转；
 * - p ∈ R³，表示三维平移；
 * - T ∈ SE(3)，表示完整的刚体变换。
 *
 * 变换约定：
 * - 使用右手坐标系；
 * - angle 使用弧度；
 * - axis 表示旋转轴，允许输入非单位向量；
 * - translation 表示目标坐标系原点在参考坐标系中的位置；
 * - 返回的变换可用于将目标坐标系中的点转换到参考坐标系。
 *
 * @param translation 三维平移向量，单位由调用方约定，通常为 m。
 * @param axis 三维旋转轴，函数内部需要归一化。
 * @param angle 绕 axis 旋转的角度，单位 rad。
 *
 * @return Eigen::Isometry3d 构造完成的三维刚体变换。
 *
 * @throws std::invalid_argument 输入包含 NaN、Inf 或旋转轴模长无效。
 *
 * @note 即使 angle == 0，也要求 axis 是有效的非零向量。
 */
Eigen::Isometry3d makeTransform3D(
    const Eigen::Vector3d& translation,
    const Eigen::Vector3d& axis,
    double angle);

} // namespace robot_core::kinematics