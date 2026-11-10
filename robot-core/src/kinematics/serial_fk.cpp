#include "robot_core/kinematics/serial_fk.hpp"

#include <cmath>
#include <stdexcept>

namespace robot_core::kinematics {

Eigen::Isometry3d forwardKinematicsSerial(
    const std::vector<Eigen::Isometry3d>& mounts,
    const Eigen::VectorXd& q,
    const Eigen::Isometry3d& tool)
{
    // TODO 1: 参数检查
    //
    // mounts.size() 必须等于 q.size()。
    // q 的每个元素必须为有限数。
    // mounts 和 tool 必须是有效的 SE(3) 刚体变换
    // SE(3) 刚体变换的定义：
    // 1. 4x4 齐次矩阵；
    // 2. 旋转矩阵 R = T.linear() 必须满足：
    //    - R^T * R = I
    //    - det(R) = 1
    // 3. 平移向量 t = T.translation() 可以是任意有限向量。
    // 4. 齐次矩阵的最后一行必须为 [0, 0, 0, 1]。

    // 提示：
    // q.allFinite()
    // T.matrix().allFinite()
    // T.linear().transpose() * T.linear()
    // T.linear().determinant()
    if (mounts.size() != static_cast<std::size_t>(q.size()) || !q.allFinite())
    {
        throw std::invalid_argument("Mount count must match finite joint angles.");
    }

    const auto validateTransform = [](const Eigen::Isometry3d &transform) {
        constexpr double tolerance = 1e-9;
        if (!transform.matrix().allFinite())
            throw std::invalid_argument("Fixed transform must be finite.");
        const Eigen::Matrix3d rotation = transform.linear();
        if (!(rotation.transpose() * rotation).isApprox(Eigen::Matrix3d::Identity(), tolerance) ||
            std::abs(rotation.determinant() - 1.0) > tolerance ||
            !transform.matrix().row(3).isApprox(Eigen::RowVector4d(0.0, 0.0, 0.0, 1.0), tolerance))
        {
            throw std::invalid_argument("Fixed transform must be a valid SE(3) transform.");
        }
    };
    for (const auto &mount : mounts)
    {
        validateTransform(mount);
    }
    validateTransform(tool);

    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();

    // TODO 2: 遍历所有关节
    //
    // 对第 i 个关节：
    // 1. T = T * mounts[i]
    // 2. 再右乘绕局部 +Y 的旋转 Ry(q[i])
    //
    // 提示：
    // Eigen::AngleAxisd(q[i], Eigen::Vector3d::UnitY())
    // T.rotate(...)
    for (size_t i = 0; i < mounts.size(); ++i)
    {
        T = T * mounts[i];
        T.rotate(Eigen::AngleAxisd(q[i], Eigen::Vector3d::UnitY()));
    }

    // TODO 3: 右乘工具固定变换
    //
    // T = T * tool;
    T = T * tool;
    return T;
}

}  // namespace robot_core::kinematics