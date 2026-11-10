#include "robot_core/kinematics/rigid_transform.hpp"

#include <cmath>
#include <stdexcept>

namespace robot_core::kinematics
{

Eigen::Isometry3d makeTransform3D(
    const Eigen::Vector3d &translation,
    const Eigen::Vector3d &axis,
    double angle)
{
    // TODO 1: 参数有限性检查
    //
    // 需要检查：
    // 1. translation 的三个分量是否全部有限；
    // 2. axis 的三个分量是否全部有限；
    // 3. angle 是否有限。
    //
    // 提示：
    // - Eigen::Vector3d::allFinite()
    // - std::isfinite()
    //
    // 非法输入抛出 std::invalid_argument。
    if (!translation.allFinite() || !axis.allFinite() || !std::isfinite(angle))
    {
        throw std::invalid_argument("Translation, axis and angle must be finite.");
    }

    // TODO 2: 验证旋转轴
    //
    // 需要检查 axis 是否可以安全归一化。
    //
    // 提示：
    // - axis.norm()
    // - axis.squaredNorm()
    //
    // 注意：
    // - 零向量不能作为旋转轴；
    // - 极小向量可能存在数值问题；
    // - 需要考虑模长计算的有限性。
    const double axis_norm = axis.stableNorm();
    constexpr double min_axis_norm = 1e-12;
    if (!std::isfinite(axis_norm) || axis_norm <= min_axis_norm)
    {
        throw std::invalid_argument("Rotation axis norm must be finite and greater than 1e-12.");
    }

    // TODO 3: 旋转轴归一化
    //
    // 目标：
    // Eigen::Vector3d normalized_axis
    //
    // 提示：
    // - axis.normalized()
    //
    // 思考：
    // 为什么 Eigen::AngleAxisd 要求旋转轴为单位向量？
    Eigen::Vector3d normalized_axis = axis / axis_norm;

    // TODO 4: 构造三维旋转矩阵
    //
    // 目标：
    // Eigen::Matrix3d rotation
    //
    // 提示：
    // - Eigen::AngleAxisd
    // - toRotationMatrix()
    //
    // rotation 必须满足：
    // R.transpose() * R ≈ I
    // det(R) ≈ 1
    Eigen::Matrix3d rotation = Eigen::AngleAxisd(angle, normalized_axis).toRotationMatrix();

    // TODO 5: 构造 SE(3) 刚体变换
    //
    // 1. 使用 Eigen::Isometry3d::Identity() 初始化；
    // 2. 将 rotation 写入 T.linear()；
    // 3. 将 translation 写入 T.translation()。
    //
    // 注意：
    // 不要将旋转和平移的设置顺序
    // 与矩阵乘法的先后顺序混淆。
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.linear() = rotation;
    T.translation() = translation;

    // TODO 6: 返回构造完成的 T
    //
    // 完成以上 TODO 后替换下面的临时返回值。
    return T;
}

} // namespace robot_core::kinematics