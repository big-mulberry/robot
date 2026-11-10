#include "robot_core/kinematics/two_joint_fk.hpp"
#include "robot_core/kinematics/rigid_transform.hpp"

#include <cmath>
#include <stdexcept>

namespace robot_core::kinematics
{

Eigen::Isometry3d forwardKinematics2R(
    double q1,
    double q2,
    double L1,
    double L2)
{
    // TODO 1:
    // 检查所有参数是否有限。
    // L1、L2 必须严格大于 0。
    // 非法输入抛出 std::invalid_argument。
    if (!std::isfinite(q1) || !std::isfinite(q2) ||
        !std::isfinite(L1) || !std::isfinite(L2) || L1 <= 0.0 || L2 <= 0.0)
    {
        throw std::invalid_argument("Joint angles must be finite and link lengths finite and positive.");
    }

    // TODO 2:
    // 构造 J1 旋转变换 T_J1。
    //
    // 绕基坐标系 +Y 轴旋转 q1。
    // 提示：复用 makeTransform3D()。
    Eigen::Isometry3d T_J1 = makeTransform3D(Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitY(), q1);

    // TODO 3:
    // 构造第一连杆固定平移 T_L1。
    //
    // 在 J1 输出坐标系中沿 +X 平移 L1。
    // (不能一次性调用 makeTransform3D() 构造 T_J1 * T_L1，因为旋转和平移顺序不同)
    // 或者 先计算 R1 * (L1,0,0)，不能直接把 (L1,0,0) 当作合并后的平移参数。 必须先旋转再平移。
    Eigen::Isometry3d T_L1 = makeTransform3D(Eigen::Vector3d(L1, 0.0, 0.0), Eigen::Vector3d::UnitX(), 0.0);

    // TODO 4:
    // 构造 J2 旋转变换 T_J2。
    //
    // 绕 J2 的局部 +Y 轴旋转 q2。
    // TODO 5:
    // 构造第二连杆固定平移 T_L2。
    //
    // 在 J2 输出坐标系中沿 +X 平移 L2。
    Eigen::Isometry3d T2 = Eigen::Isometry3d::Identity();
    T2.rotate(Eigen::AngleAxisd(q2, Eigen::Vector3d::UnitY()));
    T2.translate(Eigen::Vector3d(L2, 0.0, 0.0));

    // TODO 6:
    // 按照正确顺序计算：
    //
    // T_BT = T_J1 * T_L1 * T_J2 * T_L2
    //
    // 注意：
    // 矩阵乘法不可交换。
    Eigen::Isometry3d T_BT = Eigen::Isometry3d::Identity();
    T_BT = T_J1 * T_L1 * T2;

    // TODO 7:
    // 返回 TCP 相对于 B 的位姿。
    return T_BT;
}

} // namespace robot_core::kinematics