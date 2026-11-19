#include "robot_core/kinematics/serial_fk.hpp"

#include <cmath>
#include <stdexcept>

namespace robot_core::kinematics
{

Eigen::Isometry3d forwardKinematicsSerial(
    const std::vector<Eigen::Isometry3d> &mounts,
    const Eigen::VectorXd &q,
    const Eigen::Isometry3d &tool)
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

    const auto validateTransform = [](const Eigen::Isometry3d &transform)
    {
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

namespace
{
void validateStandardDhInputs(
    const std::vector<StandardDhJoint> &joints,
    const Eigen::VectorXd &q,
    const Eigen::Isometry3d &tool)
{
    if (joints.size() != static_cast<std::size_t>(q.size()) || !q.allFinite())
        throw std::invalid_argument("Joint count must match finite joint coordinates.");
    for (const auto &joint : joints)
    {
        if (!std::isfinite(joint.a) || !std::isfinite(joint.alpha) ||
            !std::isfinite(joint.d_offset) || !std::isfinite(joint.theta_offset) ||
            (joint.type != JointType::Revolute && joint.type != JointType::Prismatic))
            throw std::invalid_argument("DH parameters must be finite and joint type valid.");
    }
    constexpr double tolerance = 1e-9;
    if (!tool.matrix().allFinite())
        throw std::invalid_argument("Tool transform must be finite.");
    const Eigen::Matrix3d rotation = tool.linear();
    if (!(rotation.transpose() * rotation).isApprox(Eigen::Matrix3d::Identity(), tolerance) ||
        std::abs(rotation.determinant() - 1.0) > tolerance ||
        !tool.matrix().row(3).isApprox(Eigen::RowVector4d(0.0, 0.0, 0.0, 1.0), tolerance))
        throw std::invalid_argument("Tool must be a valid SE(3) transform.");

}
} // namespace

Eigen::Isometry3d forwardKinematicsStandardDh(
    const std::vector<StandardDhJoint> &joints,
    const Eigen::VectorXd &q,
    const Eigen::Isometry3d &tool)
{
    // TODO 1：验证输入
    //
    // 1. joints.size() == q.size()
    // 2. q.allFinite()
    // 3. 每行 DH 参数均有限：
    //    a、alpha、d_offset、theta_offset
    // 4. type 只能是 Revolute 或 Prismatic
    // 5. tool 必须是有效的 SE(3)
    //
    // SE(3) 检查：
    // - matrix().allFinite()
    // - R^T * R ≈ I
    // - det(R) ≈ +1
    // - 最后一行 ≈ [0, 0, 0, 1]
    //
    // 提示：可复用 validateTransform
    // 与几何Jacobian共用文件内部的输入校验。
    validateStandardDhInputs(joints, q, tool);

    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();

    for (std::size_t i = 0; i < joints.size(); ++i)
    {
        const StandardDhJoint &joint = joints[i];

        double theta = joint.theta_offset;
        double d = joint.d_offset;

        // TODO 2：根据关节类型计算实际 DH 参数
        //
        // Revolute:
        //   theta = theta_offset + q[i]
        //
        // Prismatic:
        //   d = d_offset + q[i]
        //
        // 提示：
        // switch (joint.type) { ... }
        //
        // 计算后还要检查 theta、d 是否有限，
        // 因为两个有限 double 相加仍可能溢出。
        switch (joint.type)
        {
        case JointType::Revolute:
            theta += q[i];
            break;
        case JointType::Prismatic:
            d += q[i];
            break;
        default:
            throw std::invalid_argument("Invalid joint type.");
        }

        if (!std::isfinite(theta) || !std::isfinite(d))
            throw std::invalid_argument("Effective DH parameters must be finite.");

        Eigen::Isometry3d A =
            Eigen::Isometry3d::Identity();

        // TODO 3：构造标准 DH 单节变换
        //
        // A = Rz(theta) * Dz(d) * Dx(a) * Rx(alpha)
        //
        // 使用 Eigen::Isometry3d：
        //
        // A.rotate(Eigen::AngleAxisd(...));
        // A.translate(Eigen::Vector3d(...));
        //
        // 注意：
        // - rotate() / translate() 都是右乘操作。
        // - 严格按照上述顺序调用。
        // - 旋转轴分别使用 UnitZ()、UnitX()。
        // - 不要使用 prerotate() / pretranslate()。
        A.rotate(Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ()));
        A.translate(Eigen::Vector3d(0.0, 0.0, d));
        A.translate(Eigen::Vector3d(joint.a, 0.0, 0.0));
        A.rotate(Eigen::AngleAxisd(joint.alpha, Eigen::Vector3d::UnitX()));

        // TODO 4：累积当前关节变换
        //
        // T = T * A;
        T = T * A;
    }

    // TODO 5：右乘 TCP 固定工具变换
    //
    // T = T * tool;
    T = T * tool;

    // TODO 6：检查最终变换的元素是否有限
    //
    // 若非有限，抛出 std::overflow_error。
    // 注意：正常浮点舍入不需要重新做严格正交性检查。
    if (!T.matrix().allFinite())
        throw std::overflow_error("Forward kinematics produced non-finite values.");

    return T;
}

Eigen::MatrixXd geometricJacobianStandardDh(
    const std::vector<StandardDhJoint> &joints,
    const Eigen::VectorXd &q,
    const Eigen::Isometry3d &tool)
{
    // TODO 1：参数检查
    //
    // 与forwardKinematicsStandardDh保持一致：
    // - joints.size() == q.size()
    // - q.allFinite()
    // - DH参数全部有限
    // - tool为有效SE(3)
    //
    // 不要为了本函数重新发明另一套检查规则。
    validateStandardDhInputs(joints, q, tool);

    const Eigen::Index joint_count = q.size();

    // 行数为线速度+角速度，列数为关节数
    Eigen::MatrixXd jacobian =
        Eigen::MatrixXd::Zero(6, joint_count);

    // 保存 每个关节的原点在基坐标系中的位置
    std::vector<Eigen::Vector3d> origins(
        static_cast<std::size_t>(joint_count));

    // 保存 每个关节的局部+Z轴在基坐标系中的方向
    std::vector<Eigen::Vector3d> axes(
        static_cast<std::size_t>(joint_count));

    Eigen::Isometry3d transform =
        Eigen::Isometry3d::Identity();

    for (Eigen::Index i = 0; i < joint_count; ++i)
    {
        const auto &joint = joints[static_cast<std::size_t>(i)];
        // TODO 2：在乘当前A_i之前，保存：
        //
        // p_{i-1}：当前坐标系原点在B中的位置；
        // z_{i-1}：当前坐标系局部+Z轴在B中的方向。
        //
        // 提示：
        // transform.translation()
        // transform.linear().col(2)
        origins[static_cast<std::size_t>(i)] =
            transform.translation();
        axes[static_cast<std::size_t>(i)] =
            transform.linear().col(2);
        // TODO 3：构造当前Standard DH相对变换A_i。
        //
        // Revolute:
        //   theta = theta_offset + q[i]
        //   d     = d_offset
        //
        // Prismatic:
        //   theta = theta_offset
        //   d     = d_offset + q[i]
        //
        // 复用FK-05已有的DH单行构造逻辑，
        // 不要在public接口中增加UR5e专用函数。
        double theta = joint.theta_offset;
        double d = joint.d_offset;
        switch (joint.type)
        {
        case JointType::Revolute:
            theta += q[i];
            break;
        case JointType::Prismatic:
            d += q[i];
            break;
        default:
            throw std::invalid_argument("Invalid joint type.");
        }

        if (!std::isfinite(theta) || !std::isfinite(d))
            throw std::invalid_argument("Effective DH parameters must be finite.");

        Eigen::Isometry3d relative =
            Eigen::Isometry3d::Identity();

        // TODO：完成relative
        relative.rotate(Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ()));
        relative.translate(Eigen::Vector3d(0.0, 0.0, d));
        relative.translate(Eigen::Vector3d(joint.a, 0.0, 0.0));
        relative.rotate(Eigen::AngleAxisd(joint.alpha, Eigen::Vector3d::UnitX()));
        transform = transform * relative;
    }

    // tool必须参与TCP位置计算。
    transform = transform * tool;

    if (!transform.matrix().allFinite())
        throw std::overflow_error("TCP transform is not finite.");

    const Eigen::Vector3d tcp_position =
        transform.translation();

    for (Eigen::Index i = 0; i < joint_count; ++i)
    {
        const std::size_t index =
            static_cast<std::size_t>(i);
        const auto &joint = joints[index];
        const auto &origin = origins[index];
        const auto &axis = axes[index];
        auto column = jacobian.col(i);
        // TODO 4：根据关节类型填写Jacobian第i列。
        //
        // Revolute:
        //   topRows<3>()    = z × (p_tcp - p_joint)
        //   bottomRows<3>() = z
        //
        // Prismatic:
        //   topRows<3>()    = z
        //   bottomRows<3>() = 0
        switch (joint.type)
        {
        case JointType::Revolute:
            column.topRows<3>() = axis.cross(tcp_position - origin);
            column.bottomRows<3>() = axis;
            break;
        case JointType::Prismatic:
            column.topRows<3>() = axis;
            column.bottomRows<3>().setZero();
            break;
        default:
            throw std::invalid_argument("Invalid joint type.");
        }
    }

    if (!jacobian.allFinite())
        throw std::overflow_error("Jacobian calculation produced non-finite values.");
    return jacobian;
}

} // namespace robot_core::kinematics