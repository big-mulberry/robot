
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cmath>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "robot_core/kinematics/serial_fk.hpp"

namespace
{

using robot_core::kinematics::JointType;
using robot_core::kinematics::StandardDhJoint;
using robot_core::kinematics::forwardKinematicsStandardDh;
using robot_core::kinematics::geometricJacobianStandardDh;

/**
 * @brief 使用中心有限差分计算TCP位置对关节变量的数值Jacobian。
 *
 * @details
 * 正运动学定义TCP位置：
 *
 *     p_TCP = f(q)
 *
 * 其中q为N维关节位置向量，p_TCP为基坐标系B下的三维位置。
 *
 * TCP线速度满足链式法则：
 *
 *     v_TCP = dp_TCP/dt
 *           = (dp_TCP/dq) * q_dot
 *           = J_v(q) * q_dot
 *
 * 因此线速度Jacobian是：
 *
 *     J_v(q) = dp_TCP/dq
 *
 * 对于第i个关节，使用中心有限差分：
 *
 *     J_v(:, i) ≈
 *         [p_TCP(q + epsilon * e_i)
 *        - p_TCP(q - epsilon * e_i)] / (2 * epsilon)
 *
 * 其中：
 * - e_i为第i个分量等于1、其余分量为0的单位向量；
 * - epsilon为关节位置扰动量；
 * - 每次只扰动一个关节，其余关节位置保持不变；
 * - 每一列需要执行两次FK。
 *
 * 例如六轴机器人：
 *
 *     q = [q1, q2, q3, q4, q5, q6]^T
 *
 * 验证J2时：
 *
 *     q_plus  = [q1, q2+epsilon, q3, q4, q5, q6]^T
 *     q_minus = [q1, q2-epsilon, q3, q4, q5, q6]^T
 *
 * 两次FK得到p_plus和p_minus后：
 *
 *     J_v(:, 1) ≈ (p_plus - p_minus) / (2 * epsilon)
 *
 * 注意上面的列索引1是C++的0-based索引，对应物理关节J2。
 *
 * @param joints Standard DH关节参数，包含关节类型和几何参数。
 * @param q 当前关节位置；旋转关节单位rad，移动关节单位m。
 * @param tool 末端DH坐标系{n}到TCP的固定刚体变换。
 * @param epsilon 关节扰动量，必须为有限正数。
 *
 * @return Eigen::MatrixXd
 *         3×N数值线速度Jacobian，所有向量在基坐标系B下表达。
 *         旋转关节列的单位为m/rad，移动关节列为无量纲。
 *
 * @throws std::invalid_argument epsilon不是有限正数时抛出。
 * @throws std::invalid_argument FK输入非法时由FK抛出。
 *
 * @note
 * - 仅计算Jacobian上3行，不验证角速度部分。
 * - 本函数用于测试，不建议用于实时运动控制。
 * - 对每个关节执行两次FK，总计2N次FK。
 * - epsilon过大产生截断误差，过小会放大浮点舍入误差。
 * - 本测试使用SI单位；混合关节类型时可分别选择扰动量。
 */
Eigen::MatrixXd numericalLinearJacobianCentralDifference(
    const std::vector<StandardDhJoint>& joints,
    const Eigen::VectorXd& q,
    const Eigen::Isometry3d& tool,
    double epsilon)
{
    if (!std::isfinite(epsilon) || epsilon <= 0.0)
    {
        throw std::invalid_argument(
            "epsilon must be finite and positive.");
    }

    const Eigen::Index joint_count = q.size();

    Eigen::MatrixXd numerical =
        Eigen::MatrixXd::Zero(3, joint_count);

    for (Eigen::Index i = 0; i < joint_count; ++i)
    {
        // 复制当前关节位置，不修改原始q。
        Eigen::VectorXd q_plus = q;
        Eigen::VectorXd q_minus = q;

        // 只扰动当前关节。
        q_plus[i] += epsilon;
        q_minus[i] -= epsilon;

        // 两次独立FK，获得TCP在基坐标系下的位置。
        const Eigen::Vector3d p_plus =
            forwardKinematicsStandardDh(
                joints, q_plus, tool).translation();

        const Eigen::Vector3d p_minus =
            forwardKinematicsStandardDh(
                joints, q_minus, tool).translation();

        // 数值估计TCP位置对第i个关节变量的偏导数。
        numerical.col(i) =
            (p_plus - p_minus) / (2.0 * epsilon);
    }

    return numerical;
}

/**
 * @brief 检查两个矩阵的最大元素绝对误差是否在容差范围内。
 *
 * @param actual 实际计算结果。
 * @param expected 期望结果。
 * @param tolerance 最大允许元素绝对误差。
 * @param test_name 测试名称，用于输出诊断信息。
 *
 * @throws std::runtime_error 矩阵尺寸不一致、结果非有限或误差超限。
 */
void assertMatrixNear(
    const Eigen::MatrixXd& actual,
    const Eigen::MatrixXd& expected,
    double tolerance,
    const std::string& test_name)
{
    if (actual.rows() != expected.rows() ||
        actual.cols() != expected.cols())
    {
        throw std::runtime_error(
            test_name + ": matrix dimensions mismatch.");
    }

    if (!actual.allFinite() || !expected.allFinite())
    {
        throw std::runtime_error(
            test_name + ": non-finite matrix value.");
    }

    const double max_error =
        (actual - expected).cwiseAbs().maxCoeff();

    std::cout << test_name
              << " max error = " << max_error << '\n';

    if (max_error > tolerance)
    {
        std::cerr << "Actual:\n" << actual << '\n';
        std::cerr << "Expected:\n" << expected << '\n';

        throw std::runtime_error(
            test_name + ": tolerance exceeded.");
    }

    std::cout << "[PASS] " << test_name << '\n';
}

/**
 * @brief 验证单轴2m旋转臂在零位的解析Jacobian。
 *
 * @details
 * 旋转轴为基坐标系+Z，TCP位于(2,0,0)m。
 *
 *     z = (0,0,1)
 *     p_TCP = (2,0,0)
 *
 * 解析Jacobian：
 *
 *     J = [z × p_TCP]
 *         [    z    ]
 *
 *       = [0, 2, 0, 0, 0, 1]^T
 */
void testSingleRevoluteJoint()
{
    const std::vector<StandardDhJoint> joints = {
        {JointType::Revolute, 2.0, 0.0, 0.0, 0.0}
    };

    const Eigen::VectorXd q =
        Eigen::VectorXd::Zero(1);

    const Eigen::Isometry3d tool =
        Eigen::Isometry3d::Identity();

    const Eigen::MatrixXd actual =
        geometricJacobianStandardDh(joints, q, tool);

    Eigen::Matrix<double, 6, 1> expected;
    expected << 0.0, 2.0, 0.0, 0.0, 0.0, 1.0;

    assertMatrixNear(
        actual, expected, 1e-12,
        "Single revolute joint");
}

/**
 * @brief 验证沿基坐标系+Z移动的单轴移动关节Jacobian。
 *
 * @details
 * 移动关节只产生线速度，不产生角速度。
 *
 *     J = [z]
 *         [0]
 *
 *       = [0, 0, 1, 0, 0, 0]^T
 */
void testSinglePrismaticJoint()
{
    const std::vector<StandardDhJoint> joints = {
        {JointType::Prismatic, 0.0, 0.0, 0.0, 0.0}
    };

    Eigen::VectorXd q(1);
    q << 0.3;

    const Eigen::Isometry3d tool =
        Eigen::Isometry3d::Identity();

    const Eigen::MatrixXd actual =
        geometricJacobianStandardDh(joints, q, tool);

    Eigen::Matrix<double, 6, 1> expected;
    expected << 0.0, 0.0, 1.0, 0.0, 0.0, 0.0;

    assertMatrixNear(
        actual, expected, 1e-12,
        "Single prismatic joint");
}

/**
 * @brief 用中心有限差分验证UR5e六轴线速度Jacobian。
 *
 * @details
 * 使用UR5e名义Standard DH参数。
 *
 * 选择非零关节位置，避免只在零位测试导致错误被掩盖。
 *
 * 额外设置非零TCP偏置，验证解析Jacobian是否正确
 * 考虑末端工具长度。
 *
 * 分别计算：
 *
 *     J_analytic = geometricJacobianStandardDh(...)
 *
 *     J_numeric = numericalLinearJacobianCentralDifference(...)
 *
 * 仅比较：
 *
 *     J_analytic.topRows(3) ≈ J_numeric
 *
 * 不比较角速度Jacobian，因为本测试只对TCP位置求差分。
 */
void testUr5eCentralDifference()
{
    const double pi = std::acos(-1.0);

    const std::vector<StandardDhJoint> joints = {
        {JointType::Revolute,  0.0,     pi / 2.0,  0.1625, 0.0},
        {JointType::Revolute, -0.425,   0.0,       0.0,    0.0},
        {JointType::Revolute, -0.3922,  0.0,       0.0,    0.0},
        {JointType::Revolute,  0.0,     pi / 2.0,  0.1333, 0.0},
        {JointType::Revolute,  0.0,    -pi / 2.0,  0.0997, 0.0},
        {JointType::Revolute,  0.0,     0.0,       0.0996, 0.0}
    };

    Eigen::VectorXd q(6);
    q << 0.3, -0.7, 0.5, -0.4, 0.8, 0.2;

    Eigen::Isometry3d tool =
        Eigen::Isometry3d::Identity();

    // 在末端DH坐标系{6}下，TCP沿局部Z偏移0.12m。
    tool.translate(Eigen::Vector3d(0.0, 0.0, 0.12));

    const Eigen::MatrixXd analytic =
        geometricJacobianStandardDh(joints, q, tool);

    const double epsilon = 1e-7;

    const Eigen::MatrixXd numerical =
        numericalLinearJacobianCentralDifference(
            joints, q, tool, epsilon);

    std::cout << "Analytical linear Jacobian:\n"
              << analytic.topRows(3) << "\n\n";

    std::cout << "Numerical linear Jacobian:\n"
              << numerical << "\n\n";

    assertMatrixNear(
        analytic.topRows(3),
        numerical,
        1e-6,
        "UR5e central finite difference");
}

} // namespace

int main()
{
    try
    {
        testSingleRevoluteJoint();
        testSinglePrismaticJoint();
        testUr5eCentralDifference();

        std::cout << "\nAll Jacobian tests passed.\n";
        return 0;
    }
    catch (const std::exception& e)
    {
        std::cerr << "[FAIL] " << e.what() << '\n';
        return 1;
    }
}
