
#include "robot_core/kinematics/serial_fk.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cmath>
#include <iostream>
#include <numbers>
#include <stdexcept>
#include <vector>

namespace {

constexpr double kTolerance = 1e-8;

/**
 * @brief 检查两个刚体变换是否在指定容差内相等。
 *
 * @param actual 实际计算结果。
 * @param expected 手算或独立来源的预期结果。
 *
 * @throws std::runtime_error 两个矩阵不满足近似相等条件。
 */
void requireTransformNear(
    const Eigen::Isometry3d& actual,
    const Eigen::Isometry3d& expected)
{
    if (!actual.matrix().isApprox(expected.matrix(), kTolerance))
    {
        std::cerr << "Actual:\n" << actual.matrix()
                  << "\nExpected:\n" << expected.matrix() << '\n';

        throw std::runtime_error("Transform verification failed.");
    }
}

}  // namespace

int main()
{
    using robot_core::kinematics::JointType;
    using robot_core::kinematics::StandardDhJoint;
    using robot_core::kinematics::forwardKinematicsStandardDh;

    constexpr double kPi = std::numbers::pi_v<double>;

    // TODO 1：
    // 根据UR5e官方表填写6个旋转关节。
    //
    // StandardDhJoint字段顺序：
    // {type, a, alpha, d_offset, theta_offset}/*  */
    //
    // 注意：
    // 1. 官方表的顺序不是该结构体的字段顺序；
    // 2. a2、a3必须保留负号；
    // 3. 本次6个theta_offset均为0；
    // 4. 长度统一使用m，角度统一使用rad。
    const std::vector<StandardDhJoint> joints = {
        {JointType::Revolute,  0.0,     kPi / 2.0, 0.1625, 0.0},
        {JointType::Revolute, -0.4250,  0.0,       0.0,    0.0},
        {JointType::Revolute, -0.3922,  0.0,       0.0,    0.0},
        {JointType::Revolute,  0.0,     kPi / 2.0, 0.1333, 0.0},
        {JointType::Revolute,  0.0,    -kPi / 2.0, 0.0997, 0.0},
        {JointType::Revolute,  0.0,     0.0,       0.0996, 0.0}
    };

     Eigen::VectorXd q = Eigen::VectorXd::Zero(6);
    q[0] = kPi / 2.0;

    // 本次把DH坐标系6直接作为输出坐标系，不添加TCP偏置。
    const Eigen::Isometry3d tool = Eigen::Isometry3d::Identity();

    // TODO 2：
    // 调用FK-05已经实现的Standard DH正运动学接口。
    const Eigen::Isometry3d actual =
        forwardKinematicsStandardDh(joints, q, tool);

    Eigen::Isometry3d expected = Eigen::Isometry3d::Identity();

    expected.linear() <<
        0.0, 0.0,  1.0,
        1.0, 0.0,  0.0,
        0.0, 1.0,  0.0;

    expected.translation() =
        Eigen::Vector3d(0.2329, -0.8172, 0.0628);

    // TODO 3：
    // 使用requireTransformNear检查actual与expected。
    // 成功时打印：
    // [PASS] UR5e nominal zero pose
    requireTransformNear(actual, expected);
    std::cout << "T_B6 (q=0):\n" << actual.matrix() << '\n';
    std::cout << "[PASS] UR5e nominal zero pose\n";

    return 0;
}