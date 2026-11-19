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

//*************************************** 正运动学
/**
 * @brief 关节运动类型。
 */
enum class JointType
{
    Revolute,   ///< 旋转关节，关节变量单位 rad
    Prismatic   ///< 移动关节，关节变量单位 m
};

/**
 * @brief 标准 DH（Denavit-Hartenberg）参数。
 *
 * 单节齐次变换：
 *
 * A_i = Rz(theta_i) * Dz(d_i) * Dx(a_i) * Rx(alpha_i)
 *
 * 其中：
 * - theta_i：绕上一坐标系 Z_(i-1) 轴的旋转角。
 * - d_i：沿上一坐标系 Z_(i-1) 轴的平移距离。
 * - a_i：沿当前 X_i 轴的公法线有向距离。
 * - alpha_i：绕当前 X_i 轴的有向扭转角。
 *
 * 旋转关节：
 *   theta_i = theta_offset + q_i
 *   d_i     = d_offset
 *
 * 移动关节：
 *   theta_i = theta_offset
 *   d_i     = d_offset + q_i
 *
 * @note a、d_offset 可以为负数或零。
 * @note 本结构仅表示 Standard DH，不适用于 Modified DH。
 */
struct StandardDhJoint
{
    JointType type{JointType::Revolute};

    double a{0.0};             ///< 公法线有向距离，m
    double alpha{0.0};         ///< 连杆扭转角，rad
    double d_offset{0.0};      ///< 轴向固定偏移，m
    double theta_offset{0.0};  ///< 关节固定角度偏移，rad
};

/**
 * @brief 使用标准 DH 参数计算串联机器人正运动学。
 *
 * 对每个关节构造：
 *
 * A_i(q_i) =
 *   Rz(theta_i) * Dz(d_i) * Dx(a_i) * Rx(alpha_i)
 *
 * 最终：
 *
 * T_BT = A_1(q_1) * ... * A_n(q_n) * tool
 *
 * @param joints 标准 DH 参数表，顺序与机械运动链一致。
 * @param q 关节广义坐标，长度必须等于 joints.size()。
 *          Revolute：单位 rad。
 *          Prismatic：单位 m。
 * @param tool 最后一个 DH 坐标系到 TCP 的固定刚体变换。
 *
 * @return TCP 相对于基坐标系 B 的位姿。
 *
 * @throws std::invalid_argument 以下情况：
 *         - joints.size() 与 q.size() 不一致；
 *         - q 或任意 DH 参数包含 NaN/Inf；
 *         - JointType 不合法；
 *         - tool 不属于有效的 SE(3)。
 *         - 计算出的有效参数 theta_i、d_i 非有限。
 *
 * @throws std::overflow_error 最终变换出现非有限元素。
 *
 * @note 允许零关节，此时返回 tool。
 * @note 使用右手坐标系、列向量和左侧累积矩阵乘法。
 * @note 此接口不包含关节限位和碰撞检查。
 */
Eigen::Isometry3d forwardKinematicsStandardDh(
    const std::vector<StandardDhJoint>& joints,
    const Eigen::VectorXd& q,
    const Eigen::Isometry3d& tool);

/**
 * @brief 计算串联机器人基于Standard DH的几何Jacobian矩阵。
 *
 * @details
 * 关节速度与TCP空间速度之间满足：
 *
 *     [ v_B_TCP     ]                  [ q_dot_1 ]
 *     [             ] = J_B_TCP(q)  *  [   ...   ]
 *     [ omega_B_TCP ]                  [ q_dot_n ]
 *
 * 其中：
 * - v_B_TCP     : TCP原点在基坐标系B下的线速度，单位m/s。
 * - omega_B_TCP : TCP在基坐标系B下的角速度，单位rad/s。
 * - q_dot       : 关节速度向量，旋转关节rad/s，移动关节m/s。
 * - J_B_TCP(q)  : 6×N几何Jacobian，上3行线速度，下3行角速度。
 *
 * 对于第i个旋转关节（Revolute）：
 *
 *     J_i = [ z_(i-1) × (p_TCP - p_(i-1)) ]
 *           [           z_(i-1)          ]
 *
 * 对于第i个移动关节（Prismatic）：
 *
 *     J_i = [ z_(i-1) ]
 *           [    0    ]
 *
 * 其中：
 * - z_(i-1) : 第i个关节轴线的单位方向，在基坐标系B下表达。
 * - p_(i-1) : 第i个关节轴线上一点，在基坐标系B下表达。
 * - p_TCP   : TCP原点的位置，在基坐标系B下表达。
 *
 * Standard DH坐标系约定：
 *
 *     第i个物理关节轴线 = 坐标系{i-1}的Z轴。
 *
 * 因此：
 *
 *     z_(i-1) = R_B_(i-1) * [0, 0, 1]^T
 *     p_(i-1) = translation(T_B_(i-1))
 *
 * TCP位姿：
 *
 *     T_B_TCP = A_1(q_1) * A_2(q_2) * ... * A_n(q_n) * tool
 *
 * 最终Jacobian：
 *
 *     J_B_TCP(q) = [ J_1  J_2  ...  J_n ]
 *
 * @param joints Standard DH关节参数，包含关节类型及DH几何参数。
 * @param q 关节位置向量，旋转关节单位rad，移动关节单位m。
 * @param tool 末端DH坐标系{n}到TCP坐标系的固定刚体变换。
 *
 * @return Eigen::MatrixXd
 *         6×N几何Jacobian矩阵，所有速度均在基坐标系B下表达。
 *
 * @throws std::invalid_argument
 *         当关节数量与q维度不一致、输入存在非有限值，
 *         或tool不是有效的SE(3)变换时抛出。
 *
 * @note
 * - 所有位置和方向必须统一在基坐标系B下表达。
 * - 计算第i列时，应在右乘A_i之前获取T_B_(i-1)。
 * - tool中的平移偏置会影响线速度Jacobian。
 * - tool中的固定旋转不改变基坐标系表达的角速度Jacobian。
 * - 对于零关节模型，返回6×0矩阵。
 */
Eigen::MatrixXd geometricJacobianStandardDh(
    const std::vector<StandardDhJoint>& joints,
    const Eigen::VectorXd& q,
    const Eigen::Isometry3d& tool);

}  // namespace robot_core::kinematics