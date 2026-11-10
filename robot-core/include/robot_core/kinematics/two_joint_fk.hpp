#pragma once

#include <Eigen/Geometry>

namespace robot_core::kinematics {

/**
 * @brief 计算空间二旋转关节机械臂的正运动学。
 *
 * 机械臂结构：
 * - J1 绕基坐标系 Y 轴旋转；
 * - L1 沿 J1 旋转后的局部 X 轴延伸；
 * - J2 绕自身局部 Y 轴旋转；
 * - L2 沿 J2 旋转后的局部 X 轴延伸；
 * - TCP 位于第二根连杆末端，无额外工具偏移。
 *
 * 正运动学：
 * T_BT = Rz(q1) * Dx(L1) * Ry(q2) * Dx(L2)
 *
 * @param q1 第一关节角度，单位 rad。
 * @param q2 第二关节角度，单位 rad。
 * @param L1 第一连杆长度，单位 m。
 * @param L2 第二连杆长度，单位 m。
 *
 * @return TCP 相对于基坐标系的三维刚体变换。
 *
 * @throws std::invalid_argument
 *         输入非有限，或连杆长度不大于零。
 *
 * @note 采用右手坐标系，旋转和平移均按局部坐标系连乘。
 */
Eigen::Isometry3d forwardKinematics2R(
    double q1,
    double q2,
    double L1,
    double L2);

} // namespace robot_core::kinematics