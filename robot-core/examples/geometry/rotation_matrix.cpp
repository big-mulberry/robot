
#include <Eigen/Dense>
#include <cmath>
#include <iostream>

int main()
{
    using Vector3 = Eigen::Vector3d;
    using Matrix3 = Eigen::Matrix3d;

    const double pi = std::acos(-1.0);
    const double theta = pi / 2.0;

    const Vector3 p(1.0, 0.0, 0.0);

    // 1. 构造绕 Z 轴旋转 theta 的矩阵
    Matrix3 R;

    // TODO: 使用 R << ... 填充 9 个矩阵元素
    // 注意 Eigen 的 << 初始化器按行填写
    R << std::cos(theta), -std::sin(theta), 0.0,
        std::sin(theta), std::cos(theta), 0.0,
        0.0, 0.0, 1.0;

    // 2. 计算旋转后的向量
    // TODO: const Vector3 p_rotated = ...
    const Vector3 p_rotated = R * p;
    std::cout << "旋转前向量: " << p.transpose() << '\n';
    std::cout << "旋转后向量: " << p_rotated.transpose() << '\n';
    // 3. 验证旋转前后长度相等
    // TODO: 比较 p.norm() 和 p_rotated.norm()
    std::cout << "旋转前长度: " << p.norm() << '\n';
    std::cout << "旋转后长度: " << p_rotated.norm() << '\n';

    // 4. 验证正交性
    // TODO: Matrix3 check = R.transpose() * R;
    Matrix3 check = R.transpose() * R;
    // TODO: 与 Matrix3::Identity() 比较
    std::cout << "旋转矩阵的转置乘以自身:\n"
              << check << '\n';
    std::cout << "单位矩阵:\n"
              << Matrix3::Identity() << '\n';

    // 5. 验证逆旋转
    // TODO: const Vector3 p_restored = ...
    const Vector3 p_restored = R.transpose() * p_rotated;
    std::cout << "旋转后再逆旋转得到的向量: " << p_restored.transpose() << '\n';
    std::cout << "原始向量: " << p.transpose() << '\n';
    // 6. 输出矩阵、向量和验证结果
    std::cout << "旋转矩阵:\n"
              << R << '\n';

    return 0;
}
