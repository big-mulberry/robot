#include <Eigen/Dense>
#include <cassert>
#include <iostream>

int main()
{
    using Vector3 = Eigen::Vector3d;
    using Vector4 = Eigen::Vector4d;
    using Matrix3 = Eigen::Matrix3d;
    using Matrix4 = Eigen::Matrix4d;

    Matrix3 R;
    R << 0.0, -1.0, 0.0,
        1.0, 0.0, 0.0,
        0.0, 0.0, 1.0;

    const Vector3 t(1.0, 2.0, 0.0);

    Matrix4 T = Matrix4::Identity();
    T.block<3, 3>(0, 0) = R;
    T.block<3, 1>(0, 3) = t;

    // TODO 1:
    // 手动构造 T_inv，不直接调用 T.inverse()
    Matrix4 T_inv = Matrix4::Identity();
    T_inv.block<3, 3>(0, 0) = R.transpose();
    T_inv.block<3, 1>(0, 3) = -R.transpose() * t;

    // TODO 2:
    // 检查 T_inv 与 Eigen 通用求逆的结果一致
    // 注意 assert 与模板逗号的问题
    assert(T_inv.isApprox(T.inverse(), 1e-6));

    // TODO 3:
    // 检查 T * T_inv 是否等于单位矩阵
    assert((T * T_inv).isApprox(Matrix4::Identity(), 1e-6));

    // TODO 4:
    // 将基坐标系中的点 (-1, 3, 0)
    // 转换到工具坐标系
    const Vector4 p_base(-1.0, 3.0, 0.0, 1.0);

    // TODO 5:
    // 断言转换结果为 (1, 2, 0, 1)
    const Vector4 p_tool = T_inv * p_base;
    assert(p_tool.isApprox(Vector4(1.0, 2.0, 0.0, 1.0), 1e-6));

    return 0;
}