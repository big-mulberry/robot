
#include <Eigen/Dense>
#include <cassert>
#include <iostream>

int main()
{
    using Vector3 = Eigen::Vector3d;
    using Matrix3 = Eigen::Matrix3d;

    // 工具坐标系的三个轴，在基坐标系中的方向
    const Vector3 x_tool_in_base(0.0, 1.0, 0.0);
    const Vector3 y_tool_in_base(-1.0, 0.0, 0.0);
    const Vector3 z_tool_in_base(0.0, 0.0, 1.0);

    Matrix3 R;

    // 1：将三个方向向量填入矩阵的三列
    // 提示：R.col(0) = ...
    R.col(0) = x_tool_in_base;
    R.col(1) = y_tool_in_base;
    R.col(2) = z_tool_in_base;

    // 工具坐标系中的向量
    const Vector3 v_tool(1.0, 2.0, 0.0);

    // 2：将 v_tool 转换到基坐标系
    const Vector3 v_base = R * v_tool;

    // 3：使用 R.transpose() 反向转换
    const Vector3 v_restored = R.transpose() * v_base;

    // 4：验证转换结果
    assert(v_base.isApprox(Vector3(-2.0, 1.0, 0.0)));

    // 5：验证 R.transpose() * R = I
    assert((R.transpose() * R - Matrix3::Identity()).norm() < 1e-12);
    return 0;
}
