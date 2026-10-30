#include <Eigen/Dense>
#include <cassert>
#include <cmath>
#include <iostream>

int main()
{
    using Vector3 = Eigen::Vector3d;
    using Vector4 = Eigen::Vector4d;
    using Matrix3 = Eigen::Matrix3d;
    using Matrix4 = Eigen::Matrix4d;

    // 工具坐标系相对于基坐标系旋转 90°
    Matrix3 R;
    R << 0.0, -1.0, 0.0,
        1.0, 0.0, 0.0,
        0.0, 0.0, 1.0;

    // 工具坐标系原点在基坐标系中的位置
    const Vector3 t(1.0, 2.0, 0.0);

    // TODO 1：构造 4x4 齐次变换矩阵
    Matrix4 T = Matrix4::Identity();

    // 提示：
    T.block<3, 3>(0, 0) = R;
    T.block<3, 1>(0, 3) = t;
    assert((T.block<1, 4>(3, 0).isApprox(Eigen::RowVector4d(0.0, 0.0, 0.0, 1.0), 1e-6)));

    // 工具坐标系中的一个点
    const Vector3 p_tool(1.0, 2.0, 0.0);

    // TODO 2：转换为齐次坐标
    Vector4 p_tool_h(1.0, 2.0, 0.0, 1.0);
    // TODO 3：计算基坐标系中的齐次坐标
    const Vector4 p_base_h = T * p_tool_h;

    // TODO 4：提取前三个坐标
    // 提示：p_base_h.head<3>()
    const Vector3 p_base = p_base_h.head<3>();
    // TODO 5：验证计算结果
    // 预期：(-1, 3, 0)
    assert(p_base.isApprox(Vector3(-1.0, 3.0, 0.0), 1e-6));
    // TODO 6：构造齐次方向向量 (1, 0, 0, 0)
    // 验证平移不会影响方向向量
    Vector4 v_tool_h(1.0, 0.0, 0.0, 0.0);
    const Vector4 v_base_h = T * v_tool_h;
    const Vector3 v_base = v_base_h.head<3>();
    // 预期：(0, 1, 0)
    assert(std::abs(v_base.x() - 0.0) < 1e-6);
    assert(std::abs(v_base.y() - 1.0) < 1e-6);
    assert(std::abs(v_base.z() - 0.0) < 1e-6);

    return 0;
}