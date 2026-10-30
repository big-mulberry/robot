
#include <Eigen/Dense>
#include <iostream>

int main()
{
    using Vector3 = Eigen::Vector3d;

    const Vector3 a(1.0, 0.0, 0.0);
    const Vector3 b(0.0, 1.0, 0.0);

    // 1. 计算 a × b
    const Vector3 c = a.cross(b);

    // 2. 验证 c 同时垂直于 a 和 b
    double dot_ac = c.dot(a);
    double dot_bc = c.dot(b);

    // 垂直性验证
    assert(std::abs(dot_ac) < 1e-12);
    assert(std::abs(dot_bc) < 1e-12);

    // 3. 验证交换顺序后方向反转
    const Vector3 revers = b.cross(a);

    assert((revers + c).norm() < 1e-12); // c + (b × a) ≈ 0

    // 4. 模拟旋转关节产生的线速度
    const Vector3 omega(0.0, 0.0, 2.0);
    const Vector3 r(0.5, 0.0, 0.0);

    const Vector3 velociy = omega.cross(r);

    // 5. 输出结果
    std::cout << "向量 a：" << a.transpose() << '\n';
    std::cout << "向量 b：" << b.transpose() << '\n';
    std::cout << "a × b：" << c.transpose() << '\n';
    std::cout << "c · a：" << dot_ac << '\n';
    std::cout << "c · b：" << dot_bc << '\n';
    std::cout << "b × a：" << revers.transpose() << '\n';
    std::cout << "旋转关节角速度：" << omega.transpose() << '\n';
    std::cout << "旋转关节位置：" << r.transpose() << '\n';
    std::cout << "旋转关节产生的线速度：" << velociy.transpose() << '\n';

    return 0;
}
