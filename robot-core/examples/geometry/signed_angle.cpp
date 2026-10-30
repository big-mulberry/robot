#include <Eigen/Dense>
#include <cmath>
#include <iostream>

using Vector3 = Eigen::Vector3d;

// 计算绕指定单位法向量的有符号夹角
// 前提：a、b 非零，均位于垂直于 normal 的平面
double signedAngle(
    const Vector3 &a,
    const Vector3 &b,
    const Vector3 &normal)
{
    // TODO 1：计算点积
    const double x = a.dot(b);

    // TODO 2：计算叉积在法向量方向上的分量
    const double y = normal.dot(a.cross(b));

    // TODO 3：利用 std::atan2 返回弧度
    return std::atan2(y, x);
}

int main()
{
    const Vector3 a(1.0, 0.0, 0.0);
    const Vector3 b(0.0, 1.0, 0.0);
    const Vector3 normal(0.0, 0.0, 1.0);

    // TODO 4：计算 a 到 b 的有符号夹角
    const double angle = signedAngle(a, b, normal);

    // TODO 5：转换为角度
    // 提示：180.0 / std::acos(-1.0)
    const double angle_degrees = angle * 180.0 / M_PI;
    std::cout << "a转b有符号夹角（弧度）：" << angle << '\n';
    std::cout << "a转b有符号夹角（角度）：" << angle_degrees << '\n';

    // TODO 6：交换 a、b，重新计算夹角
    const double angle_reversed = signedAngle(b, a, normal);
    const double angle_reversed_degrees = angle_reversed * 180.0 / M_PI;
    std::cout << "b转a有符号夹角（弧度）：" << angle_reversed << '\n';
    std::cout << "b转a有符号夹角（角度）：" << angle_reversed_degrees << '\n';
    // TODO 7：将 normal 改为 (0,0,-1)，观察结果
    const Vector3 normal_reversed(0.0, 0.0, -1.0);
    const double angle2 = signedAngle(a, b, normal_reversed);
    const double angle2_degrees = angle2 * 180.0 / M_PI;
    std::cout << "a转b有符号夹角（法向量反向，弧度）：" << angle2 << '\n';
    std::cout << "a转b有符号夹角（法向量反向，角度）：" << angle2_degrees << '\n';

    return 0;
}