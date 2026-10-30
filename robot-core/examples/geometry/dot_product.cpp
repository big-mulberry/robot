
#include <Eigen/Dense>
#include <cmath>
#include <iostream>

int main()
{
    using Vector3 = Eigen::Vector3d;

    const Vector3 a(3.0, 4.0, 0.0);
    const Vector3 b(1.0, 0.0, 0.0);

    // 1. 计算点积
    double dot_product = a.dot(b);

    // 2. 计算两个向量的模长
    double norm_a = a.norm();
    double norm_b = b.norm();

    // 3. 计算夹角余弦
    double cos_theta = dot_product / (norm_a * norm_b);

    // 4. 计算夹角（弧度）
    double theta = std::acos(cos_theta);
    // 提示：std::acos()

    // 5. 将弧度转换为角度
    double theta_deg = theta * (180.0 / M_PI);

    // 6. 计算 a 在 b 方向上的有符号投影长度
    double projection = dot_product / norm_b;

    // 7. 输出结果
    std::cout << "向量 a：" << a.transpose() << '\n';
    std::cout << "向量 b：" << b.transpose() << '\n';
    std::cout << "点积：" << dot_product << '\n';
    std::cout << "向量 a 的模长：" << norm_a << '\n';
    std::cout << "向量 b 的模长：" << norm_b << '\n';
    std::cout << "夹角余弦：" << cos_theta << '\n';
    std::cout << "夹角（弧度）：" << theta << '\n';
    std::cout << "夹角（角度）：" << theta_deg << "°\n";
    std::cout << "a 在 b 方向上的有符号投影长度：" << projection << '\n';

    return 0;
}
