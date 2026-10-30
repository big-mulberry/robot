#include <Eigen/Dense>

#include <iostream>

int main()
{
    using Vector3 = Eigen::Vector3d;

    // 1. 定义两个三维空间点
    const Vector3 point_a(1.0, 2.0, 3.0);
    const Vector3 point_b(4.0, 6.0, 3.0);

    // 2. 计算从 A 指向 B 的位移向量
    const Vector3 displacement = point_b - point_a;

    // 3. 计算位移向量的模长
    const double length = displacement.norm();

    // 4. 计算单位方向向量
    // 零向量没有方向；保留零向量，避免除以零。
    Vector3 unit_direction = Vector3::Zero();
    if (length > 0.0)
    {
        unit_direction = displacement / length;
    }

    // 5. 输出计算结果
    std::cout << "Point A: "
              << point_a.transpose() << '\n';

    std::cout << "Point B: "
              << point_b.transpose() << '\n';

    std::cout << "Displacement: "
              << displacement.transpose() << '\n';

    std::cout << "Length: " << length << '\n';

    if (length > 0.0)
    {
        std::cout << "Unit direction: "
                  << unit_direction.transpose() << '\n';
    }
    else
    {
        std::cout << "Unit direction: undefined (zero displacement)\n";
    }

    // 6. 定义平移向量
    const Vector3 translation(10.0, 20.0, 30.0);

    // 7. 计算平移后的两个点
    // TODO: point_a_new = ?
    auto point_a_new = point_a + translation;
    // TODO: point_b_new = ?
    auto point_b_new = point_b + translation;

    // 8. 计算平移后的位移向量
    // TODO: displacement_new = ?
    auto displacement_new = point_b_new - point_a_new;

    // 9. 比较平移前后的位移向量
    // TODO: 输出两个位移向量
    std::cout << "Displacement before translation: "
              << displacement.transpose() << '\n';
    std::cout << "Displacement after translation: "
              << displacement_new.transpose() << '\n';
    // TODO: 判断它们是否相等
    if (displacement.isApprox(displacement_new))
    {
        std::cout << "The displacement vectors are equal after translation.\n";
    }
    else
    {
        std::cout << "The displacement vectors are NOT equal after translation.\n";
    }
    return 0;
}
