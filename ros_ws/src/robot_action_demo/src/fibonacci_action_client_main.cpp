#include <chrono>
#include <cstdint>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "robot_action_demo/fibonacci_action_client.hpp"

int main(int argc, char **argv)
{
    std::string line;
    while (true)
    {
        std::cout << "\n请输入 order 和取消时间(ms，0表示不取消)，输入 q 退出：" << std::flush;
        if (!std::getline(std::cin, line) || line == "q" || line == "Q")
        {
            break;
        }

        std::int32_t order;
        std::int64_t cancel_ms;
        std::string extra;
        std::istringstream input(line);
        if (!(input >> order >> cancel_ms) || (input >> extra) ||
            /*order <= 0 ||*/ cancel_ms < 0)
        {
            std::cout << "请输入两个整数：order > 0，取消时间 >= 0。例如：10 1500\n";
            continue;
        }

        // 现有客户端回调会 shutdown；每轮重新初始化并创建节点。
        rclcpp::init(argc, argv);
        {
            auto node = std::make_shared<FibonacciActionClient>();
            node->sendGoal(order, std::chrono::milliseconds{cancel_ms});
            if (rclcpp::ok())
            {
                rclcpp::spin(node);
            }
            if (rclcpp::ok())
            {
                rclcpp::shutdown();
            }
        }
    }
    return 0;
}