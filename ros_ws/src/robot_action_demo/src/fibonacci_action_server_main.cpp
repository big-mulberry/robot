#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "robot_action_demo/fibonacci_action_server.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node =
        std::make_shared<FibonacciActionServer>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}