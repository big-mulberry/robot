#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "robot_status_demo/status_service_server.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<StatusServiceServer>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}