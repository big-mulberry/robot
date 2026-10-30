
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

// RobotMonitor：模拟机器人上位机状态监控程序。
class RobotMonitor : public rclcpp::Node {
public:
    RobotMonitor()
        : rclcpp::Node("robot_monitor")
    {
        // TODO 4：
        // 调用 create_subscription 创建订阅者。
        //
        // Topic：/robot/status
        // 消息类型：std_msgs::msg::String
        // QoS 深度：10
        //
        // 收到消息时调用 on_status()。
        //
        // 提示：可以用 lambda 捕获 this。
        subscription_ = create_subscription<std_msgs::msg::String>(
            "/robot/status", 10,
            [this](const std_msgs::msg::String::SharedPtr msg)
            {
                on_status(msg);
            });
    }

private:
    void on_status(
        const std_msgs::msg::String::SharedPtr msg)
    {
        // TODO 5：
        // 使用 RCLCPP_INFO 打印 msg->data。
        RCLCPP_INFO(this->get_logger(), "Received: %s", msg->data.c_str());
    }

    // 保存订阅者，维持其生命周期。
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr
        subscription_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<RobotMonitor>();

    // 负责调度订阅回调。
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
