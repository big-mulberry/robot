
#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

// 使用 chrono 字面量，例如 1000ms。
using namespace std::chrono_literals;

// RobotController 是一个 ROS 2 Node。
// 继承 rclcpp::Node 后，可以使用 ROS 2 的通信接口。
class RobotController : public rclcpp::Node
{
public:
    RobotController()
        : rclcpp::Node("robot_controller")
    {
        // 第一个参数：Topic 名称。
        // 第二个参数：QoS 队列深度。
        // 10 不是发布频率。
        publisher_ =
            create_publisher<std_msgs::msg::String>(
                "/robot/status", 10);

        // 创建周期定时器。
        // 每隔约 1000ms 请求执行一次回调。
        // 这不是硬实时定时器。
        timer_ = create_wall_timer(1000ms, [this](){ on_timer(); });
    }

private:
    void on_timer()
    {
        static int count = 0;
        // 创建 ROS 2 消息对象。
        std_msgs::msg::String msg;

        // TODO 1：
        // 将 msg.data 设置为 "Standstill"。
        if(count % 2 == 0)
        {
            msg.data = "Moving " + std::to_string(count);
        }else if(count % 3 == 0)
        {
            msg.data = "Stopping " + std::to_string(count);
        }else
        {
            msg.data = "Standstill " + std::to_string(count);
        }
        count++;
        // TODO 2：
        // 通过 publisher_ 发布 msg。
        publisher_->publish(msg);
        // TODO 3：
        // 使用 RCLCPP_INFO 打印 msg.data。
        RCLCPP_INFO(this->get_logger(), "Publishing: %s", msg.data.c_str());
    }

    // 发布者对象。
    // 保存为成员变量，确保它不会提前销毁。
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr
        publisher_;

    // ROS 2 定时器对象。
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    // 初始化 ROS 2。
    rclcpp::init(argc, argv);

    // 创建节点对象。
    auto node = std::make_shared<RobotController>();

    // 让 Executor 持续处理回调。
    // 在这个例子中，主要处理 Timer 回调。
    rclcpp::spin(node);

    // 退出时清理 ROS 2 运行时。
    rclcpp::shutdown();
    return 0;
}
