#pragma once

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

class StatusServiceServer : public rclcpp::Node
{
public:
    StatusServiceServer();

private:
    using Trigger = std_srvs::srv::Trigger;

    void handleRequest(
        const std::shared_ptr<Trigger::Request> request,
        std::shared_ptr<Trigger::Response> response);

    rclcpp::Service<Trigger>::SharedPtr service_;

    // 模拟机器人控制器内部状态。
    bool servo_on_{true};
    bool has_error_{false};
    std::string motion_state_{"Standstill"};
};