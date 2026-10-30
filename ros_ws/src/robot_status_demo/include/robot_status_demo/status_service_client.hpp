#pragma once

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

class StatusServiceClient : public rclcpp::Node
{
public:
    using Trigger = std_srvs::srv::Trigger;
    using SharedFuture =
        rclcpp::Client<Trigger>::SharedFuture;

    StatusServiceClient();

    bool waitForServer(
        std::chrono::seconds timeout);

    SharedFuture sendRequest();

private:
    rclcpp::Client<Trigger>::SharedPtr client_;
};