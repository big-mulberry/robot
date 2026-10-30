#pragma once

#include <cstdint>
#include <memory>

#include "example_interfaces/action/fibonacci.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class FibonacciActionServer : public rclcpp::Node
{
public:
    using Fibonacci = example_interfaces::action::Fibonacci;

    // server 侧，某个 goal 的执行句柄
    // 你可以通过它：
    // - 读取 Goal的执行状态
    // - 判断是否正在取消。
    // - 发布 Feedback。
    // - 设置最终 Result。
    using GoalHandleFibonacci =
        rclcpp_action::ServerGoalHandle<Fibonacci>;

    FibonacciActionServer();

private:
    rclcpp_action::GoalResponse handleGoal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const Fibonacci::Goal> goal);

    rclcpp_action::CancelResponse handleCancel(
        const std::shared_ptr<GoalHandleFibonacci> goal_handle);

    void handleAccepted(
        const std::shared_ptr<GoalHandleFibonacci> goal_handle);

    void execute(
        const std::shared_ptr<GoalHandleFibonacci> goal_handle);

    rclcpp_action::Server<Fibonacci>::SharedPtr action_server_;
};