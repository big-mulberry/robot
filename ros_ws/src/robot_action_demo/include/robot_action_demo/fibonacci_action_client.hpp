#pragma once

#include <chrono>
#include <cstdint>
#include <memory>

#include "example_interfaces/action/fibonacci.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class FibonacciActionClient : public rclcpp::Node
{
public:
    using Fibonacci = example_interfaces::action::Fibonacci;
    using GoalHandleFibonacci =
        rclcpp_action::ClientGoalHandle<Fibonacci>;

    FibonacciActionClient();

    /**
     * @param order 请求的 Fibonacci 数列长度，无量纲
     * @param cancel_after 取消等待时间；0ms 表示不主动取消
     */
    void sendGoal(
        std::int32_t order,
        std::chrono::milliseconds cancel_after);

private:
    void goalResponseCallback(
        const GoalHandleFibonacci::SharedPtr& goal_handle);

    void feedbackCallback(
        GoalHandleFibonacci::SharedPtr goal_handle,
        const std::shared_ptr<const Fibonacci::Feedback> feedback);

    void resultCallback(
        const GoalHandleFibonacci::WrappedResult& result);

    void cancelCurrentGoal();

    rclcpp_action::Client<Fibonacci>::SharedPtr action_client_;

    // 保存已被 Server 接受的 Goal，供取消操作使用。
    GoalHandleFibonacci::SharedPtr goal_handle_;

    rclcpp::TimerBase::SharedPtr cancel_timer_;
    std::chrono::milliseconds cancel_after_{0};
};