#include "robot_action_demo/fibonacci_action_server.hpp"

#include <chrono>
#include <thread>

using namespace std::chrono_literals;

FibonacciActionServer::FibonacciActionServer()
    : rclcpp::Node("fibonacci_action_server")
{
    action_server_ =
        rclcpp_action::create_server<Fibonacci>(
            this,
            "fibonacci",
            [this](const rclcpp_action::GoalUUID &uuid,
                   std::shared_ptr<const Fibonacci::Goal> goal)
            {
                return handleGoal(uuid, goal);
            },
            [this](std::shared_ptr<GoalHandleFibonacci> goal_handle)
            {
                return handleCancel(goal_handle);
            },
            [this](std::shared_ptr<GoalHandleFibonacci> goal_handle)
            {
                handleAccepted(goal_handle);
            });
}

rclcpp_action::GoalResponse
FibonacciActionServer::handleGoal(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const Fibonacci::Goal> goal)
{
    (void)uuid;
    RCLCPP_INFO(get_logger(), "Received goal: order=%d", goal->order);

    // TODO 1：order <= 0 时拒绝，其余情况接受并立即执行。
    if (goal->order <= 0 || goal->order > 47)
    {
        return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse
FibonacciActionServer::handleCancel(
    const std::shared_ptr<GoalHandleFibonacci> goal_handle)
{
    (void)goal_handle;
    RCLCPP_INFO(get_logger(), "Received cancel request");

    // TODO 2：本课允许取消所有正在执行的 Goal。
    return rclcpp_action::CancelResponse::ACCEPT;
}

void FibonacciActionServer::handleAccepted(
    const std::shared_ptr<GoalHandleFibonacci> goal_handle)
{
    std::thread(
        [this, goal_handle]()
        {
            execute(goal_handle);
        })
        .detach();
}
void FibonacciActionServer::execute(
    const std::shared_ptr<GoalHandleFibonacci> goal_handle)
{
    auto result =
        std::make_shared<Fibonacci::Result>();

    auto feedback =
        std::make_shared<Fibonacci::Feedback>();

    const auto goal = goal_handle->get_goal();

    // TODO 3:
    // 初始化 Fibonacci 数列。
    // 根据 goal->order 控制最终长度。
    if (goal->order > 47)
    {
        RCLCPP_ERROR(get_logger(), "order exceeds int32_t range");
        goal_handle->abort(result);
        return;
    }
    // 每次迭代：
    for (int i = 0; i < goal->order && rclcpp::ok(); ++i)
    {
        // 1. 检查 goal_handle->is_canceling()
        // 2. 如果取消，填写 result->sequence
        //    并调用 goal_handle->canceled(result)，然后 return
        if (goal_handle->is_canceling())
        {
            result->sequence = feedback->sequence;
            goal_handle->canceled(result);
            return;
        }
        // 3. 计算下一项
        int32_t next;
        if (i == 0)
        {
            next = 0;
        }
        else if (i == 1)
        {
            next = 1;
        }
        else
        {
            const auto &sequence = feedback->sequence;
            next = sequence[i - 1] + sequence[i - 2];
        }
        // 4. 更新 feedback->sequence
        feedback->sequence.push_back(next);
        // 5. 调用 publish_feedback(feedback)
        goal_handle->publish_feedback(feedback);
        // 6. 使用 std::this_thread::sleep_for(500ms)
        //    模拟耗时任务
        if (i + 1 < goal->order)
        {
            std::this_thread::sleep_for(500ms);
        }
    }
    // ROS 关闭时，不再发送结果。
    if (!rclcpp::ok())
    {
        return;
    }
    // 循环结束后：
    // 7. 填写 result->sequence
    result->sequence = feedback->sequence;
    // 8. 调用 goal_handle->succeed(result)
    if (goal_handle->is_canceling())
    {
        goal_handle->canceled(result);
        return;
    }
    goal_handle->succeed(result);
}