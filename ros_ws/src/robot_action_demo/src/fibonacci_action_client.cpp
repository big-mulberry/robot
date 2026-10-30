#include "robot_action_demo/fibonacci_action_client.hpp"

#include <chrono>
#include <functional>
#include <sstream>

using namespace std::chrono_literals;

FibonacciActionClient::FibonacciActionClient()
    : rclcpp::Node("fibonacci_action_client")
{
    action_client_ =
        rclcpp_action::create_client<Fibonacci>(
            this,
            "fibonacci");
}

void FibonacciActionClient::sendGoal(
    std::int32_t order,
    std::chrono::milliseconds cancel_after)
{
    cancel_after_ = cancel_after;

    // wait_for_action_server() 等待发现对应的 Action Server。
    // 5s 是最长等待时间，不是任务执行时间。
    if (!action_client_->wait_for_action_server(5s))
    {
        RCLCPP_ERROR(
            get_logger(),
            "Fibonacci Action Server is unavailable");

        rclcpp::shutdown();
        return;
    }

    Fibonacci::Goal goal;
    goal.order = order;

    rclcpp_action::Client<Fibonacci>::SendGoalOptions options;

    options.goal_response_callback =
        [this](const GoalHandleFibonacci::SharedPtr& goal_handle)
        {
            goalResponseCallback(goal_handle);
        };

    options.feedback_callback =
        [this](
            GoalHandleFibonacci::SharedPtr goal_handle,
            const std::shared_ptr<const Fibonacci::Feedback> feedback)
        {
            feedbackCallback(goal_handle, feedback);
        };

    options.result_callback =
        [this](const GoalHandleFibonacci::WrappedResult& result)
        {
            resultCallback(result);
        };

    RCLCPP_INFO(
        get_logger(),
        "Sending goal: order=%d",
        order);

    // 该调用立即返回；后续事件由 Executor 分派给以上回调。
    action_client_->async_send_goal(goal, options);
}

void FibonacciActionClient::goalResponseCallback(
    const GoalHandleFibonacci::SharedPtr& goal_handle)
{
    // TODO 1：
    // 1. goal_handle 为空时，说明 Goal 被拒绝。
    // 2. 被拒绝时输出日志并调用 rclcpp::shutdown()。
    // 3. 被接受时保存到成员 goal_handle_。
    // 4. cancel_after_ > 0ms 时创建定时器，
    //    到期后调用 cancelCurrentGoal()。
    if (!goal_handle)
    {
        RCLCPP_ERROR(get_logger(), "Goal rejected by server");
        rclcpp::shutdown();
        return;
    }

    goal_handle_ = goal_handle;
    RCLCPP_INFO(get_logger(), "Goal accepted by server");
    if (cancel_after_ > 0ms)
    {
        cancel_timer_ = create_wall_timer(cancel_after_, [this]()
        {
            cancelCurrentGoal();
        });
    }
}

void FibonacciActionClient::feedbackCallback(
    GoalHandleFibonacci::SharedPtr goal_handle,
    const std::shared_ptr<const Fibonacci::Feedback> feedback)
{
    (void)goal_handle;

    // TODO 2：
    // 输出 feedback->sequence。
    //
    // 注意：当前 Humble 环境的字段名是 sequence，
    // 不是 partial_sequence。
    std::ostringstream sequence;
    for (const auto value : feedback->sequence)
    {
        sequence << value << ' ';
    }
    RCLCPP_INFO(get_logger(), "Feedback sequence: [%s]", sequence.str().c_str());
}

void FibonacciActionClient::cancelCurrentGoal()
{
    // 定时器默认会周期触发，本 Demo 只取消一次。
    if (cancel_timer_)
    {
        cancel_timer_->cancel();
    }

    // TODO 3：
    // 1. 检查 goal_handle_ 是否为空。
    // 2. 调用 action_client_->async_cancel_goal(goal_handle_)。
    // 3. 输出“取消请求已发送”的日志。
    //
    // 注意：发出取消请求不等于任务已经进入 CANCELED 终态。
    if (!goal_handle_)
    {
        RCLCPP_WARN(get_logger(), "No accepted goal to cancel");
        return;
    }
    action_client_->async_cancel_goal(
    goal_handle_,
    [this](auto response)
    {
        if (!response->goals_canceling.empty())
        {
            RCLCPP_INFO(
                get_logger(),
                "取消请求已获准，等待任务停止");
        }
        else
        {
            RCLCPP_WARN(
                get_logger(),
                "未进入取消状态，返回码=%d",
                static_cast<int>(response->return_code));
        }
    });
    RCLCPP_INFO(get_logger(), "Cancel request sent");
}

void FibonacciActionClient::resultCallback(
    const GoalHandleFibonacci::WrappedResult& result)
{
    if (cancel_timer_)
    {
        cancel_timer_->cancel();
    }

    // TODO 4：
    // 根据 result.code 区分：
    //
    // rclcpp_action::ResultCode::SUCCEEDED
    // rclcpp_action::ResultCode::ABORTED
    // rclcpp_action::ResultCode::CANCELED
    // 其他未知状态
    //
    // 成功或取消时输出 result.result->sequence。
    // 处理结束后调用 rclcpp::shutdown()。
    bool print_sequence = false;
    switch (result.code)
    {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(get_logger(), "Goal succeeded");
            print_sequence = true;
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(get_logger(), "Goal aborted");
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_INFO(get_logger(), "Goal canceled");
            print_sequence = true;
            break;
        default:
            RCLCPP_ERROR(get_logger(), "Unknown goal result code");
            break;
    }
    if (print_sequence)
    {
        if (result.result)
        {
            std::ostringstream sequence;
            for (const auto value : result.result->sequence)
            {
                sequence << value << ' ';
            }
            RCLCPP_INFO(get_logger(), "Result sequence: [%s]", sequence.str().c_str());
        }
        else
        {
            RCLCPP_ERROR(get_logger(), "Result payload is null");
        }
    }
    rclcpp::shutdown();
}