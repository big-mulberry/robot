#include <chrono>
#include <memory>
#include <exception>

#include "rclcpp/rclcpp.hpp"
#include "robot_status_demo/status_service_client.hpp"

using namespace std::chrono_literals;

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    int exit_code = 0;
    try
    {
        auto node = std::make_shared<StatusServiceClient>();

        // TODO 1：最多等待 Server 5 秒，服务不可用时不发送请求。
        if (!node->waitForServer(5s))
        {
            RCLCPP_ERROR(node->get_logger(), "Service unavailable after waiting up to 5 seconds.");
            rclcpp::shutdown();
            return 1;
        }

        auto future = node->sendRequest();

        // TODO 2：等待响应的同时驱动 Executor，最多等待 5 秒。
        const auto result = rclcpp::spin_until_future_complete(node, future, 5s);
        if (result == rclcpp::FutureReturnCode::SUCCESS)
        {
            // TODO 3：通信成功后读取业务结果。
            const auto response = future.get();
            RCLCPP_INFO(node->get_logger(), "Motion ready: success=%s, message=%s",
                        response->success ? "true" : "false",
                        response->message.c_str());
        }
        else if (result == rclcpp::FutureReturnCode::TIMEOUT)
        {
            RCLCPP_ERROR(node->get_logger(), "Service response timed out after 5 seconds.");
            exit_code = 1;
        }
        else
        {
            RCLCPP_ERROR(node->get_logger(), "Service request interrupted.");
            exit_code = 1;
        }
        // 本程序只查询一次，随后销毁节点和客户端，不保留未完成请求。
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("status_service_client"),
                     "Client failed: %s", e.what());
        exit_code = 1;
    }

    rclcpp::shutdown();
    return exit_code;
}