#include "robot_status_demo/status_service_client.hpp"

StatusServiceClient::StatusServiceClient()
    : rclcpp::Node("status_service_client")
{
    // TODO 1：
    // 创建 /robot/is_motion_ready 的 Trigger Client。
    client_ = create_client<Trigger>(
        "/robot/is_motion_ready");
}

bool StatusServiceClient::waitForServer(
    std::chrono::seconds timeout)
{
    // TODO 2：
    // 调用 client_->wait_for_service(timeout)。
    return client_->wait_for_service(timeout);
}

StatusServiceClient::SharedFuture
StatusServiceClient::sendRequest()
{
    // Trigger Request 没有数据字段。
    auto request =
        std::make_shared<Trigger::Request>();

    // TODO 3：
    // 使用 async_send_request() 发出请求并返回 future。
    auto future = client_->async_send_request(request);
    return future.future.share();
}