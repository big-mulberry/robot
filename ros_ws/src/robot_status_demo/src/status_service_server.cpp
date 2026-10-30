#include "robot_status_demo/status_service_server.hpp"

StatusServiceServer::StatusServiceServer()
    : rclcpp::Node("status_service_server")
{
    service_ = create_service<Trigger>(
        "/robot/is_motion_ready",

        [this](
            const std::shared_ptr<Trigger::Request> request,
            std::shared_ptr<Trigger::Response> response)
        {
            handleRequest(request, response);
        });
}

void StatusServiceServer::handleRequest(
    const std::shared_ptr<Trigger::Request> request,
    std::shared_ptr<Trigger::Response> response)
{
    // Trigger 的 Request 没有字段。
    // 避免编译器产生“未使用参数”警告。
    (void)request;

    // TODO 1：
    // 根据 servo_on_、has_error_、motion_state_
    // 计算 motion_ready。

    const bool motion_ready = servo_on_ && !has_error_ && motion_state_ == "Standstill";

    // TODO 2：
    // 将结果写入 response->success。
    response->success = motion_ready;

    // TODO 3：
    // 根据具体原因填写 response->message。
    std::string msg;
    if(!servo_on_) msg = "Servo is off";
    else if(has_error_) msg = "Controller has an error";
    else if(motion_state_ == "Standstill") msg = "Standstill: motion ready";
    else msg = "Robot is already moving";
    // 示例：
    // "Standstill: motion ready"
    // "Servo is off"
    // "Controller has an error"
    // "Robot is already moving"

    response->message = msg;

    // TODO 4：
    // 使用 RCLCPP_INFO 打印查询结果。
    RCLCPP_INFO(
        get_logger(),
        "Motion readiness query: success=%s, message=%s",
        response->success ? "true" : "false",
        msg.c_str());
}