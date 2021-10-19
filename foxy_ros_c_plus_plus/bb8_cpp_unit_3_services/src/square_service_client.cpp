#include <chrono>
#include <cinttypes>
#include <iostream>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"


std_srvs::srv::Empty::Response::SharedPtr send_request(
  rclcpp::Node::SharedPtr node,
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr client,
  std_srvs::srv::Empty::Request::SharedPtr request)
{

  auto result = client->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::executor::FutureReturnCode::SUCCESS)
  {

    RCLCPP_INFO(node->get_logger(), "Calling service...");
    return result.get();
  } else {
    RCLCPP_ERROR(node->get_logger(), "service call failed :(");
    return NULL;
  }

}

int main(int argc, char ** argv)
{
    // Force flush of the stdout buffer.
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("square_service_client");
    auto service_name = std::string("/bb8_make_square_service");
    auto client = node->create_client<std_srvs::srv::Empty>(service_name);
    auto request = std::make_shared<std_srvs::srv::Empty::Request>();

    //  In this case its an empty, so nothing to be filled in the request.

    while (!client->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(node->get_logger(), "service not available, waiting again...");
    }

    auto result = send_request(node, client, request);
    if (result) {
        RCLCPP_INFO(node->get_logger(), "Result-Success.");
    } else {
        RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for response. Exiting.");
    }

    rclcpp::shutdown();
    return 0;
}