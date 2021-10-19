#include <chrono>
#include <cinttypes>
#include <iostream>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "unit_8_services_custom_msgs/srv/my_custom_service_message.hpp"

using MyCustomServiceMessage = unit_8_services_custom_msgs::srv::MyCustomServiceMessage;

MyCustomServiceMessage::Response::SharedPtr send_request(
  rclcpp::Node::SharedPtr node,
  rclcpp::Client<MyCustomServiceMessage>::SharedPtr client,
  MyCustomServiceMessage::Request::SharedPtr request)
{

  auto result = client->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(node->get_logger(),"Client request->radius: %f", request->radius);
    RCLCPP_INFO(node->get_logger(),"Client request->repetitions: %i", request->repetitions);

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

    auto node = rclcpp::Node::make_shared("custom_service_client");
    auto topic = std::string("/my_custom_service");
    auto client = node->create_client<MyCustomServiceMessage>(topic);
    auto request = std::make_shared<MyCustomServiceMessage::Request>();

    //  Fill In The variables of the Custom Service Message
    request->radius = 2.3;
    request->repetitions = 2;

    while (!client->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(node->get_logger(), "service not available, waiting again...");
    }

    auto result = send_request(node, client, request);
    if (result) {

        auto result_str = result->success ? "True" : "False";

        RCLCPP_INFO(node->get_logger(), "Result-Success : %s", result_str);
    } else {
        RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for response. Exiting.");
    }

    rclcpp::shutdown();
    return 0;
}