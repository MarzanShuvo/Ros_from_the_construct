#include <inttypes.h>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "unit_8_services_custom_msgs/srv/my_custom_service_message.hpp"

using MyCustomServiceMessage = unit_8_services_custom_msgs::srv::MyCustomServiceMessage;
rclcpp::Node::SharedPtr g_node = nullptr;

void handle_service(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<MyCustomServiceMessage::Request> request,
    const std::shared_ptr<MyCustomServiceMessage::Response> response)
{
    (void)request_header;
    RCLCPP_INFO(g_node->get_logger(),"Incoming request\nradius: %f", request->radius);
    RCLCPP_INFO(g_node->get_logger(),"Incoming request\nrepetitions: %i", request->repetitions);

    response->success = true;
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    g_node = rclcpp::Node::make_shared("custom_service_server");
    auto server = g_node->create_service<MyCustomServiceMessage>("/my_custom_service", handle_service);
    RCLCPP_INFO(g_node->get_logger(),"CustomServiceServer...READY");
    rclcpp::spin(g_node);
    rclcpp::shutdown();
    g_node = nullptr;
    return 0;
}