#include <inttypes.h>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"

using Empty = std_srvs::srv::Empty;
rclcpp::Node::SharedPtr g_node = nullptr;

void my_handle_service(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<Empty::Request> request,
    const std::shared_ptr<Empty::Response> response)
{
    (void)request_header;
    RCLCPP_INFO(g_node->get_logger(),"My_callback has been called");
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    g_node = rclcpp::Node::make_shared("empty_service_server");
    auto server = g_node->create_service<Empty>("/my_service", my_handle_service);
    rclcpp::spin(g_node);
    rclcpp::shutdown();
    g_node = nullptr;
    return 0;
}