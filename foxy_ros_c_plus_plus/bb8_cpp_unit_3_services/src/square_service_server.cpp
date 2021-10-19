#include "bb8_cpp_unit_3_services/move_bb8.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"
#include <inttypes.h>
#include <memory>

using Empty = std_srvs::srv::Empty;
rclcpp::Node::SharedPtr g_node = nullptr;

void my_handle_service(const std::shared_ptr<rmw_request_id_t> request_header,
                       const std::shared_ptr<Empty::Request> request,
                       const std::shared_ptr<Empty::Response> response) {
  (void)request_header;
  RCLCPP_INFO(g_node->get_logger(), "bb8_make_square_service has been called");

  MoveBB8_ROS2 moveBB8_ROS2(g_node);

  rclcpp::Rate loop_rate(20);

  bool finished_statemachine = false;

  while (rclcpp::ok() && !finished_statemachine) {
    finished_statemachine = moveBB8_ROS2.runTimeStateMachine();
    loop_rate.sleep();
  }
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  g_node = rclcpp::Node::make_shared("empty_service_server");
  auto server = g_node->create_service<Empty>("/bb8_make_square_service",
                                              my_handle_service);
  rclcpp::spin(g_node);
  rclcpp::shutdown();
  g_node = nullptr;
  return 0;
}