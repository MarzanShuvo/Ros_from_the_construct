#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
using std::placeholders::_1;

class OdomSubscriber : public rclcpp::Node {
public:
  OdomSubscriber() : Node("odom_subscriber") {
    subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10, std::bind(&OdomSubscriber::topic_callback, this, _1));
  }

private:
  void topic_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "I heard: '%d'", msg->pose.pose.position.x);
  }
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomSubscriber>());
  rclcpp::shutdown();
  return 0;
}