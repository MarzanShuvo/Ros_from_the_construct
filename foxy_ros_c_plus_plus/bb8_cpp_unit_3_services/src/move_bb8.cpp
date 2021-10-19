#include <bb8_cpp_unit_3_services/move_bb8.hpp>

using namespace std::chrono_literals;

MoveBB8_ROS2::MoveBB8_ROS2(std::shared_ptr<rclcpp::Node> node) {
  // Other variables
  this->running_ = true;
  this->state_ = 0;
  this->rate_hz_ = 20;
  this->duration_ = 0;
  this->times_ = 5 * 1;

  this->node_ = node;
  RCLCPP_INFO(this->node_->get_logger(), "Starting ");

  // ASSERT_FALSE(this->loop_rate.is_steady());
  // Rate
  rclcpp::WallRate loop_rate(500ms);
  // std::cout << " loop_rate is: " << typeid(loop_rate).name() << '\n';

  // ROS Publishers
  this->pub_cmd_vel_ =
      this->node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
}

MoveBB8_ROS2::~MoveBB8_ROS2(void) {}

void MoveBB8_ROS2::rateSleep(void) {
  // this->loop_rate.sleep();
}

geometry_msgs::msg::Twist MoveBB8_ROS2::getStateVelocity() {
  geometry_msgs::msg::Twist vel;
  switch (state_) {
  case 0:
    // go ahead
    vel.linear.x = 0.2;
    vel.angular.z = 0;
    break;
  case 1:
    // stop
    vel.linear.x = 0;
    vel.angular.z = 0;
    break;
  case 2:
    // turn right
    vel.linear.x = 0;
    vel.angular.z = 0.2;
    break;
  case 3:
    // stop
    vel.linear.x = 0;
    vel.angular.z = 0;
    break;
  }
  return vel;
}

bool MoveBB8_ROS2::runTimeStateMachine(void) {
  geometry_msgs::msg::Twist vel;
  // running_ = true;

  if (!running_) {
    vel.linear.x = 0;
    vel.angular.z = 0;
    this->pub_cmd_vel_->publish(vel);
    return true;
  }

  vel = this->getStateVelocity();

  this->pub_cmd_vel_->publish(vel);

  duration_ -= 1 / (float)rate_hz_;

  RCLCPP_INFO(this->node_->get_logger(),
              "State [%d], Vel[%.2f, %.2f], Duration [%.2f]", state_,
              vel.linear.x, vel.angular.z, duration_);

  if (duration_ <= 0) {
    RCLCPP_INFO(this->node_->get_logger(),
                "Duration finished, changing of STATE>>>");
    float state_duration[4] = {2.0, 3.8, 4.0, 0.1};
    int next_state = state_ + 1;
    if (state_ == 3) {
      RCLCPP_INFO(this->node_->get_logger(),
                  "Got to State =[%d], Redusing by one times: [%d]", state_,
                  times_);
      next_state = 0;
      times_ -= 1;
      RCLCPP_INFO(this->node_->get_logger(), "new times: [%d]", times_);
    }
    int next_state_duration = state_duration[next_state];
    this->changeState(next_state, next_state_duration);
  }

  if (times_ == 0) {
    RCLCPP_INFO(this->node_->get_logger(),
                "STOPPING, Reached  times = 0 : [%d]", times_);
    running_ = false;
    vel.linear.x = 0;
    vel.angular.z = 0;
    this->pub_cmd_vel_->publish(vel);
  }

  return false;
}

void MoveBB8_ROS2::changeState(int state, float duration) {
  state_ = state;
  duration_ = duration;
  RCLCPP_INFO(this->node_->get_logger(), "Change to state [%d]", state_);
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node =
      rclcpp::Node::make_shared("MoveBB8_ROS2");

  RCLCPP_INFO(node->get_logger(), "STart Init MoveBB8_ROS2");
  MoveBB8_ROS2 moveBB8_ROS2(node);

  rclcpp::Rate loop_rate(20);

  bool finished_statemachine = false;

  while (rclcpp::ok() && !finished_statemachine) {
    finished_statemachine = moveBB8_ROS2.runTimeStateMachine();
    loop_rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}