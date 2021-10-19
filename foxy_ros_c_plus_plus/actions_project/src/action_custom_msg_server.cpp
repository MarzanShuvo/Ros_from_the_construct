#include <functional>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "actions_project/action/project.hpp"
#include "geometry_msgs/msg/twist.hpp"

class CustomActionServer : public rclcpp::Node {
public:
  using Move = actions_project::action::Project;
  using GoalHandleMove = rclcpp_action::ServerGoalHandle<Move>;

  explicit CustomActionServer(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("actions_project_server", options) {
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<Move>(
        this, "action_custom_msg_as",
        std::bind(&CustomActionServer::handle_goal, this, _1, _2),
        std::bind(&CustomActionServer::handle_cancel, this, _1),
        std::bind(&CustomActionServer::handle_accepted, this, _1));

    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  }

private:
  rclcpp_action::Server<Move>::SharedPtr action_server_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

  rclcpp_action::GoalResponse
  handle_goal(const rclcpp_action::GoalUUID &uuid,
              std::shared_ptr<const Move::Goal> goal) {
    RCLCPP_INFO(this->get_logger(), "Received goal request with secs %d",
                goal->goal);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse
  handle_cancel(const std::shared_ptr<GoalHandleMove> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleMove> goal_handle) {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a
    // new thread
    std::thread{std::bind(&CustomActionServer::execute, this, _1), goal_handle}
        .detach();
  }

  void execute(const std::shared_ptr<GoalHandleMove> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Move::Feedback>();
    auto &message = feedback->feedback;
    message = "Starting movement...";
    auto result = std::make_shared<Move::Result>();
    auto move = geometry_msgs::msg::Twist();
    rclcpp::Rate loop_rate(1);

    if (goal->goal == "FORWARD") {

      for (int i = 0; (i < 5) && rclcpp::ok(); ++i) {
        // Check if there is a cancel request
        if (goal_handle->is_canceling()) {
          goal_handle->canceled(result);
          RCLCPP_INFO(this->get_logger(), "Goal canceled");
          return;
        }
        // Move robot forward and send feedback
        message = "Moving forward...";
        move.linear.x = 0.3;
        publisher_->publish(move);
        goal_handle->publish_feedback(feedback);
        RCLCPP_INFO(this->get_logger(), "Publish feedback");

        loop_rate.sleep();
      }
    }

    if (goal->goal == "BACKWARD") {
      for (int i = 0; (i < 5) && rclcpp::ok(); ++i) {
        // Check if there is a cancel request
        if (goal_handle->is_canceling()) {
          goal_handle->canceled(result);
          RCLCPP_INFO(this->get_logger(), "Goal canceled");
          return;
        }
        // Move robot backward and send feedback
        message = "Moving Backward...";
        move.linear.x = -0.3;
        publisher_->publish(move);
        goal_handle->publish_feedback(feedback);
        RCLCPP_INFO(this->get_logger(), "Publish feedback");

        loop_rate.sleep();
      }
    }

    if (goal->goal == "STOP") {

      message = "Stopping...";
      move.linear.x = 0.0;
      publisher_->publish(move);
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(), "Publish feedback");
      loop_rate.sleep();
    }

    // Check if goal is done
    if (rclcpp::ok()) {
      move.linear.x = 0.0;
      publisher_->publish(move);
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
  }
}; // class CustomActionServer

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto action_server = std::make_shared<CustomActionServer>();

  rclcpp::spin(action_server);

  rclcpp::shutdown();
  return 0;
}