#ifndef BB8_CPP_UNIT3_3_SERVICES__MOVE_BB8_HPP_
#define BB8_CPP_UNIT3_3_SERVICES__MOVE_BB8_HPP_


#include <iostream>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <std_srvs/srv/empty.hpp>


class MoveBB8_ROS2
{
    private:
        // ROS Publishers
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;
        
        
        // Other variables
        bool running_;
        int state_;
        int rate_hz_;
        float duration_;
        int times_;
        
        
        
        // ROS Node
        std::shared_ptr<rclcpp::Node> node_;

        
    public:
    
        MoveBB8_ROS2(std::shared_ptr<rclcpp::Node> node);
        ~MoveBB8_ROS2(void);
        void rateSleep(void);
        geometry_msgs::msg::Twist getStateVelocity();
        bool runTimeStateMachine(void);
        void changeState(int state, float duration);
        
};

int main(int argc, char * argv[]);

#endif  // BB8_CPP_UNIT3_3_SERVICES__MOVE_BB8_HPP_