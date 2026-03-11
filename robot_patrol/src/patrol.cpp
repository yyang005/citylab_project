#include "robot_patrol/patrol.h"
#include "rclcpp/logging.hpp"
#include <cmath>
#include <cstddef>
#include <math.h>

Patrol::Patrol():Node("Patrol_node"){
    
    direction_ = 0.0;
    
    auto qos = rclcpp::QoS(10).reliability(rclcpp::ReliabilityPolicy::Reliable);
    scan_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan",
        qos,
        std::bind(&Patrol::scan_callback, this, std::placeholders::_1)
    );

    twist_pub = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // Setup a timer 
    auto timer_period = std::chrono::milliseconds(100); // 0.1 second
    timer_ = this->create_wall_timer(timer_period, std::bind(&Patrol::timer_callback, this));
};


void Patrol::scan_callback(sensor_msgs::msg::LaserScan::ConstSharedPtr msg){
    RCLCPP_INFO(this->get_logger(), "min range: %f", msg->range_min);
    RCLCPP_INFO(this->get_logger(), "max range: %f", msg->range_max);

    float front_angle = 0.17; //(-10 degree ~ 10 degree)
    float max_distance = 0.0;
    float best_angle = 0.0;
    bool is_blocked = false;

    for (size_t i = 0; i < msg->ranges.size(); i++){
        float angle = msg->angle_min + i * msg->angle_increment;
        // mapping from (0, 2pi) to (-pi, pi)
        if (angle > M_PI) angle -= 2*M_PI;

        // discard invalid scans
        if (!std::isfinite(msg->ranges[i]) 
            || msg->ranges[i] < msg->range_min 
            || msg->ranges[i] > msg->range_max) continue; 

        // filter out the front rays 
        if (angle >= -front_angle && angle <= front_angle) {
            // check if it is too close to the wall or obstacle
            if (msg->ranges[i] < 0.35) {
                is_blocked = true;
            }
        }

        if (msg->ranges[i] > max_distance) {
            max_distance = msg->ranges[i];
            best_angle = angle;
        }
    }

    // int side = (msg->ranges.size())/5;
    // float safe_side_dis = 0.05;
    // bool close_to_left = false, close_to_right = false;
    // // check if it is too close to left fence
    // for (int i = 0; i <= side; i++){
    //     if (!std::isfinite(msg->ranges[i]) 
    //         || msg->ranges[i] < msg->range_min 
    //         || msg->ranges[i] > msg->range_max) continue; // discard invalid scans

    //     if (msg->ranges[i] < safe_side_dis) {
    //         close_to_left = true;
    //         break;
    //     }
    // }
    // // check if it is too close to the right fence
    // for (int i = msg->ranges.size()-side; i <= msg->ranges.size(); i++){
    //     if (!std::isfinite(msg->ranges[i]) 
    //         || msg->ranges[i] < msg->range_min 
    //         || msg->ranges[i] > msg->range_max) continue; // discard invalid scans

    //     if (msg->ranges[i] < safe_side_dis) {
    //         close_to_right = true;
    //         break;
    //     }
    // }

    if (is_blocked){
        direction_ = best_angle*2;
    }else{
        direction_ = 0; // move forward
        // if (close_to_left ) direction_ = -0.1;
        // else if (close_to_right ) direction_ = 0.1;
    }
};

void Patrol::timer_callback(){
    geometry_msgs::msg::Twist twist_msg;
    twist_msg.linear.x = 0.1; // always 0.1
    twist_msg.angular.z = direction_/2;
    twist_pub->publish(twist_msg);

    RCLCPP_INFO(this->get_logger(), "linear: %f", twist_msg.linear.x);
    RCLCPP_INFO(this->get_logger(), "angular: %f", twist_msg.angular.z);
}
