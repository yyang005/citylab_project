#include "robot_patrol/patrol.h"
#include "rclcpp/logging.hpp"
#include <cstddef>
#include <math.h>

Patrol::Patrol():Node("Patrol_node"){
    
    direction_ = 0.0;
    
    auto qos = rclcpp::QoS(10).reliability(rclcpp::ReliabilityPolicy::Reliable);
    scan_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/fastbot_1/scan",
        qos,
        std::bind(&Patrol::scan_callback, this, std::placeholders::_1)
    );

    twist_pub = this->create_publisher<geometry_msgs::msg::Twist>("/fastbot_1/cmd_vel", 10);

    // Setup a timer 
    auto timer_period = std::chrono::milliseconds(100); // 0.1 second
    timer_ = this->create_wall_timer(timer_period, std::bind(&Patrol::timer_callback, this));
};


void Patrol::scan_callback(sensor_msgs::msg::LaserScan::ConstSharedPtr msg){
    // Get the front 180 degree rays
    float front_angle = 0.34;
    int start_index = (-front_angle - msg->angle_min) / msg->angle_increment;
    int end_index   = ( front_angle - msg->angle_min) / msg->angle_increment;
    start_index = std::max(0, start_index);
    end_index = std::min((int)msg->ranges.size() - 1, end_index);

    bool is_blocked = false;
    // check front rays from -10 degree to 10 degree
    for (int i = start_index; i <= end_index; i++){
        if (!std::isfinite(msg->ranges[i]) 
            || msg->ranges[i] < msg->range_min 
            || msg->ranges[i] > msg->range_max) continue; // discard invalid scans

        if (msg->ranges[i] < 0.35) {
            is_blocked = true;
            break;
        }
    }

    // int side = (msg->ranges.size())/5;
    // float safe_side_dis = 0.2;
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
    // for (int i = msg->ranges.size()-side; i < (int)msg->ranges.size(); i++){
    //     if (!std::isfinite(msg->ranges[i]) 
    //         || msg->ranges[i] < msg->range_min 
    //         || msg->ranges[i] > msg->range_max) continue; // discard invalid scans

    //     if (msg->ranges[i] < safe_side_dis) {
    //         close_to_right = true;
    //         break;
    //     }
    // }

    if (is_blocked){
        float max_distance = 0.0;
        int max_index = -1;
        start_index = (-M_PI/2 - msg->angle_min) / msg->angle_increment;
        end_index   = ( M_PI/2 - msg->angle_min) / msg->angle_increment;
        start_index = std::max(0, start_index);
        end_index = std::min((int)msg->ranges.size() - 1, end_index);

        for (int i = start_index; i <= end_index; i++){
            if (!std::isfinite(msg->ranges[i]) 
                || msg->ranges[i] < msg->range_min 
                || msg->ranges[i] > msg->range_max) continue; // discard invalid scans

            if (msg->ranges[i] > max_distance) {
                max_distance = msg->ranges[i];
                max_index = i;
            }
        }
        // RCLCPP_INFO(this->get_logger(), "min range: %f", msg->range_min);
        // RCLCPP_INFO(this->get_logger(), "max range: %f", msg->range_max);
        // RCLCPP_INFO(this->get_logger(), "max idx: %d", max_index);

        auto new_direction = msg->angle_min + max_index * msg->angle_increment;
        direction_ = 3*new_direction;
    }/*else if (close_to_left){
        direction_ = -0.17;
    }else if (close_to_right){
        direction_ = 0.17;
    }*/else{
        direction_ = 0; // move forward
    }
};

void Patrol::timer_callback(){
    geometry_msgs::msg::Twist twist_msg;
    twist_msg.linear.x = 0.1; // always 0.1
    twist_msg.angular.z = direction_/2;
    twist_pub->publish(twist_msg);

    //RCLCPP_INFO(this->get_logger(), "direction: %f", direction_);
}
