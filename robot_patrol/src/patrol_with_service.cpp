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

    client_ = this->create_client<robot_patrol::srv::GetDirection>("/direction_service");
};


void Patrol::scan_callback(sensor_msgs::msg::LaserScan::ConstSharedPtr msg){
    
    float front_angle = 0.17;
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

    if (is_blocked){
        // call the direction service
        auto request = std::make_shared<robot_patrol::srv::GetDirection::Request>();
        request->laser_data = *msg;
      
        client_->async_send_request(
            request,
            std::bind(&Patrol::handle_response, this, std::placeholders::_1)
        );
        RCLCPP_INFO(this->get_logger(), "Service Request...");
        
    }else{
        angular_vel = 0;
    }
};

void Patrol::handle_response(
        rclcpp::Client<robot_patrol::srv::GetDirection>::SharedFuture future)
{
    auto response = future.get();
    if (response->direction == "Forward"){
        angular_vel = 0.0;
    }else if (response->direction == "Left"){
        angular_vel = 0.5;
    }else if (response->direction == "Right"){
        angular_vel = -0.5;
    }
    RCLCPP_INFO(this->get_logger(), "Service Response");
    RCLCPP_INFO(this->get_logger(), "Direction: %s", response->direction.c_str());
}

void Patrol::timer_callback(){
    geometry_msgs::msg::Twist twist_msg;
    twist_msg.linear.x = 0.1; // always 0.1
    twist_msg.angular.z = angular_vel;
    twist_pub->publish(twist_msg);

    //RCLCPP_INFO(this->get_logger(), "direction: %f", direction_);
}
