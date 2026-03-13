#pragma once

#include "rclcpp/create_subscription.hpp"
#include "rclcpp/create_timer.hpp"
#include "rclcpp/publisher.hpp"
#include <functional>
#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "robot_patrol/direction_service.h"
#include "rclcpp/client.hpp"

class Patrol: public rclcpp::Node{
public:
    Patrol();

private:
    void scan_callback(sensor_msgs::msg::LaserScan::ConstSharedPtr msg);
    void timer_callback();
    void handle_response(
        rclcpp::Client<robot_patrol::srv::GetDirection>::SharedFuture future);

    // safest direction to move
    float direction_;

    float angular_vel = 0.0;

    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub;

    rclcpp::Client<robot_patrol::srv::GetDirection>::SharedPtr client_;
};