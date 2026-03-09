#pragma once

#include "rclcpp/create_subscription.hpp"
#include "rclcpp/create_timer.hpp"
#include "rclcpp/publisher.hpp"
#include <functional>
#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"

class Patrol: public rclcpp::Node{
public:
    Patrol();

private:
    void scan_callback(sensor_msgs::msg::LaserScan::ConstSharedPtr msg);
    void timer_callback();

    // safest direction to move
    float direction_;

    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub;
};