#pragma once

#include "rclcpp/create_subscription.hpp"
#include "rclcpp/create_timer.hpp"
#include "rclcpp/publisher.hpp"
#include <functional>
#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "robot_patrol/srv/get_direction.hpp"

using DirectionSrv = robot_patrol::srv::GetDirection;

class DirectionService : public rclcpp::Node{
public:
    DirectionService(std::string service_name);
private:
    rclcpp::Service<DirectionSrv>::SharedPtr service_;

    std::string service_name_;
    
    void get_direction_callback(
        const std::shared_ptr<DirectionSrv::Request> request,
        std::shared_ptr<DirectionSrv::Response> response);
};