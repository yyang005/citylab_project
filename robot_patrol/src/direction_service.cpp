

#include "robot_patrol/direction_service.h"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include <cmath>

using DirectionSrv = robot_patrol::srv::GetDirection;

DirectionService::DirectionService(std::string service_name):Node("Direction_service_node"){
    service_name_ = service_name;
    service_ = this->create_service<DirectionSrv>(
            service_name,
            std::bind(&DirectionService::get_direction_callback, this,
                      std::placeholders::_1, std::placeholders::_2));
    
    RCLCPP_INFO(this->get_logger(), "%s Service Server Ready...", service_name_.c_str());

}

void DirectionService::get_direction_callback(const std::shared_ptr<DirectionSrv::Request> request,
        std::shared_ptr<DirectionSrv::Response> response){
    RCLCPP_INFO(this->get_logger(), "%s Service Requested...", service_name_.c_str());

    sensor_msgs::msg::LaserScan scan = request->laser_data;

    float front_angle = M_PI / 6;
    int front_start = (-front_angle - scan.angle_min) / scan.angle_increment;
    int front_end   = ( front_angle - scan.angle_min) / scan.angle_increment;

    int right_start = (-M_PI/2 - scan.angle_min) / scan.angle_increment;
    int left_end = (M_PI/2 - scan.angle_min) / scan.angle_increment;

    // ----- compute sector sum -----
    auto sectorSum = [&](int start, int end)
    {
        float sum = 0.0;
        for (int i = start; i < end; i++)
        {
            if (std::isfinite(scan.ranges[i]) && scan.ranges[i] >= scan.range_min 
            && scan.ranges[i] <= scan.range_max)
                sum += scan.ranges[i];
        }
        return sum;
    };

    float total_dist_sec_right = sectorSum(right_start, front_start);
    float total_dist_sec_front = sectorSum(front_start, front_end);
    float total_dist_sec_left = sectorSum(front_end, left_end);

    if (total_dist_sec_front >= total_dist_sec_left 
        && total_dist_sec_front >= total_dist_sec_right){
        response->direction = "Forward";
    }else if (total_dist_sec_left > total_dist_sec_front 
        && total_dist_sec_left >= total_dist_sec_right){
        response->direction = "Left";
    }
    else if (total_dist_sec_right > total_dist_sec_front 
        && total_dist_sec_right >= total_dist_sec_left){
        response->direction = "Right";
    }else{
        response->direction = "Forward";
    }
    
    RCLCPP_INFO(this->get_logger(), "%s Service Completed", service_name_.c_str());
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DirectionService>("/direction_service");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}