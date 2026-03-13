#include <rclcpp/rclcpp.hpp>
#include "rclcpp/executors.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <functional>
#include <memory>

#include "robot_patrol/direction_service.h"
#include "rclcpp/client.hpp"

class testServiceNode: public rclcpp::Node{
public:
    testServiceNode():Node("test_service_node"){
        auto qos = rclcpp::QoS(10).reliability(rclcpp::ReliabilityPolicy::Reliable);

        scan_sub = this->create_subscription<sensor_msgs::msg::LaserScan>("/fastbot_1/scan", 
                    qos,
                    std::bind(&testServiceNode::scan_callback, this, std::placeholders::_1));
        
        client_ = this->create_client<robot_patrol::srv::GetDirection>("/direction_service");
        
        RCLCPP_INFO(this->get_logger(), "Service Client Ready...");

        // Wait for the service to be available (checks every second)
        while (!client_->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
        }
    };

private:
    void scan_callback(sensor_msgs::msg::LaserScan::ConstSharedPtr msg){

        float front_angle = 0.17;
        int front_start = (-front_angle - msg->angle_min) / msg->angle_increment;
        int front_end   = ( front_angle - msg->angle_min) / msg->angle_increment;
       
        // ----- compute sector minimums -----
        auto sectorMin = [&](int start, int end)
        {
            float m = msg->range_max;
            for (int i = start; i < end; i++)
            {
                if (std::isfinite(msg->ranges[i]) && msg->ranges[i] >= msg->range_min 
                && msg->ranges[i] <= msg->range_max)
                    m = std::min(m, msg->ranges[i]);
            }
            return m;
        };

        if (sectorMin(front_start, front_end) >= 0.35){
            return;
        }

        auto request = std::make_shared<robot_patrol::srv::GetDirection::Request>();
        request->laser_data = *msg;
      
        client_->async_send_request(
            request,
            std::bind(&testServiceNode::handle_response, this, std::placeholders::_1)
        );
        RCLCPP_INFO(this->get_logger(), "Service Request...");
    };

    void handle_response(
        rclcpp::Client<robot_patrol::srv::GetDirection>::SharedFuture future)
    {
        auto response = future.get();
        RCLCPP_INFO(this->get_logger(), "Service Response");
        RCLCPP_INFO(this->get_logger(), "Direction: %s", response->direction.c_str());
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub;
    rclcpp::Client<robot_patrol::srv::GetDirection>::SharedPtr client_;

};

int main(int argc, char** argv)
{
    // Initialize the ROS communication
    rclcpp::init(argc, argv);
    
    // Declare the node constructor
    auto test_client = std::make_shared<testServiceNode>();
    
    // Run the send_request() method
    rclcpp::spin(test_client);
    
    // Shutdown the ROS communication
    rclcpp::shutdown();
    return 0;
}