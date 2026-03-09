#include "rclcpp/create_subscription.hpp"
#include "rclcpp/create_timer.hpp"
#include "rclcpp/publisher.hpp"
#include <functional>
#include <rclcpp/rclcpp.hpp>
include "sensor_msgs/msg/laser_scan.hpp"
include "geometry_msgs/msg/twist.hpp"

class Patrol:Node{
    public:
    Patrol::Patrol():Node("Patrol_node"){
        twist_msg.linear.x = 0.1;
        twist_msg.angular.z = 0.0;
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
        timer_ = this->create_timer(timer_period, std::bind(&Patrol::timer_callback, this));
    };

    private:
    void scan_callback(sensor_msgs::msg::LaserScan::consPtr msg){
        float min_distance = std::numeric_limits<float>::infinity();
        int min_index;
        for (size_t i = 0; i < msg->ranges.size(); i++){
            if (msg->ranges[i] < range_min || msg->ranges[i] > range_max) continue; // discard invalid scans
            if (msg->ranges[i] < min_distance) {
                min_distance = msg->ranges[i];
                min_index = i;
            }
        }

        if (min_distance < 35){
            auto new_direction = msg->angle_min + min_index * msg->angle_increment;
            direction_ = (new_direction != direction_)?new_direction: direction_+0.1;
        }else{
            direction_ = 0;
        }
    };

    void timer_callback(){
        geometry_msgs::msg::Twist twist_msg;
        twist_msg.linear.x = 0.1;
        twist_msg.angular.z = direction_/2;
        twist_pub->publish(twist_msg);
    }

    // safest direction to move
    float direction_;

    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub;

}