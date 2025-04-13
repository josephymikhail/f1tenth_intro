#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <chrono>

class LineFollowing : public rclcpp::Node
{
public:
     LineFollowing() : Node("line_following_node")
    {
        // Create a subscriber to the "scan" topic for lidar data
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&LineFollowing::laser_scan_callback, this, std::placeholders::_1));
        
        // Create a publisher to send movement commands to the car
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    }

private:
     void laser_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
     {
          std::vector<float> ranges = msg->ranges;
          float left = ranges[718];

          auto message = geometry_msgs::msg::Twist();

          if (left > 0.3) {
               message.linear.x = 0.2;
               message.angular.z = 0.2;
               RCLCPP_INFO(this->get_logger(), "Turning left: distance = %.2f", left);   
          } else {
               message.linear.x = 0.0;
               message.angular.z = 0.0;
          }
          publisher_->publish(message);
     }

     rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
     rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;     
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LineFollowing>());
    rclcpp::shutdown();
    return 0;
}