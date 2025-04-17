#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <chrono>
#include <cmath>

#include <string>
#include "nav_msgs/msg/odometry.hpp"
//#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

class LineFollowing : public rclcpp::Node
{
public:
     LineFollowing() : Node("line_following_node"), start_time_(this->now())
    {
        // Create a subscriber to the "scan" topic for lidar data
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&LineFollowing::laser_scan_callback, this, std::placeholders::_1));
        
        // Create a publisher to send movement commands to the car
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    }

private:
    
    double prev_error = 0.0;
    double integral = 0.0;

     void laser_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
     {
        auto elapsed_time = this->now() - start_time_;

        //must input value between -45 degrees and 225 (based on how i defined unit circle)
        //there are 1080 beams for 270 degrees, gives ratio of 4 beams per angle
        //0 degrees as pos x axis, front of car is 90 degrees, 
        //front is thus 90 - (-45) = 135 degrees * 4 beams = ranges[540]
        //say you want left, that would be 180 degrees, 180 - (-45) = 225 * 4 = ranges[900]
        //right would be 0 degrees, 0 - (-45) = 45 degrees * 4 beams = ranges[180]
        //0.4 as distance threshold
        std::vector<float> ranges = msg->ranges;
        //want to go counterclockwise first
        //ray is left (neg x axis)
        float a = ranges[900];
        
        //second ray is at 110 degrees with respect to robot thus (70 + 45) * 4 = 
        float theta = 1.91986; //110 degrees
        float b = ranges[(110 + 45) * 4];

        float alpha = atan((a*cos(theta) - b) / (a * sin(theta)));
        float dist  = b * cos(alpha);

        float desired_dist = 0.5;
        float error = desired_dist - dist;
        //RCLCPP_INFO(this->get_logger(), "error = %.2f", error); 



        // PID controller
        double kp = 0.4;  //
        double ki = 0.0;  // not working rn because of integral buildup
        double kd = 0.2;  //

        // proportional term
        double p_term = kp * error;
        

        // integral term
        integral += error;
        integral += error;
        if (std::abs(integral) > 1.0) {
            integral = 1.0 * (integral > 0 ? 1 : -1); // clamp integral term to avoid windup?
        }
        double i_term = ki * integral;


        // derivative term
        double d_term = kd * (error - prev_error);


        // pid output (steering angle)
        double pid_output = p_term + i_term + d_term;
        //RCLCPP_INFO(this->get_logger(), "steer = %.2f", pid_output); 

        // update the previous error
        prev_error = error;

        //controlling car
        double speed = 0.5; 
        if (pid_output >= 0 && pid_output <= 10) {
            speed = 1.5; 
        } else if (pid_output > 10 && pid_output <= 20) {
            speed = 1.0;
        }

        // output to terminal
        if (elapsed_time.seconds() > 1.5) {
            RCLCPP_INFO(this->get_logger(), "dist: %.2f, steering angle: %.2f", dist, pid_output);
            RCLCPP_INFO(this->get_logger(), "p: %.2f, i: %.2f, d: %.2f", p_term, i_term, d_term);
            
            start_time_ = this->now();
        }


        // Create and publish the Twist message to move the car

        auto message = geometry_msgs::msg::Twist();
        message.linear.x = speed;        //  speed based on the steering angle
        message.angular.z = pid_output * -1; // steering angle based on PID output
        publisher_->publish(message);
        


     }

     rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
     rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;   
     rclcpp::Time start_time_;  
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LineFollowing>());
    rclcpp::shutdown();
    return 0;
}
