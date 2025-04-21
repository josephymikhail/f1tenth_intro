#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <chrono>
#include <cmath>

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include<fstream>
#include <string>
#include "nav_msgs/msg/odometry.hpp"
//#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

class LineFollowing : public rclcpp::Node
{
public:
     LineFollowing() : Node("line_following_node"), window(30), data(window, 0.0), prev_time(this->now())
    {
        // Create a subscriber to the "scan" topic for lidar data
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&LineFollowing::laser_scan_callback, this, std::placeholders::_1));
        
        // Create a publisher to send movement commands to the car
        publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);
    }

private:
    
    double prev_error = 0.0;
    double integral = 0.0;

    //assuming angle2 > angle1
    //must input value between -45 degrees and 225 (based on how i defined unit circle)
    //there are 1080 beams for 270 degrees, gives ratio of 4 beams per angle
    //0 degrees as pos x axis, front of car is 90 degrees, 
    //front is thus 90 - (-45) = 135 degrees * 4 beams = ranges[540]
    //say you want left, that would be 180 degrees, 180 - (-45) = 225 * 4 = ranges[900]
    //right would be 0 degrees, 0 - (-45) = 45 degrees * 4 beams = ranges[180]
    //0.4 as distance threshold
    double calculate_error (double angle1, double angle2, double desired_dist, const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        std::vector<float> ranges = msg->ranges;

        int index1 = (angle1 + 45) * 4;
        float a = ranges[index1];
        int index2 = (angle2 + 45) * 4;
        float b =  ranges[index2];
        float theta_deg = angle2 - angle1;
        float theta_rad = theta_deg * 0.0174533;

        float alpha = atan((a*cos(theta_rad) - b) / (a * sin(theta_rad)));
        float dist  = b * cos(alpha);
        float error = desired_dist - dist;
        return error;
    }


    double pid_control (double kp, double ki, double kd, double error, double integral, double prev_error, double timestep) {
        // pid output (steering angle)
        integral = integral + prev_error * timestep;
        double pid_output = (kp * error) + (ki * integral) + (kd * (error - prev_error) / timestep);
        return pid_output;
    }

    int index = 0;
    int window = 30;
    std::vector<double> data;
    //add in distance from right wall so that it can handle shape of map 
    void laser_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
     {

        float desired_distance = 0.85;
        float error = calculate_error(180, 110, desired_distance, msg);

        //low pass filter (moving average) ..not sure if this improves performance 
        /*  
        double average = 0.0;
        double sum = 0.0; 
        data[index] = error;
        sum = std::accumulate(data.begin(), data.end(), 0.0);
        average = sum / window;
        index = (index + 1) % window;
        */
        

        // PID controller
        double kp = 1.00;  
        double ki = 0.005; 
        double kd = 0.001;  

        //get timestep to input into PID
        rclcpp::Duration timestep = this->now() - prev_time;
        prev_time = this->now();

        //RCLCPP_INFO(this->get_logger(), "filtered = %.2f, unfiltered = %.2f", average, error); 
        double pid_output = pid_control(kp, ki, kd, error, integral, prev_error, timestep.seconds());
        prev_error = error; //not using low pass filter
        //prev_error = average; //for low pass filter

        //controlling car
        double speed = 0.5;
        if (pid_output >= 0 && pid_output <= 10) {
            speed = 1.5; 
        } else if (pid_output > 10 && pid_output <= 20) {
            speed = 1.0;
        }

        ackermann_msgs::msg::AckermannDriveStamped message;
        message.drive.steering_angle = -1 * pid_output;
        message.drive.speed = speed;
        publisher_->publish(message);
        
        //code for outputting data to file for signal visualization        
        std::ofstream output_file;
        output_file.open("/sim_ws/src/line_following/src/error.csv", std::ios::out);
        if (output_file.is_open()) {
            //output_file << average << "\n";
        }
        output_file.close();
     }

     rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
     rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_;
     rclcpp::Time prev_time;  
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LineFollowing>());
    rclcpp::shutdown();
    return 0;
}
