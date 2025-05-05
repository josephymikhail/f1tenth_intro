#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <chrono>
#include <cmath>
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include<fstream>
#include <string>
#include "nav_msgs/msg/odometry.hpp"

class LineFollowing : public rclcpp::Node
{
public:
     LineFollowing() : Node("line_following_node"), prev_time(this->now())
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
    double calc_dist(double angle1, double angle2, const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        std::vector<float> ranges = msg->ranges;
        int index1 = (angle1 + 45) * 4;
        float a = ranges[index1];
        int index2 = (angle2 + 45) * 4;
        float b =  ranges[index2];
        float theta_rad = (angle2 - angle1) * 0.0174533;

        float alpha = atan((a*cos(theta_rad) - b) / (a * sin(theta_rad)));
        float dist  = b * cos(alpha);
        float transform = 0.5 * sin(alpha); //try to fine tune 0.5 value...look at geometry/length of car
        dist = dist + transform;
        return dist;
    }


    double pid_control (double kp, double ki, double kd, double error, double integral, double prev_error, double timestep) {
        // pid output (steering angle)
        integral = integral + prev_error * timestep;
        double pid_output = (kp * error) + (ki * integral) + (kd * (error - prev_error) / timestep);

        return pid_output;
    }


    int count = 0; //for frequency of printing out stuff
    bool cove = false;

    void laser_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
     {
        std::vector<float> ranges = msg->ranges;
        float desired_distance = 0.85;
        float left = calc_dist(125,180, msg);
        float error = desired_distance - left;

        

        //PID parameters
        double kp = 2.0; //1.0 
        double ki = 0.005; //0.005
        double kd = 0.001; //0.001

        //get timestep to input into PID
        rclcpp::Duration timestep = this->now() - prev_time;
        prev_time = this->now();

        //PID controller
        double pid_output = pid_control(kp, ki, kd, error, integral, prev_error, timestep.seconds());
        prev_error = error;
        
        //speed of car dependent on steering angle (pid_output)
        double speed = 0.5;       
        if (pid_output >= 0 && pid_output <= 10) {
            speed = 1.5; 
        } else if (pid_output > 10 && pid_output <= 20) {
            speed = 1.0;
        }
        ackermann_msgs::msg::AckermannDriveStamped message;
        message.drive.steering_angle = -1 * pid_output;



        //if cove false then use pid
        //if detected, freeze pid controller (set cove = true) until error in normal straight range (0.02 bounds?)
        //0.79 and 0.82 for lookahead, error(left) between -0.58 and -0.528
        float front = ranges[540];
        float lookahead = calc_dist(135, 190, msg);
        if (lookahead > 0.79 && lookahead < 0.82 && error < -0.528 && error > -0.58) {
            cove = true;
            RCLCPP_INFO(this->get_logger(), "cove detected"); 
        }
        if (cove == true) {
            RCLCPP_INFO(this->get_logger(), "still in cove"); 
            message.drive.steering_angle = 0;
            if (abs(error) < 0.01 && front > 4) {
                cove = false;
                RCLCPP_INFO(this->get_logger(), "leaving cove"); 
            }
        }
        
        message.drive.speed = speed;
        publisher_->publish(message);
        
        //output for debugging
        if (count % 50 == 0) {
            RCLCPP_INFO(this->get_logger(), "left_error = %.2f, lookahead = %.2f", error, lookahead);    
        }
        count = count + 1;

        
        //code for outputting data to file for signal visualization        
        std::ofstream output_file;
        output_file.open("/sim_ws/src/line_following/src/error.csv", std::ios::out | std::ios::app);
        if (output_file.is_open()) {
            //output_file << error << "," << lookahead << "\n";
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