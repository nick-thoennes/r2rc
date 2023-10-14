/*
This node sends unsigned integer values (1000 to 2000) to the Arduino in order to
control the steering servo. In microseconds.
ndt
*/

#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <chrono>
#include <thread>
#include <unistd.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

#define USMIN 1000
#define USMAX 2000

using std::placeholders::_1;

int serialPort;

/*
The purpose of this function is to map a percentage of the steering range
available specified in the Arduino code "r2rc.ino".
*/
unsigned int percentage_microseconds_map(float servo_pos_percent){
        /* Determine right or left. 
        1000 corresponds to left, 
        1500 corresponds to straight, 
        2000 corresponds to right*/

        /* Each direction gets half of the total range. */
        unsigned int microsecond_dir_range= USMAX-USMIN / 2;

        /* In the case of no new input, default to steering straight. 
        Could change this to have a smooth return to center, like a car. Or 
        none at all. Not sure which would be better. */
        unsigned int microsecond_value= 1500;
        
        /* Right. Map to range 1000-1500. */
        if (servo_pos_percent > 0) {
            microsecond_value= ( (1 - (servo_pos_percent / 100)) * 500) + 1000;
            /* Here I do 1 - the percentage to get the opposite side of the 
            percentage rage I'm trying to get. This is because of the 100...-100
            scale of a joystick and the general number line. */
        }
        /* Left. Map to range 1500-2000. */
        else if (servo_pos_percent < 0) {
            servo_pos_percent= abs(servo_pos_percent);
            microsecond_value= ((servo_pos_percent / 100) * 500) + 1500;
        }
        /* If 0, then it goes straight. */
        else
            microsecond_value= 1500;

        /* Return the mapped value. */
        return microsecond_value;
    }

// Construct the ROS2 node
class SteeringNodeSubscriber : public rclcpp::Node{

    public:
    SteeringNodeSubscriber(): Node("steering_node"){
        subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&SteeringNodeSubscriber::servo_callback, this, _1));
    }

    private:
    void servo_callback(const geometry_msgs::msg::Twist & msg) const{
        /* I want msg.angular.z to be a percentage of how much I can steer the
        servo. For example, pushing the stick 50% to the right will send 50%
        of the servo's right range (which is (1500,2000] ) to the Arduino.
        However, for "negative percentages", the range will simply be 
        [1000, 1500) or left. */

        /* Retrieve the desired steering range percentage. */
        float servo_pos_percent= msg.angular.z;

        /* Map it to the actual range of the servo controller using linear interpolation. */
        unsigned int servo_pos_microseconds= percentage_microseconds_map(servo_pos_percent);
        
        /* Just log a message with the data for debugging purposes. */
        RCLCPP_INFO(this->get_logger(), "Servo Position: %d", servo_pos_microseconds);
        
        /* Build the serial message. */
        char formattedData[50]; /* Define a character array to hold the formatted string */
        std::sprintf(formattedData, "<%d, %d, %d, %d>", servo_pos_microseconds, servo_pos_microseconds, servo_pos_microseconds, servo_pos_microseconds);
        const char* data = formattedData; /* Assign the formatted string to the data pointer */
        int bytesWritten = write(serialPort, data, strlen(data)); /* Write the data. */
    
        /* In the case that the data was not written successfully, throw an error. */
        if (bytesWritten == -1) {
            RCLCPP_ERROR(this->get_logger(), "Error writing to serial port");
        }

    }
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
};

int main(int argc, char * argv[]) {
    // Arduino
    const char* portName = "/dev/arduino"; /* Udev rule to always have Arduino in same port. */
    serialPort = open(portName, O_WRONLY | O_NOCTTY);

    if (serialPort == -1) {
        // RCLCPP_ERROR(rclcpp::get_logger(), "Error opening serial port");
        return 1;
    }

    struct termios tty;
    memset(&tty, 0, sizeof(tty));

    if (tcgetattr(serialPort, &tty) != 0) {
        // RCLCPP_ERROR(rclcpp::get_logger(), "Error getting serial port attributes");
        close(serialPort);
        return 1;
    }

    cfsetospeed(&tty, B115200); // Set baud rate to match your Arduino configuration
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(serialPort, TCSANOW, &tty) != 0) {
        // RCLCPP_ERROR(rclcpp::get_logger(), "Error getting serial port attributes");
        close(serialPort);
        return 1;
    }

    // ROS
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SteeringNodeSubscriber>());
    rclcpp::shutdown();

    close(serialPort);
    // RCLCPP_INFO(rclcpp::get_logger(), "Data sent successfully!");
    return 0;
}