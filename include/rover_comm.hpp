#ifndef ROVER_COMM_HPP
#define ROVER_COMM_HPP

#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rover_interfaces/msg/serial.hpp"
#include "rover_interfaces/msg/odomplot.hpp"
#include <cmath>
#include <algorithm>

#include <csignal>
#include <unistd.h>  // For sleep()

#include <cstddef>
#include <functional>
#include <memory>
#include <vector>
#include <fstream>
#include <iostream>
#include <string>

#include "cave_talk.h"
#include "jetson_cave_talk.h"
#include "ring_buffer.h"
#include <termios.h>

#define MAX_LINEAR_VEL 1.0
#define MAX_ANGULAR_VEL 1.0

/*Common ports that are assigned on default (we should implement automatic port find and selection):
    /dev/ttyTHS1 - Jetson Nano built in UART pins on pin 8 (TX) and pin 10 (RX)
    /dev/ttyUSB0 - This should usually be the default for a FTDI adapter but...
    /dev/ttyUSB1 - Sometimes it switches to this for some reason.
*/

class RoverComm : public rclcpp::Node
{
public:
    RoverComm();
    ~RoverComm();
    std::shared_ptr<cave_talk::Talker> talker;
    std::shared_ptr<cave_talk::Listener> listener;
    rclcpp::TimerBase::SharedPtr speak_timer_;
    rclcpp::TimerBase::SharedPtr listen_timer_;
    rclcpp::Publisher<rover_interfaces::msg::Odomplot>::SharedPtr odom_read_pub_; // public to be accessed from callbacks
    std::string CaveTalk_ErrorToString(CaveTalk_Error_t error); // map to string outputs

    bool looping = true;
    bool waiting_booga = true;
private:
    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg);
    void listen_callback();
    void speak_callback();
    std::string gameControllerType();

    // sub for /cmd_vel_joy topics and publish to joystick topic
    rclcpp::Publisher<rover_interfaces::msg::Serial>::SharedPtr serial_read_pub_;

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

    // Params
    std::string game_controller_type_;
    double prev_v_;
    double prev_omega_;
    double prev_cam_pan_;
    double prev_cam_tilt_;
    bool lights_toggle_; 
    bool first_talk_; // bool to assist syncing with MCU
};

#endif // ROVER_COMM_HPP
