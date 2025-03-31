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
#include <chrono>

#include "cave_talk.h"
#include "new_serial.hpp"
#include "tinyxml2.h"
#include "jetson_cave_talk.h"


#define MAX_LINEAR_VEL 1.0
#define MAX_ANGULAR_VEL 1.0

/*Common ports that are assigned on default (we should implement automatic port find and selection):
    /dev/ttyTHS1 - Jetson Nano built in UART pins on pin 8 (TX) and pin 10 (RX)
    /dev/ttyUSB0 - This should usually be the default for a FTDI adapter but...
    /dev/ttyUSB1 - Sometimes it switches to this for some reason.
*/

struct CameraMovement
{
    static const int maxLength = 10;
    int length;
    int index = 0;
    double cam_pan_radians[maxLength] = {0U};
    double cam_tilt_radians[maxLength] = {0U};
    double durations[maxLength] = {0U};
};

class RoverComm : public rclcpp::Node
{
public:
    RoverComm();
    ~RoverComm();
    std::shared_ptr<cave_talk::Talker> talker;
    std::shared_ptr<cave_talk::Listener> listener;
    rclcpp::TimerBase::SharedPtr speak_timer_;
    rclcpp::TimerBase::SharedPtr listen_timer_;
    rclcpp::TimerBase::SharedPtr cam_move_timer_;
    rclcpp::Publisher<rover_interfaces::msg::Odomplot>::SharedPtr odom_read_pub_; // public to be accessed from callbacks
    std::string CaveTalk_ErrorToString(CaveTalk_Error_t error); // map to string outputs

    bool looping = true;
    bool waiting_booga = true;
private:
    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg);
    void listen_callback();
    void speak_callback();
    void cam_move_callback();
    std::string gameControllerType();
    bool sendConfigs(std::string file);
    bool openAndSendConfigEncoder(std::string file);
    bool openAndSendConfigLog(std::string file);
    bool openAndSendConfigServoWheels(std::string file);
    bool openAndSendConfigServoCams(std::string file);
    bool openAndSendConfigMotor(std::string file);
    bool readCameraMovementConfig(std::string file);
    bool sendCameraMovement();

    // sub for /cmd_vel_joy topics and publish to joystick topic
    rclcpp::Publisher<rover_interfaces::msg::Serial>::SharedPtr serial_read_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

    // Params
    std::string game_controller_type_;
    double prev_v_;
    double prev_omega_;
    double prev_cam_pan_ = 0;
    double prev_cam_tilt_ = 0;
    double cam_pan_ = 1.5707963;
    double cam_tilt_ = 1.5707963;
    double min_cam_pan_radian_ = 0;
    double max_cam_pan_radian_ = 6.2831853;
    double min_cam_tilt_radian_ = 0;
    double max_cam_tilt_radian_ = 6.2831853;
    bool lights_toggle_; 
    bool arm_toggle_;
    bool first_talk_; // bool to assist syncing with MCU

    // camera movement vars
    struct CameraMovement profiles[5];
    int camera_movement_profile_length_ = 0;
    int camera_movement_profile_index_ = 0;
    std::chrono::time_point<std::chrono::steady_clock> cam_move_last_move_time_;

    // button sw-debouncing with a .5sec timeout
    rclcpp::Time last_lights_toggle_ = this->get_clock()->now();
    rclcpp::Time last_arm_toggle_ = this->get_clock()->now();
    double toggle_button_timeout_ = 0.5; // half-second time-out

    // // serial params, can be modified in rover_comms/src/Serial_Config.xml
    // std:string serial_port_ = "/dev/ttyTHS1";
    // unsigned long serialBaud_ = 1000000;
    // serial::Timeout timeout = serial::Timeout::simpleTimeout(0); // non-blocking
};

#endif // ROVER_COMM_HPP