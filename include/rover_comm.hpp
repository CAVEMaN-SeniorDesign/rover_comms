#ifndef ROVER_COMM_HPP
#define ROVER_COMM_HPP

#include "ament_index_cpp/get_package_share_directory.hpp" // for finding package path

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include "rover_interfaces/msg/encoders.hpp"
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
#include "rover_comms_serial.hpp"
#include "tinyxml2.h"
#include <chrono>

#define MAX_LINEAR_VEL  1.0
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
    int index                          = 0;
    double cam_pan_radians[maxLength]  = {0U};
    double cam_tilt_radians[maxLength] = {0U};
    double durations[maxLength]        = {0U};
};

struct CT_Sender_Movements
{
    static const int maxLength = 20;
    int length = 0;
    int index                          = 0;
    double speed_mps[maxLength]  = {0U};
    double turn_rate_rps[maxLength] = {0U};
    double durations[maxLength]        = {0U};
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
        rclcpp::TimerBase::SharedPtr ct_sender_timer_;
        rclcpp::Publisher<rover_interfaces::msg::Encoders>::SharedPtr odom_read_pub_; // public to be accessed from callbacks
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_; // public to be accessed from callbacks
        std::string CaveTalk_ErrorToString(CaveTalk_Error_t error); // map to string outputs
        bool looping       = true;
        bool waiting_booga = true;

        bool manual_enable_ = true; // true if we are in manual mode
        bool auto_enable_   = true; // true if we are in auto mode
        bool CT_sender_enable_ = true; // true if we are sending cmds from xml sender

    private:
        void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg);
        void listen_callback();
        void speak_callback();
        void cam_move_callback();
        void ct_cmd_sender_callback();
        std::string gameControllerType();
        bool sendConfigs(std::string file);
        bool openAndSendConfigEncoder(std::string file);
        bool openAndSendConfigLog(std::string file);
        bool openAndSendConfigServoWheels(std::string file);
        bool openAndSendConfigServoCams(std::string file);
        bool openAndSendConfigMotor(std::string file);
        bool readOperatingModeConfig(std::string file);
        bool readCameraMovementConfig(std::string file);
        bool readCaveTalkSender(std::string file);
        bool checkXMLPositiveValue(std::string value);

        // sub for /cmd_vel_joy topics and publish to joystick topic
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
        std::string this_pkg_path = ament_index_cpp::get_package_share_directory("rover_comms");

        // config files
        std::string cavetalk_config_ = this_pkg_path + "/configs/CaveTalk_Config.xml";
        std::string serial_config_   = this_pkg_path + "/configs/Serial_Config.xml";
        std::string operating_mode_config_ = this_pkg_path + "/configs/OperatingMode.xml";
        std::string camera_movement_config_ = this_pkg_path + "/configs/CameraMovement.xml";
        std::string cavetalk_sender_config_ = this_pkg_path + "/configs/CaveTalk_Sender.xml";

        // Params
        std::string game_controller_type_;
        double prev_v_;
        double prev_omega_;
        double prev_cam_pan_        = 0;
        double prev_cam_tilt_       = 0;
        double cam_pan_             = 1.5707963;
        double cam_tilt_            = 1.5707963;
        double min_cam_pan_radian_  = 0;
        double max_cam_pan_radian_  = 6.2831853;
        double min_cam_tilt_radian_ = 0;
        double max_cam_tilt_radian_ = 6.2831853;
        bool cam_move_manual_enable_ = false;
        bool cam_profile_move_enable_ = true;
        bool lights_toggle_         = false;
        bool arm_toggle_            = false;
        bool first_talk_            = true; // bool to assist syncing with MCU

        // CT Sender Movement Vars
        struct CT_Sender_Movements move_sequence_;

        // camera movement vars
        struct CameraMovement profiles_[5];
        int camera_movement_profile_length_ = 0;
        int camera_movement_profile_index_  = 0;

        // button sw-debouncing with a .5sec timeout
        rclcpp::Time last_speak_movement_ = this->get_clock()->now();
        rclcpp::Time last_lights_toggle_  = this->get_clock()->now();
        rclcpp::Time last_arm_toggle_     = this->get_clock()->now();
        rclcpp::Time cam_move_last_move_time_ = this->get_clock()->now();
        rclcpp::Time CT_sender_last_move_time_ = this->get_clock()->now();
        rclcpp::Time cam_move_profile_button_ = this->get_clock()->now();
        double toggle_button_timeout_     = 0.5; // half-second time-out

};

#endif // ROVER_COMM_HPP