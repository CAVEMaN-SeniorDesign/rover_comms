#ifndef ROVER_COMM_HPP
#define ROVER_COMM_HPP

// ros2 helpers
#include "ament_index_cpp/get_package_share_directory.hpp" // for finding package path
#include "rclcpp/rclcpp.hpp"

// default msgs
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

// custom msgs
#include "rover_interfaces/msg/encoders.hpp"
#include "rover_interfaces/msg/airquality.hpp"
#include "rover_interfaces/msg/speakmovement.hpp"


// datatypes, mathematics, and algorithms
#include <cmath>
#include <algorithm>
#include <unordered_map>
#include <string>
#include <vector>

// utility
#include <csignal>
#include <unistd.h>  // For sleep()
#include <cstddef>
#include <functional>
#include <memory>
#include <fstream>
#include <iostream>
#include <cstdlib>   // for std::get_env for controller selection
#include <chrono>

// uh idk how to categorize
#include "cave_talk.h"
#include "rover_comms_serial.hpp"
#include "tinyxml2.h"

#define MAX_LINEAR_VEL  1.5
#define MAX_ANGULAR_VEL 1.0
#define MAX_AUTO_OMEGA 3.2
#define MAX_AUTO_V 0.5 // to be scaled up
#define AUTO_PORTION_OF_MAX 0.65 // multiplier to reduce max speed

/*Common ports that are assigned on default (we should implement automatic port find and selection):
    /dev/ttyTHS1 - Jetson Nano built in UART pins on pin 8 (TX) and pin 10 (RX)
    /dev/ttyUSB0 - This should usually be the default for a FTDI adapter but...
    /dev/ttyUSB1 - Sometimes it switches to this for some reason.
 */

struct CameraMovement
{
    static const int maxLength = 20;
    int length;
    int index                          = 0;
    double cam_pan_radians[maxLength]  = {0U};
    double cam_tilt_radians[maxLength] = {0U};
    double durations[maxLength]        = {0U};
};

struct CT_Sender_Movements
{
    static const int maxLength = 25;
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
        rclcpp::Publisher<rover_interfaces::msg::Airquality>::SharedPtr air_quality_read_pub_; // public to be accessed from callbacks
        rclcpp::Publisher<rover_interfaces::msg::Speakmovement>::SharedPtr speak_movement_pub_; // public to be accessed from callbacks
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_; // public to be accessed from callbacks
        std::string CaveTalk_ErrorToString(CaveTalk_Error_t error); // map to string outputs
	    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_raw_pub_; // public to be accessed from callbacks
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;

        bool looping       = true;
        bool waiting_booga = true;
        
//        vector<double> imu_samples; // 
        int imu_sample_cap = 5;

        bool manual_enable_ = true; // true if we are in manual mode
        bool auto_enable_   = true; // true if we are in auto mode
        bool CT_sender_enable_ = true; // true if we are sending cmds from xml sender
        bool clamp_move_cmds_ = true;
        bool speed_else_steer_clamp_ = true;

    private:
        void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg);
        void listen_callback();
        void speak_callback();
        void cam_move_callback();
        void ct_cmd_sender_callback();
        void gameControllerType();
        bool sendConfigs(std::string file);
        bool openAndSendConfigEncoder(std::string file);
        bool openAndSendConfigLog(std::string file);
        bool openAndSendConfigServoWheels(std::string file);
        bool openAndSendConfigServoCams(std::string file);
        bool openAndSendConfigMotor(std::string file);
        bool openAndSendConfigWheelPID(std::string file);
        bool openAndSendConfigSteerPID(std::string file);
        bool readOperatingModeConfig(std::string file);
        bool readCameraMovementConfig(std::string file);
        bool readCaveTalkSender(std::string file);
        bool checkXMLPositiveValue(std::string value);
        void calculateCamMovement(const sensor_msgs::msg::Joy::SharedPtr msg);
        void calculateMovement(const sensor_msgs::msg::Joy::SharedPtr msg);
        void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
        void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
        void calculateGoal();


        // sub for /cmd_vel_joy topics and publish to joystick topic
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr visual_odom_sub_;
        std::string this_pkg_path = ament_index_cpp::get_package_share_directory("rover_comms");

        // config files
        std::string cavetalk_config_ = this_pkg_path + "/configs/CaveTalk_Config.xml";
        std::string serial_config_   = this_pkg_path + "/configs/Serial_Config.xml";
        std::string operating_mode_config_ = this_pkg_path + "/configs/OperatingMode.xml";
        std::string camera_movement_config_ = this_pkg_path + "/configs/CameraMovement.xml";
        std::string cavetalk_sender_config_ = this_pkg_path + "/configs/CaveTalk_Sender.xml";

        // Params
        std::string game_controller_type_;
        double v_                   = 0;
        double omega_               = 0;
        double prev_v_              = 0;
        double prev_omega_          = 0;
        double v_auto_              = 0;
        double omega_auto_          = 0;
        rover_interfaces::msg::Speakmovement speak_movement_msg_ = rover_interfaces::msg::Speakmovement();
        nav_msgs::msg::Odometry::SharedPtr visual_odom_;
        geometry_msgs::msg::PoseStamped goal_;
        double max_wheel_speed_rps_ = 18.75;
        double v_overrider_ctsender_;
        double omega_overrider_ctsender_;

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
        bool mode_toggle_           = false;
        bool first_talk_            = true; // bool to assist syncing with MCU

        // This is here just so we can see all the available mappings.
        std::unordered_map<std::string, int> controller_mappings_ = {
            {"L_trigger", -1},
            {"R_trigger", -1},
            {"arm", -1}, // right most button
            {"lights", -1}, // up x
            {"mode", -1},
            {"L_shoulder", -1}, // Cam profile -
            {"R_shoulder", -1}, // Cam profile +
            {"L_joy_x", -1},
            {"L_joy_y", -1},
            {"R_joy_x", -1},
            {"R_joy_y", -1},
            {"D_x", -1},
            {"D_y", -1},
            {"D_up", -1},
            {"D_down", -1},
            {"D_left", -1},
            {"D_right", -1}
        };
        
        // CT Sender Movement Vars
        struct CT_Sender_Movements move_sequence_;

        // camera movement vars
        bool return_to_center_ = false;
        bool cmd_vel_inactive_ = true;
        double prev_cam_move_pan_;
        double prev_cam_move_tilt_;
        struct CameraMovement profiles_[5];
        int camera_movement_profile_length_ = 0;
        int camera_movement_profile_index_  = 0;

        // button sw-debouncing with a .5sec timeout
        rclcpp::Time last_speak_movement_ = this->get_clock()->now();
        rclcpp::Time last_lights_toggle_  = this->get_clock()->now();
        rclcpp::Time last_arm_toggle_     = this->get_clock()->now();
        rclcpp::Time last_mode_toggle_     = this->get_clock()->now();
        rclcpp::Time cam_move_last_move_time_ = this->get_clock()->now();
        rclcpp::Time CT_sender_last_move_time_ = this->get_clock()->now();
        rclcpp::Time cam_move_profile_button_ = this->get_clock()->now();
        rclcpp::Time last_set_goal_ = this->get_clock()->now();
        rclcpp::Time last_cmd_vel_ = this->get_clock()->now();
        double toggle_button_timeout_     = 0.5; // half-second time-out
        double cmd_vel_inactive_threshold_ = 5.0; // if asleep for 5 seconds, move camera around
};

#endif // ROVER_COMM_HPP
