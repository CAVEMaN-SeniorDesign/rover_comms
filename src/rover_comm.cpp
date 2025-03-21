#include "rover_comm.hpp"


RoverComm::RoverComm() : Node("rover_comm")
{
    game_controller_type_ = "xbox";
    lights_toggle_ = false;
    
    // Create joy subscription
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 10, std::bind(&RoverComm::joyCallback, this, std::placeholders::_1)); // "10" is queue-size for messages, how many to queue before dropping

    // Odom publisher
    odom_read_pub_ = this->create_publisher<rover_interfaces::msg::Odomplot>(
        "/odom_raw", 10);

    // Check for connected game controllers
    std::string type = this->gameControllerType();

    CaveTalk_Error_t init_error = cave_talk::init();

    while ((serial_port < 0) && (init_error != CAVE_TALK_ERROR_NONE) && looping){
        RCLCPP_INFO(this->get_logger(), "UART init start error, trying again...");
        init_error = cave_talk::init();
        sleep(1);
    }

    first_talk_ = true;

    RCLCPP_INFO(this->get_logger(), "rover_comms up on port ");

    speak_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&RoverComm::speak_callback, this)
    );
    
    listen_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&RoverComm::listen_callback, this)
    );
    
}

RoverComm::~RoverComm(){
    talker->SpeakOogaBooga(cave_talk::SAY_BOOGA); // close loop by sending BOOGA
    cave_talk::flush(); // flush buffer
}

std::string RoverComm::CaveTalk_ErrorToString(CaveTalk_Error_t error) {
    switch (error) {
        case CAVE_TALK_ERROR_NONE: return "CAVE_TALK_ERROR_NONE";
        case CAVE_TALK_ERROR_NULL: return "CAVE_TALK_ERROR_NULL";
        case CAVE_TALK_ERROR_SIZE: return "CAVE_TALK_ERROR_SIZE";
        case CAVE_TALK_ERROR_SOCKET_CLOSED: return "CAVE_TALK_ERROR_SOCKET_CLOSED";
        case CAVE_TALK_ERROR_INCOMPLETE: return "CAVE_TALK_ERROR_INCOMPLETE";
        case CAVE_TALK_ERROR_CRC: return "CAVE_TALK_ERROR_CRC";
        case CAVE_TALK_ERROR_VERSION: return "CAVE_TALK_ERROR_VERSION";
        case CAVE_TALK_ERROR_ID: return "CAVE_TALK_ERROR_ID";
        case CAVE_TALK_ERROR_PARSE: return "CAVE_TALK_ERROR_PARSE";
        default: return "UNKNOWN_ERROR";
    }
}

void RoverComm::listen_callback() {
    if (listener) {
        //RCLCPP_INFO(this->get_logger(), "Listening...");
        listener->Listen();
    }
    else{
        RCLCPP_INFO(this->get_logger(), "Waiting for listener to be passed...");
    }
}

void RoverComm::speak_callback(){
    if(talker && waiting_booga){ //if still waiting for booga, send more oogas
        RCLCPP_INFO(this->get_logger(), "Sending Ooga, awaiting Booga...");
        talker->SpeakOogaBooga(cave_talk::SAY_OOGA);
    }
    else if(!talker){
            RCLCPP_INFO(this->get_logger(), "Waiting for Speaker to be passed...");
    }
}


void RoverComm::joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    if(talker && !(waiting_booga)){
        static bool first_log = true; // static persists between calls
	    static bool first_log_cams = true;
        RCLCPP_INFO(this->get_logger(), "Speaking...");
        //talker->SpeakOogaBooga(cave_talk::SAY_OOGA);

        double v = 0;
        double omega = 0;
        double cam_pan = 0;
        double cam_tilt = 0;

        if(this->game_controller_type_=="xbox"){
            // invert the values
            double r_trig = -msg->axes[4] + 1; // Default unpressed is 1.0, down to -1 fully pressed
            double l_trig = -msg->axes[5] + 1; //
            omega = msg->axes[0]; // Angular velocity on joy 0
            cam_pan = ((msg->axes[2]*-1.0)+1)*3.1415926/2.0;
            cam_tilt = (msg->axes[3]+1)*3.1415926/2.0;
            v = (r_trig - l_trig) * (MAX_LINEAR_VEL / 2.0);//normalize to MAX_LINEAR_VEL
        }
        else{
            double l_joy = msg->axes[1];
            omega = msg->axes[2]; //powerA Steering with right joy
            v = (l_joy);
        }

        if(msg->buttons[4]){
            lights_toggle_ = !lights_toggle_; //toggle
            //MARK: add error checking
            CaveTalk_Error_t error_Lights = talker->SpeakLights(lights_toggle_);

            if (error_Lights != CAVE_TALK_ERROR_NONE){
                std::string error_Lights_str = CaveTalk_ErrorToString(error_Lights);
                RCLCPP_INFO(this->get_logger(), error_Lights_str);
            }
	        RCLCPP_INFO(this->get_logger(), "Lights toggled");
        }

        if (first_log || (v != prev_v_ || omega != prev_omega_)) {
            first_log = false;

            std::string command_vel_msg = "Linear_Vel: " + std::to_string(v) + ", Angular Vel: " + std::to_string(omega);
            //MARK: add error checking
            CaveTalk_Error_t error_Movement = talker->SpeakMovement(v, omega);
            
            if (error_Movement != CAVE_TALK_ERROR_NONE){
                std::string error_Movement_str = CaveTalk_ErrorToString(error_Movement);
                RCLCPP_INFO(this->get_logger(), error_Movement_str);
            }
            RCLCPP_INFO(this->get_logger(), command_vel_msg);

            prev_v_ = v;
            prev_omega_ = omega;
        }

        if (first_log_cams || (cam_pan != prev_cam_pan_ || cam_tilt != prev_cam_tilt_)) {
            first_log_cams = false;

            std::string cmd_cam_msg = "Cam_Pan: "  + std::to_string(cam_pan) + ", Cam_Tilt: " + std::to_string(cam_tilt);
            //MARK: add error checking
            talker->SpeakCameraMovement(cam_pan, cam_tilt);
            RCLCPP_INFO(this->get_logger(), cmd_cam_msg);

            prev_cam_pan_ = cam_pan;
            prev_cam_tilt_ = cam_tilt;
        }

        // std::string command_vel_msg = "Linear_Vel: " + std::to_string(v) + ", Angular Vel: " + std::to_string(omega);
        // talker->SpeakMovement(v, omega);
        // RCLCPP_INFO(this->get_logger(), command_vel_msg);
    }
    else if(!waiting_booga){
        RCLCPP_INFO(this->get_logger(), "Joy is awaiting talker pointer to be passed...");
    }
}

// RoverComm implementation
std::string RoverComm::gameControllerType(){
    std::ifstream file("/proc/bus/input/devices");
    std::string line;

    std::cout << "Checking connected game controllers...\n";

    // Looping through input device folders
    while (std::getline(file, line)) {
        // Search the line with "Name="
        if (line.find("Name=") != std::string::npos) {
            // If found line with "Controller" or "Gamepad" in it.
            bool xbox = line.find("Microsoft Xbox") != std::string::npos;
            bool powerA = line.find("PowerA NSW") != std::string::npos;
            if (xbox) {
                this->game_controller_type_ = "xbox";
                std::cout << game_controller_type_ << " controller detected" << std::endl;
                return std::string("xbox");
            }
            else if(powerA){
                this->game_controller_type_ = "powerA";
                std::cout << game_controller_type_ << " controller detected" << std::endl;
                return std::string("powerA"); //powerA has no analog triggers, will be default case
            }
        }
    }
    return "powerA"; //powerA has no analog triggers, will be default case
}