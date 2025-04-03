#include "rover_comm.hpp"


RoverComm::RoverComm() : Node("rover_comm")
{
    game_controller_type_ = "xbox";

    // Create joy subscription
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 10, std::bind(&RoverComm::joyCallback, this, std::placeholders::_1));

    // Odom publisher
    odom_read_pub_ = this->create_publisher<rover_interfaces::msg::Encoders>(
        "/odom_raw", 10);

    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(
        "/imu_data", 10);

    // Check for connected game controllers
    std::string type = this->gameControllerType();

    speak_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&RoverComm::speak_callback, this)
    );
    
    listen_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&RoverComm::listen_callback, this)
    );

    cam_move_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(20),
        std::bind(&RoverComm::cam_move_callback, this)
    );
    
}

RoverComm::~RoverComm(){
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
        CaveTalk_Error_t error = listener->Listen();
        if (CAVE_TALK_ERROR_NONE != error)
        {
            RCLCPP_INFO(this->get_logger(), "Listener error %s", CaveTalk_ErrorToString(error).c_str());
        }
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

void RoverComm::cam_move_callback(){
    double time_elapsed = (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - cam_move_last_move_time_)).count() * .001;
    int posIdx = profiles[camera_movement_profile_index_].index;
    int posDuration = profiles[camera_movement_profile_index_].durations[posIdx];

    if (time_elapsed >= posDuration){
        posIdx++;
        if(posIdx >= profiles[camera_movement_profile_index_].length){
            posIdx = 0;
        }
        profiles[camera_movement_profile_index_].index = posIdx;

        cam_move_last_move_time_ = std::chrono::steady_clock::now();
        double new_pan = profiles[camera_movement_profile_index_].cam_pan_radians[posIdx];
        double new_tilt = profiles[camera_movement_profile_index_].cam_tilt_radians[posIdx];
        talker->SpeakCameraMovement(new_pan, new_tilt);
    }
    else{
        RCLCPP_INFO(this->get_logger(), "Waiting for camera movement to finish...");
    }
}


void RoverComm::joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    if(first_talk_){
        bool success = sendConfigs("/root/ros2_ws/src/rover_comms/src/CaveTalk_Config.xml");
        if(success){
            RCLCPP_INFO(this->get_logger(), "Open & Send Config Status: True");
        }else{
            RCLCPP_INFO(this->get_logger(), "Open & Send Config Status: False");
        }
        first_talk_ = false;
    }

    if(talker && !(waiting_booga)){
        static bool first_log = true; // static persists between calls
	    static bool first_log_cams = true;

        double v = 0;
        double omega = 0;

        if(this->game_controller_type_=="xbox"){
            // invert the values
            double r_trig = -msg->axes[4] + 1; // Default unpressed is 1.0, down to -1 fully pressed
            double l_trig = -msg->axes[5] + 1; //
            omega = msg->axes[0]; // Angular velocity on joy 0

            //left-rght is msg->axes[6], left is +
            //up-down is msg->axes[7], up is +
            cam_pan_ += ((msg->axes[6])*3.1415926/32.0);
            cam_tilt_ += ((msg->axes[7])*3.1415926/32.0);

            if(cam_pan_ < min_cam_pan_radian_){
                cam_pan_ = min_cam_pan_radian_;
            }
            else if(cam_pan_ > max_cam_pan_radian_){
                cam_pan_ = max_cam_pan_radian_;
            }

            if(cam_tilt_ < min_cam_tilt_radian_){
                cam_tilt_ = min_cam_tilt_radian_;
            }
            else if(cam_tilt_ > max_cam_tilt_radian_){
                cam_tilt_ = max_cam_tilt_radian_;
            }
            v = (r_trig - l_trig) * (MAX_LINEAR_VEL / 2.0);//normalize to MAX_LINEAR_VEL
        }
        else{
            double l_joy = msg->axes[1];
            omega = msg->axes[2]; //powerA Steering with right joy
            v = (l_joy);
        }

        if(msg->buttons[4] && ((this->get_clock()->now() - last_lights_toggle_).seconds() > toggle_button_timeout_)){
            lights_toggle_ = !lights_toggle_; //toggle
            //MARK: add error checking
            CaveTalk_Error_t error_Lights = talker->SpeakLights(lights_toggle_);

            if (error_Lights != CAVE_TALK_ERROR_NONE){
                std::string error_Lights_str = CaveTalk_ErrorToString(error_Lights);
                RCLCPP_INFO(this->get_logger(), error_Lights_str.c_str());
            }
	        RCLCPP_INFO(this->get_logger(), "Lights toggled");
            last_lights_toggle_ = this->get_clock()->now();
        }

        if(msg->buttons[1] && ((this->get_clock()->now() - last_arm_toggle_).seconds() > toggle_button_timeout_)){
            arm_toggle_ = !arm_toggle_; //toggle
            //MARK: add error checking
            CaveTalk_Error_t error_arm = talker->SpeakArm(arm_toggle_);

            if (error_arm != CAVE_TALK_ERROR_NONE){
                std::string error_arm_str = CaveTalk_ErrorToString(error_arm);
                RCLCPP_INFO(this->get_logger(), error_arm_str.c_str());
            }

            if (arm_toggle_){
                RCLCPP_INFO(this->get_logger(), "Rover ARMED");
            }
            else{
                RCLCPP_INFO(this->get_logger(), "Rover DISARMED");
            }

            last_arm_toggle_ = this->get_clock()->now();
        }
        

        if ((first_log || (v != prev_v_ || omega != prev_omega_)) || (this->get_clock()->now() - last_speak_movement_).seconds() > 0.75) {
            first_log = false;

            std::string command_vel_msg = "Linear_Vel: " + std::to_string(v) + ", Angular Vel: " + std::to_string(omega);
            //MARK: add error checking
            CaveTalk_Error_t error_Movement = talker->SpeakMovement(v, omega);
            last_speak_movement_ = this->get_clock()->now();
            
            if (error_Movement != CAVE_TALK_ERROR_NONE){
                std::string error_Movement_str = CaveTalk_ErrorToString(error_Movement);
                RCLCPP_INFO(this->get_logger(), error_Movement_str.c_str());
            }
            RCLCPP_INFO(this->get_logger(), command_vel_msg.c_str());

            prev_v_ = v;
            prev_omega_ = omega;
        }

        if (first_log_cams || (cam_pan_ != prev_cam_pan_ || cam_tilt_ != prev_cam_tilt_)) {
            first_log_cams = false;

            std::string cmd_cam_msg = "Cam_Pan: "  + std::to_string(cam_pan_) + ", Cam_Tilt: " + std::to_string(cam_tilt_);
            //MARK: add error checking
            talker->SpeakCameraMovement(cam_pan_, cam_tilt_);
            RCLCPP_INFO(this->get_logger(), cmd_cam_msg.c_str());

            prev_cam_pan_ = cam_pan_;
            prev_cam_tilt_ = cam_tilt_;
        }

        // std::string command_vel_msg = "Linear_Vel: " + std::to_string(v) + ", Angular Vel: " + std::to_string(omega);
        // talker->SpeakMovement(v, omega);
        // RCLCPP_INFO(this->get_logger(), command_vel_msg);
    }
    else if (!waiting_booga)
    {
        RCLCPP_INFO(this->get_logger(), "Joy is awaiting talker pointer to be passed...");
    }
}

// RoverComm implementation
std::string RoverComm::gameControllerType()
{
    std::ifstream file("/proc/bus/input/devices");
    std::string   line;

    std::cout << "Checking connected game controllers...\n";

    // Looping through input device folders
    while (std::getline(file, line))
    {
        // Search the line with "Name="
        if (line.find("Name=") != std::string::npos)
        {
            // If found line with "Controller" or "Gamepad" in it.
            bool xbox   = line.find("Microsoft Xbox") != std::string::npos;
            bool powerA = line.find("PowerA NSW") != std::string::npos;
            if (xbox)
            {
                this->game_controller_type_ = "xbox";
                std::cout << game_controller_type_ << " controller detected" << std::endl;
                return std::string("xbox");
            }
            else if (powerA)
            {
                this->game_controller_type_ = "powerA";
                std::cout << game_controller_type_ << " controller detected" << std::endl;
                return std::string("powerA"); //powerA has no analog triggers, will be default case
            }
        }
    }
    return "powerA"; //powerA has no analog triggers, will be default case
}

// Open and send XML config file to MCU
bool RoverComm::sendConfigs(std::string file)
{

    bool success = true;

    success &= openAndSendConfigEncoder(file);
    success &= openAndSendConfigLog(file);
    success &= openAndSendConfigServoWheels(file);
    success &= openAndSendConfigServoCams(file);
    success &= openAndSendConfigMotor(file);

    return success;
}

bool RoverComm::openAndSendConfigEncoder(std::string file)
{

    tinyxml2::XMLDocument doc;
    tinyxml2::XMLError    error = doc.LoadFile(file.c_str());
    // std::cout << error << std::endl;
    if (error != tinyxml2::XML_SUCCESS)
    {
        RCLCPP_INFO(this->get_logger(), "Error opening file: %s", file.c_str());
        // std::cout << "Open File Error" << std::endl;

        return false;
    }

    cave_talk::ConfigEncoder encoders[4];

    double smoothing_factor;
    double radians_per_pulse;
    double pulses_per_period;
    // cave_talk::EncoderMode mode;
    int mode;

    tinyxml2::XMLElement *root = doc.FirstChildElement();
    if (root == nullptr)
    {
        RCLCPP_INFO(this->get_logger(), "Error finding root, good luck");
        // std::cout << "Error parsing file: COuldn't find root, you're fucked" << std::endl;
        return false;
    }

    tinyxml2::XMLElement *config_encoder = root->FirstChildElement("ConfigEncoder");
    if (config_encoder == nullptr)
    {
        RCLCPP_INFO(this->get_logger(), "Error finding ConfigEncoder");
        // std::cout << "Error parsing file: Couldn't find ConfigEncoder" << std::endl;
        return false;
    }

    for (int i = 0; i < 4; i++)
    {
        encoders[i] = cave_talk::ConfigEncoder();

        std::string encoder_wheel_name = "Encoder_Wheel_" + std::to_string(i);

        tinyxml2::XMLElement *encoder_wheel = config_encoder->FirstChildElement(encoder_wheel_name.c_str());
        if (encoder_wheel == nullptr)
        {
            RCLCPP_INFO(this->get_logger(), "Error parsing file: Couldn't find %s", encoder_wheel_name.c_str());
            // std::cout << "Error parsing file: Couldnt find " << encoder_wheel_name << std::endl;
            return false;
        }

        int extractResult = -1;

        tinyxml2::XMLElement *smoothing_factor_node = encoder_wheel->FirstChildElement("smoothing_factor");
        if (smoothing_factor_node != nullptr)
        {
            extractResult = smoothing_factor_node->QueryDoubleText(&smoothing_factor);

            if (extractResult == 0)
            {
                encoders[i].set_smoothing_factor(smoothing_factor);
                // encoders[i].smoothing_factor = smoothing_factor;
                RCLCPP_INFO(this->get_logger(), "Smoothing factor: %f", smoothing_factor);
                // std::cout << smoothing_factor << std::endl;
            }
            else
            {
                std::cout << "Failed to extract smooth" << std::endl;
            }

        }
        else
        {
            encoders[i].set_smoothing_factor(-1);
            // encoders[i].smoothing_factor = -1;
        }

        tinyxml2::XMLElement *radians_per_pulse_node = encoder_wheel->FirstChildElement("radians_per_pulse");
        if (radians_per_pulse_node != nullptr)
        {
            extractResult = radians_per_pulse_node->QueryDoubleText(&radians_per_pulse);

            if (extractResult == 0)
            {
                encoders[i].set_radians_per_pulse(radians_per_pulse);
                // encoders[i].radians_per_pulse = radians_per_pulse;
                RCLCPP_INFO(this->get_logger(), "Radians per Pulse: %f", radians_per_pulse);
                // std::cout << radians_per_pulse << std::endl;
            }
            else
            {
                std::cout << "Failed to extract rads per pulse" << std::endl;
            }
        }
        else
        {
            encoders[i].set_radians_per_pulse(-1);
            // encoders[i].radians_per_pulse = -1;
        }

        tinyxml2::XMLElement *pulses_per_period_node = encoder_wheel->FirstChildElement("pulses_per_period");
        if (pulses_per_period_node != nullptr)
        {
            extractResult = pulses_per_period_node->QueryDoubleText(&pulses_per_period);

            if (extractResult == 0)
            {
                encoders[i].set_pulses_per_period(pulses_per_period);
                // encoders[i].pulses_per_period = pulses_per_period;
                RCLCPP_INFO(this->get_logger(), "Pulses per Period: %f", pulses_per_period);
                // std::cout << pulses_per_period << std::endl;
            }
            else
            {
                std::cout << "Failed to extract pulse per pd" << std::endl;
            }
        }
        else
        {
            encoders[i].set_pulses_per_period(-1);
            // encoders[i].pulses_per_period = -1;
        }

        tinyxml2::XMLElement *mode_node = encoder_wheel->FirstChildElement("mode");
        if (mode_node != nullptr)
        {
            extractResult = mode_node->QueryIntText(&mode);
            if (extractResult == 0)
            {
                encoders[i].set_mode(static_cast<cave_talk::EncoderMode>(mode));
                // encoders[i].mode = static_cast<cave_talk_EncoderMode>(mode);
                RCLCPP_INFO(this->get_logger(), "Mode: %d", mode);
                // std::cout << mode << std::endl;
            }
            else
            {
                std::cout << "Failed to extract mode" << std::endl;
            }
        }
        else
        {
            encoders[i].set_mode(cave_talk::EncoderMode::BSP_ENCODER_USER_MODE_PULSES_PER_ROTATON);
            // encoders[i].mode = cave_talk_EncoderMode::cave_talk_EncoderMode_BSP_ENCODER_USER_MODE_PULSES_PER_ROTATON;
        }

    }

    // roverMouth.SpeakConfigEncoder(encoders[0], encoders[1], encoders[2], encoders[3]));
    talker->SpeakConfigEncoder(encoders[0], encoders[1], encoders[2], encoders[3]);
    return true;

}

bool RoverComm::openAndSendConfigLog(std::string file)
{

    tinyxml2::XMLDocument doc;
    tinyxml2::XMLError    error = doc.LoadFile(file.c_str());
    if (error != tinyxml2::XML_SUCCESS)
    {
        RCLCPP_INFO(this->get_logger(), "Error opening file: %s", file.c_str());
        // std::cout << "Open File Error" << std::endl;
        return false;
    }

    cave_talk::LogLevel log_level;
    int                 log_level_int;

    tinyxml2::XMLElement *root_xml = doc.FirstChildElement();
    if (root_xml == nullptr)
    {
        RCLCPP_INFO(this->get_logger(), "Error finding root, good luck");
        // std::cout << "Error parsing file: COuldn't find root, you're fucked" << std::endl;
        return false;
    }

    tinyxml2::XMLElement *config_log_xml = root_xml->FirstChildElement("ConfigLog");
    if (config_log_xml == nullptr)
    {
        RCLCPP_INFO(this->get_logger(), "Error finding ConfigLog");
        // std::cout << "Error parsing file: Couldn't find ConfigEncoder" << std::endl;
        return false;
    }

    int extractResult = -1;

    tinyxml2::XMLElement *log_level_xml = config_log_xml->FirstChildElement("log_level");
    if (log_level_xml != nullptr)
    {
        extractResult = log_level_xml->QueryIntText(&log_level_int);

        if (extractResult == 0)
        {
            log_level = static_cast<cave_talk::LogLevel>(log_level_int);
            RCLCPP_INFO(this->get_logger(), "Log Level: %d", log_level_int);
            // std::cout << log_level_int << std::endl;
        }
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Error finding log_level");
        // std::cout << "Error parsing file: Couldn't find log_level" << std::endl;
        return false;
    }

    talker->SpeakConfigLog(log_level);
    return true;

}

bool RoverComm::openAndSendConfigServoWheels(std::string file)
{
    tinyxml2::XMLDocument doc;
    tinyxml2::XMLError    error = doc.LoadFile(file.c_str());
    // std::cout << error << std::endl;
    if (error != tinyxml2::XML_SUCCESS)
    {
        RCLCPP_INFO(this->get_logger(), "Error opening file: %s", file.c_str());
        // std::cout << "Open File Error" << std::endl;
        return false;
    }

    cave_talk::Servo servo_wheels[4];
    double           min_angle_radian;
    double           max_angle_radian;
    double           center_angle_radian;
    double           min_duty_cycle_microsec;
    double           max_duty_cycle_microsec;
    double           center_duty_cycle_microsec;

    tinyxml2::XMLElement *root = doc.FirstChildElement();
    if (root == nullptr)
    {
        RCLCPP_INFO(this->get_logger(), "Error finding root, good luck");
        // std::cout << "Error parsing file: COuldn't find root, you're fucked" << std::endl;
        return false;
    }

    tinyxml2::XMLElement *config_servo_wheels = root->FirstChildElement("ConfigServoWheels");
    if (config_servo_wheels == nullptr)
    {
        RCLCPP_INFO(this->get_logger(), "Error finding ConfigServoWheels");
        // std::cout << "Error parsing file: Couldn't find ConfigServoWheels" << std::endl;
        return false;
    }

    for (int i = 0; i < 4; i++)
    {
        servo_wheels[i] = cave_talk::Servo();

        std::string servo_wheel_name = "Servo_Wheel_" + std::to_string(i);

        tinyxml2::XMLElement *servo_wheel = config_servo_wheels->FirstChildElement(servo_wheel_name.c_str());
        if (servo_wheel == nullptr)
        {
            RCLCPP_INFO(this->get_logger(), "Error parsing file: Couldn't find %s", servo_wheel_name.c_str());
            // std::cout << "Error parsing file: Couldnt find " << servo_wheel_name << std::endl;
            return false;
        }

        int extractResult = -1;

        tinyxml2::XMLElement *min_angle_radian_node = servo_wheel->FirstChildElement("min_angle_radian");
        if (min_angle_radian_node != nullptr)
        {
            extractResult = min_angle_radian_node->QueryDoubleText(&min_angle_radian);

            if (extractResult == 0)
            {
                servo_wheels[i].set_min_angle_radian(min_angle_radian);
                RCLCPP_INFO(this->get_logger(), "min Angle Radian: %f", min_angle_radian);
                // std::cout << min_angle_radian << std::endl;
            }
            else
            {
                std::cout << "Failed to extract min angle radian" << std::endl;
            }
        }
        else
        {
            servo_wheels[i].set_min_angle_radian(-1);
        }

        tinyxml2::XMLElement *max_angle_radian_node = servo_wheel->FirstChildElement("max_angle_radian");
        if (max_angle_radian_node != nullptr)
        {
            extractResult = max_angle_radian_node->QueryDoubleText(&max_angle_radian);

            if (extractResult == 0)
            {
                servo_wheels[i].set_max_angle_radian(max_angle_radian);
                RCLCPP_INFO(this->get_logger(), "max Angle Radian: %f", max_angle_radian);
                // std::cout << max_angle_radian << std::endl;
            }
            else
            {
                std::cout << "Failed to extract max angle radian" << std::endl;
            }
        }
        else
        {
            servo_wheels[i].set_max_angle_radian(-1);
        }

        tinyxml2::XMLElement *center_angle_radian_node = servo_wheel->FirstChildElement("center_angle_radian");
        if (center_angle_radian_node != nullptr)
        {
            extractResult = center_angle_radian_node->QueryDoubleText(&center_angle_radian);

            if (extractResult == 0)
            {
                servo_wheels[i].set_center_angle_radian(center_angle_radian);
                RCLCPP_INFO(this->get_logger(), "center Angle Radian: %f", center_angle_radian);
                // std::cout << center_angle_radian << std::endl;
            }
            else
            {
                std::cout << "Failed to extract center angle radian" << std::endl;
            }
        }
        else
        {
            servo_wheels[i].set_center_angle_radian(-1);
        }

        tinyxml2::XMLElement *min_duty_cycle_microsec_node = servo_wheel->FirstChildElement("min_duty_cycle_microsec");
        if (min_duty_cycle_microsec_node != nullptr)
        {
            extractResult = min_duty_cycle_microsec_node->QueryDoubleText(&min_duty_cycle_microsec);

            if (extractResult == 0)
            {
                servo_wheels[i].set_min_duty_cycle_percentage(min_duty_cycle_microsec);
                RCLCPP_INFO(this->get_logger(), "Min Angle Radian: %f", min_duty_cycle_microsec);
                // std::cout << min_duty_cycle_microsec << std::endl;
            }
            else
            {
                std::cout << "Failed to extract min angle radian" << std::endl;
            }
        }
        else
        {
            servo_wheels[i].set_min_duty_cycle_percentage(-1);
        }

        tinyxml2::XMLElement *max_duty_cycle_microsec_node = servo_wheel->FirstChildElement("max_duty_cycle_microsec");
        if (max_duty_cycle_microsec_node != nullptr)
        {
            extractResult = max_duty_cycle_microsec_node->QueryDoubleText(&max_duty_cycle_microsec);

            if (extractResult == 0)
            {
                servo_wheels[i].set_max_duty_cycle_percentage(max_duty_cycle_microsec);
                RCLCPP_INFO(this->get_logger(), "max Angle Radian: %f", max_duty_cycle_microsec);
                // std::cout << max_duty_cycle_microsec << std::endl;
            }
            else
            {
                std::cout << "Failed to extract max angle radian" << std::endl;
            }
        }
        else
        {
            servo_wheels[i].set_max_duty_cycle_percentage(-1);
        }

        tinyxml2::XMLElement *center_duty_cycle_microsec_node = servo_wheel->FirstChildElement("center_duty_cycle_microsec");
        if (center_duty_cycle_microsec_node != nullptr)
        {
            extractResult = center_duty_cycle_microsec_node->QueryDoubleText(&center_duty_cycle_microsec);

            if (extractResult == 0)
            {
                servo_wheels[i].set_center_duty_cycle_percentage(center_duty_cycle_microsec);
                RCLCPP_INFO(this->get_logger(), "center Angle Radian: %f", center_duty_cycle_microsec);
                // std::cout << center_duty_cycle_microsec << std::endl;
            }
            else
            {
                std::cout << "Failed to extract center angle radian" << std::endl;
            }
        }
        else
        {
            servo_wheels[i].set_center_duty_cycle_percentage(-1);
        }
    }

    talker->SpeakConfigServoWheels(servo_wheels[0], servo_wheels[1], servo_wheels[2], servo_wheels[3]);
    return true;
}

bool RoverComm::openAndSendConfigServoCams(std::string file)
{
    tinyxml2::XMLDocument doc;
    tinyxml2::XMLError    error = doc.LoadFile(file.c_str());
    // std::cout << error << std::endl;
    if (error != tinyxml2::XML_SUCCESS)
    {
        RCLCPP_INFO(this->get_logger(), "Error opening file: %s", file.c_str());
        // std::cout << "Open File Error" << std::endl;
        return false;
    }

    cave_talk::Servo servo_cams[2];
    double           min_angle_radian;
    double           max_angle_radian;
    double           center_angle_radian;
    double           min_duty_cycle_microsec;
    double           max_duty_cycle_microsec;
    double           center_duty_cycle_microsec;

    tinyxml2::XMLElement *root = doc.FirstChildElement();
    if (root == nullptr)
    {
        RCLCPP_INFO(this->get_logger(), "Error finding root, good luck");
        // std::cout << "Error parsing file: COuldn't find root, you're fucked" << std::endl;
        return false;
    }

    tinyxml2::XMLElement *config_servo_cams = root->FirstChildElement("ConfigServoCams");
    if (config_servo_cams == nullptr)
    {
        RCLCPP_INFO(this->get_logger(), "Error finding ConfigServoCams");
        // std::cout << "Error parsing file: Couldn't find ConfigServoCams" << std::endl;
        return false;
    }

    for (int i = 0; i < 2; i++)
    {
        servo_cams[i] = cave_talk::Servo();

        std::string servo_cams_name = "Servo_Cam_";
        if (i == 0)
        {
            servo_cams_name += "Pan";
        }
        else
        {
            servo_cams_name += "Tilt";
        }

        tinyxml2::XMLElement *servo_cams_xml = config_servo_cams->FirstChildElement(servo_cams_name.c_str());
        if (servo_cams_xml == nullptr)
        {
            RCLCPP_INFO(this->get_logger(), "Error parsing file: Couldn't find %s", servo_cams_name.c_str());
            // std::cout << "Error parsing file: Couldnt find " << servo_cams_name << std::endl;
            return false;
        }

        int extractResult = -1;

        tinyxml2::XMLElement *min_angle_radian_node = servo_cams_xml->FirstChildElement("min_angle_radian");
        if (min_angle_radian_node != nullptr)
        {
            extractResult = min_angle_radian_node->QueryDoubleText(&min_angle_radian);

            if (extractResult == 0)
            {
                servo_cams[i].set_min_angle_radian(min_angle_radian);
                RCLCPP_INFO(this->get_logger(), "min Angle Radian: %f", min_angle_radian);
                // std::cout << min_angle_radian << std::endl;
            }
            else
            {
                std::cout << "Failed to extract min angle radian" << std::endl;
            }

            if (i == 0)
            {
                min_cam_pan_radian_ = min_angle_radian;
            }
            else
            {
                min_cam_tilt_radian_ = min_angle_radian;
            }
        }
        else
        {
            servo_cams[i].set_min_angle_radian(-1);
        }

        tinyxml2::XMLElement *max_angle_radian_node = servo_cams_xml->FirstChildElement("max_angle_radian");
        if (max_angle_radian_node != nullptr)
        {
            extractResult = max_angle_radian_node->QueryDoubleText(&max_angle_radian);

            if (extractResult == 0)
            {
                servo_cams[i].set_max_angle_radian(max_angle_radian);
                RCLCPP_INFO(this->get_logger(), "max Angle Radian: %f", max_angle_radian);
                // std::cout << max_angle_radian << std::endl;
            }
            else
            {
                std::cout << "Failed to extract max angle radian" << std::endl;
            }

            if (i == 0)
            {
                max_cam_pan_radian_ = max_angle_radian;
            }
            else
            {
                max_cam_tilt_radian_ = max_angle_radian;
            }
        }
        else
        {
            servo_cams[i].set_max_angle_radian(-1);
        }

        tinyxml2::XMLElement *center_angle_radian_node = servo_cams_xml->FirstChildElement("center_angle_radian");
        if (center_angle_radian_node != nullptr)
        {
            extractResult = center_angle_radian_node->QueryDoubleText(&center_angle_radian);

            if (extractResult == 0)
            {
                servo_cams[i].set_center_angle_radian(center_angle_radian);
                RCLCPP_INFO(this->get_logger(), "center Angle Radian: %f", center_angle_radian);
                // std::cout << center_angle_radian << std::endl;
            }
            else
            {
                std::cout << "Failed to extract center angle radian" << std::endl;
            }

            if (i == 0)
            {
                cam_pan_ = center_angle_radian;
            }
            else
            {
                cam_tilt_ = center_angle_radian;
            }
        }
        else
        {
            servo_cams[i].set_center_angle_radian(-1);
        }

        tinyxml2::XMLElement *min_duty_cycle_microsec_node = servo_cams_xml->FirstChildElement("min_duty_cycle_microsec");
        if (min_duty_cycle_microsec_node != nullptr)
        {
            extractResult = min_duty_cycle_microsec_node->QueryDoubleText(&min_duty_cycle_microsec);

            if (extractResult == 0)
            {
                servo_cams[i].set_min_duty_cycle_percentage(min_duty_cycle_microsec);
                RCLCPP_INFO(this->get_logger(), "Min Angle Radian: %f", min_duty_cycle_microsec);
                // std::cout << min_duty_cycle_microsec << std::endl;
            }
            else
            {
                std::cout << "Failed to extract min angle radian" << std::endl;
            }
        }
        else
        {
            servo_cams[i].set_min_duty_cycle_percentage(-1);
        }

        tinyxml2::XMLElement *max_duty_cycle_microsec_node = servo_cams_xml->FirstChildElement("max_duty_cycle_microsec");
        if (max_duty_cycle_microsec_node != nullptr)
        {
            extractResult = max_duty_cycle_microsec_node->QueryDoubleText(&max_duty_cycle_microsec);

            if (extractResult == 0)
            {
                servo_cams[i].set_max_duty_cycle_percentage(max_duty_cycle_microsec);
                RCLCPP_INFO(this->get_logger(), "max Angle Radian: %f", max_duty_cycle_microsec);
                // std::cout << max_duty_cycle_microsec << std::endl;
            }
            else
            {
                std::cout << "Failed to extract max angle radian" << std::endl;
            }
        }
        else
        {
            servo_cams[i].set_max_duty_cycle_percentage(-1);
        }

        tinyxml2::XMLElement *center_duty_cycle_microsec_node = servo_cams_xml->FirstChildElement("center_duty_cycle_microsec");
        if (center_duty_cycle_microsec_node != nullptr)
        {
            extractResult = center_duty_cycle_microsec_node->QueryDoubleText(&center_duty_cycle_microsec);

            if (extractResult == 0)
            {
                servo_cams[i].set_center_duty_cycle_percentage(center_duty_cycle_microsec);
                RCLCPP_INFO(this->get_logger(), "center Angle Radian: %f", center_duty_cycle_microsec);
                // std::cout << center_duty_cycle_microsec << std::endl;
            }
            else
            {
                std::cout << "Failed to extract center angle radian" << std::endl;
            }
        }
        else
        {
            servo_cams[i].set_center_duty_cycle_percentage(-1);
        }
    }

    talker->SpeakConfigServoCams(servo_cams[0], servo_cams[1]);
    return true;
}
bool RoverComm::openAndSendConfigMotor(std::string file)
{
    tinyxml2::XMLDocument doc;
    tinyxml2::XMLError    error = doc.LoadFile(file.c_str());
    // std::cout << error << std::endl;
    if (error != tinyxml2::XML_SUCCESS)
    {
        RCLCPP_INFO(this->get_logger(), "Error opening file: %s", file.c_str());
        // std::cout << "Open File Error" << std::endl;
        return false;
    }

    cave_talk::Motor motor_wheels[4];
    double           pwm_carrier_freq_hz;
    double           min_speed_loaded_mps;
    double           max_speed_loaded_mps;
    double           min_duty_cycle_percent;
    double           max_duty_cycle_percent;

    tinyxml2::XMLElement *root = doc.FirstChildElement();
    if (root == nullptr)
    {
        RCLCPP_INFO(this->get_logger(), "Error finding root, good luck");
        // std::cout << "Error parsing file: COuldn't find root, you're fucked" << std::endl;
        return false;
    }

    tinyxml2::XMLElement *config_motor = root->FirstChildElement("ConfigMotor");
    if (config_motor == nullptr)
    {
        RCLCPP_INFO(this->get_logger(), "Error finding ConfigMotor");
        // std::cout << "Error parsing file: Couldn't find ConfigMotor" << std::endl;
        return false;
    }

    for (int i = 0; i < 4; i++)
    {
        motor_wheels[i] = cave_talk::Motor();

        std::string motor_wheel_name = "Motor_Wheel_" + std::to_string(i);

        tinyxml2::XMLElement *motor_wheel = config_motor->FirstChildElement(motor_wheel_name.c_str());
        if (motor_wheel == nullptr)
        {
            RCLCPP_INFO(this->get_logger(), "Error parsing file: Couldn't find %s", motor_wheel_name.c_str());
            // std::cout << "Error parsing file: Couldnt find " << motor_wheel_name << std::endl;
            return false;
        }

        int extractResult = -1;

        tinyxml2::XMLElement *pwm_freq_node = motor_wheel->FirstChildElement("min_angle_radian");
        if (pwm_freq_node != nullptr)
        {
            extractResult = pwm_freq_node->QueryDoubleText(&pwm_carrier_freq_hz);

            if (extractResult == 0)
            {
                motor_wheels[i].set_pwm_carrier_freq_hz(pwm_carrier_freq_hz);
                RCLCPP_INFO(this->get_logger(), "min Angle Radian: %f", pwm_carrier_freq_hz);
                // std::cout << pwm_carrier_freq_hz << std::endl;
            }
            else
            {
                std::cout << "Failed to extract min angle radian" << std::endl;
            }
        }
        else
        {
            motor_wheels[i].set_pwm_carrier_freq_hz(-1);
        }

        tinyxml2::XMLElement *max_speed_loaded_mps_node = motor_wheel->FirstChildElement("max_speed_loaded_mps");
        if (max_speed_loaded_mps_node != nullptr)
        {
            extractResult = max_speed_loaded_mps_node->QueryDoubleText(&max_speed_loaded_mps);

            if (extractResult == 0)
            {
                motor_wheels[i].set_max_speed_loaded_meters_per_second(max_speed_loaded_mps);
                RCLCPP_INFO(this->get_logger(), "max speed loaded meters per second: %f", max_speed_loaded_mps);
                // std::cout << max_speed_loaded_mps << std::endl;
            }
            else
            {
                std::cout << "Failed to extract max speed loaded meters per second" << std::endl;
            }
        }
        else
        {
            motor_wheels[i].set_max_speed_loaded_meters_per_second(-1);
        }

        tinyxml2::XMLElement *min_speed_loaded_mps_node = motor_wheel->FirstChildElement("min_speed_loaded_mps");
        if (min_speed_loaded_mps_node != nullptr)
        {
            extractResult = min_speed_loaded_mps_node->QueryDoubleText(&min_speed_loaded_mps);

            if (extractResult == 0)
            {
                motor_wheels[i].set_min_speed_loaded_meters_per_second(min_speed_loaded_mps);
                RCLCPP_INFO(this->get_logger(), "min speed loaded m/s: %f", min_speed_loaded_mps);
                // std::cout << min_speed_loaded_mps << std::endl;
            }
            else
            {
                std::cout << "Failed to extract min speed loaded m/s" << std::endl;
            }
        }
        else
        {
            motor_wheels[i].set_min_speed_loaded_meters_per_second(-1);
        }

        tinyxml2::XMLElement *min_duty_cycle_percent_node = motor_wheel->FirstChildElement("min_duty_cycle_percent");
        if (min_duty_cycle_percent_node != nullptr)
        {
            extractResult = min_duty_cycle_percent_node->QueryDoubleText(&min_duty_cycle_percent);

            if (extractResult == 0)
            {
                motor_wheels[i].set_min_duty_cycle_percentage(min_duty_cycle_percent);
                RCLCPP_INFO(this->get_logger(), "Min Angle Radian: %f", min_duty_cycle_percent);
                // std::cout << min_duty_cycle_percent << std::endl;
            }
            else
            {
                std::cout << "Failed to extract min angle radian" << std::endl;
            }
        }
        else
        {
            motor_wheels[i].set_min_duty_cycle_percentage(-1);
        }

        tinyxml2::XMLElement *max_duty_cycle_percent_node = motor_wheel->FirstChildElement("max_duty_cycle_percent");
        if (max_duty_cycle_percent_node != nullptr)
        {
            extractResult = max_duty_cycle_percent_node->QueryDoubleText(&max_duty_cycle_percent);

            if (extractResult == 0) {
                motor_wheels[i].set_max_duty_cycle_percentage(max_duty_cycle_percent);
                RCLCPP_INFO(this->get_logger(), "max Angle Radian: %f", max_duty_cycle_percent);
                // std::cout << max_duty_cycle_percent << std::endl;
            }else{
                std::cout << "Failed to extract max angle radian" << std::endl;
            } 
        }
        else
        {
            motor_wheels[i].set_max_duty_cycle_percentage(-1);
        }

        
    }

    talker->SpeakConfigMotor(motor_wheels[0], motor_wheels[1], motor_wheels[2], motor_wheels[3]);
    return true;
}

bool RoverComm::readCameraMovementConfig(std::string file){
 
    tinyxml2::XMLDocument doc;
    tinyxml2::XMLError error = doc.LoadFile(file.c_str());
    if (error != tinyxml2::XML_SUCCESS) {
        RCLCPP_INFO(this->get_logger(), "Error opening file: %s", file.c_str());
        // std::cout << "Open File Error" << std::endl;
        return false;
    }

    tinyxml2::XMLElement *root_xml = doc.FirstChildElement();
    if (root_xml == nullptr) {
        RCLCPP_INFO(this->get_logger(), "Error finding root, good luck");
        // std::cout << "Error parsing file: COuldn't find root, you're fucked" << std::endl;
        return false;
    }

    tinyxml2::XMLElement *cam_movements = root_xml->FirstChildElement("CameraMovement");
    if (cam_movements == nullptr) {
        RCLCPP_INFO(this->get_logger(), "Error finding CameraMovement");
        // std::cout << "Error parsing file: Couldn't find ConfigEncoder" << std::endl;
        return false;
    }

    int idx = 0;
    std::string durationTime = "duration";
    double durationTimeDouble;
    double cam_pan_rad_ref;
    double cam_tilt_rad_ref;
    int extractResult = -1;
    for(tinyxml2::XMLElement *profile = cam_movements->FirstChildElement(); profile != NULL; profile = profile->NextSiblingElement())
    {
        int posIdx = 0;
        for(tinyxml2::XMLElement *position = profile->FirstChildElement(); position != NULL; position = position->NextSiblingElement())
        {
            extractResult = profile->QueryDoubleAttribute(durationTime.c_str(), &durationTimeDouble);
            if (extractResult == 0) {
                profiles[idx].durations[posIdx] = durationTimeDouble;
                RCLCPP_INFO(this->get_logger(), "Duration: %f", durationTimeDouble);
                // std::cout << profiles[idx].duration << std::endl;
            }

            tinyxml2::XMLElement *cam_tilt_node = position->FirstChildElement("Cam_Tilt_Radians");
            if (cam_tilt_node != nullptr)
            {
                extractResult = cam_tilt_node->QueryDoubleText(&cam_tilt_rad_ref);

                if (extractResult == 0) {
                    profiles[idx].cam_pan_radians[posIdx] = cam_tilt_rad_ref;
                    // RCLCPP_INFO(this->get_logger(), "max Angle Radian: %f", cam_tilt_rad_ref);
                    // std::cout << cam_tilt_rad_ref << std::endl;
                }
            }

            tinyxml2::XMLElement *cam_pan_node = position->FirstChildElement("Cam_Pan_Radians");
            if (cam_pan_node != nullptr)
            {
                extractResult = cam_pan_node->QueryDoubleText(&cam_pan_rad_ref);

                if (extractResult == 0) {
                    profiles[idx].cam_pan_radians[posIdx] = cam_pan_rad_ref;
                    // RCLCPP_INFO(this->get_logger(), "max Angle Radian: %f", cam_pan_rad_ref);
                    // std::cout << cam_pan_rad_ref << std::endl;
                }
            }
            posIdx++;
        }
        profiles[idx].length = posIdx;

        idx++;
    }
    
    return true;

}