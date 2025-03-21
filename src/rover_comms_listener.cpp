#include "rover_comms_listener.hpp"



// ListenerCallbacks implementation
RoverCommsListener::RoverCommsListener(std::shared_ptr<RoverComm> node):rover_comm_node_(node) // assigns to rover_comm_node_
{

}

cave_talk::ListenerCallbacks::~ListenerCallbacks() = default;

void RoverCommsListener::HearOogaBooga(const cave_talk::Say ooga_booga)
{
    std::string output  = "Heard ";
    if (rover_comm_node_){
        if (ooga_booga==cave_talk::SAY_OOGA){
            CaveTalk_Error_t error_Ooga = (rover_comm_node_->talker)->SpeakOogaBooga(cave_talk::SAY_BOOGA);
            if (error_Ooga == CAVE_TALK_ERROR_NONE){
                output = output + "OOGA, said BOOGA";
            }
            else{
                output = "Ooga error";
            }
        }
        else if (ooga_booga==cave_talk::SAY_BOOGA){
            // (rover_comm_node_->talker)->SpeakOogaBooga(cave_talk::SAY_OOGA);
            rover_comm_node_->waiting_booga = false;
            if(rover_comm_node_->speak_timer_){
                rover_comm_node_->speak_timer_->cancel(); // stops timer callbacks
            }
            output = output + "Heard Booga, loop closed";
        }

        RCLCPP_INFO(rover_comm_node_->get_logger(), output);
    }
    else if (direct_talker_){ 
        direct_talker_->SpeakOogaBooga(cave_talk::SAY_BOOGA);
        std::cout<<"Heard OOGA, said BOOGA"<<std::endl;
    }
}

void RoverCommsListener::HearMovement(const CaveTalk_MetersPerSecond_t speed, const CaveTalk_RadiansPerSecond_t turn_rate)
{
    if (rover_comm_node_){
        RCLCPP_INFO(rover_comm_node_->get_logger(), "wagu!?");
    }
    else{
        std::cout<<"wagu!?"<<std::endl;
    }
}

void RoverCommsListener::HearCameraMovement(const CaveTalk_Radian_t pan, const CaveTalk_Radian_t tilt)
{
    if (rover_comm_node_){
        RCLCPP_INFO(rover_comm_node_->get_logger(), "wagu!?");
    }
    else{
        std::cout<<"wagu!?"<<std::endl;
    }
    RCLCPP_INFO(rover_comm_node_->get_logger(), "maka-keega");
}

void RoverCommsListener::HearLights(const bool headlights)
{
    if (rover_comm_node_){
        RCLCPP_INFO(rover_comm_node_->get_logger(), "wagu!?");
    }
    else{
        std::cout<<"wagu!?"<<std::endl;
    }

    RCLCPP_INFO(rover_comm_node_->get_logger(), "unk!");
}

/*
void RoverCommsListener::HearMode(const bool manual)
{
    if (rover_comm_node_){
        RCLCPP_INFO(rover_comm_node_->get_logger(), "oosha");
    }
    else{
        std::cout<<"wagu!?"<<std::endl;
    }
}
*/
void RoverCommsListener::HearConfigServoWheels(const cave_talk::Servo &servo_wheel_0, const cave_talk::Servo &servo_wheel_1, const cave_talk::Servo &servo_wheel_2, const cave_talk::Servo &servo_wheel_3)
{
    if (rover_comm_node_){
        RCLCPP_INFO(rover_comm_node_->get_logger(), "Hear config servowheels not impl.");
    }
    else{
        std::cout<<"wagu!?"<<std::endl;
    }
}

void RoverCommsListener::HearConfigServoCams(const cave_talk::Servo &servo_cam_pan, const cave_talk::Servo &servo_cam_tilt)
{
    if (rover_comm_node_){
        RCLCPP_INFO(rover_comm_node_->get_logger(), "Hear config servocams not impl.");
    }
    else{
        std::cout<<"wagu!?"<<std::endl;
    }
}

void RoverCommsListener::HearConfigMotor(const cave_talk::Motor &motor_wheel_0, const cave_talk::Motor &motor_wheel_1, const cave_talk::Motor &motor_wheel_2, const cave_talk::Motor &motor_wheel_3)
{
    if (rover_comm_node_){
        RCLCPP_INFO(rover_comm_node_->get_logger(), "Hear config motor not impl.");
    }
    else{
        std::cout<<"wagu!?"<<std::endl;
    }
}

void RoverCommsListener::HearOdometry(const cave_talk::Imu &IMU, const cave_talk::Encoder &encoder_wheel_0, const cave_talk::Encoder &encoder_wheel_1, const cave_talk::Encoder &encoder_wheel_2, const cave_talk::Encoder &encoder_wheel_3)
{
    if (rover_comm_node_){
        auto msg = rover_interfaces::msg::Odomplot();

        msg.total_pulses_encoder_wheel_0 = encoder_wheel_0.total_pulses();
        msg.rate_rads_per_sec_encoder_wheel_0 = encoder_wheel_0.rate_radians_per_second();

        msg.total_pulses_encoder_wheel_1 = encoder_wheel_1.total_pulses();
        msg.rate_rads_per_sec_encoder_wheel_1 = encoder_wheel_1.rate_radians_per_second();

        msg.total_pulses_encoder_wheel_2 = encoder_wheel_2.total_pulses();
        msg.rate_rads_per_sec_encoder_wheel_2 = encoder_wheel_2.rate_radians_per_second();

        msg.total_pulses_encoder_wheel_3 = encoder_wheel_3.total_pulses();
        msg.rate_rads_per_sec_encoder_wheel_3 = encoder_wheel_3.rate_radians_per_second();

        msg.x_accel_mpss = IMU.accel().x_meters_per_second_squared(); 
        msg.y_accel_mpss = IMU.accel().y_meters_per_second_squared();
        msg.z_accel_mpss = IMU.accel().z_meters_per_second_squared();
        msg.roll_rads_per_sec = IMU.gyro().roll_radians_per_second();
        msg.pitch_rads_per_sec = IMU.gyro().pitch_radians_per_second();
        msg.yaw_rads_per_sec = IMU.gyro().yaw_radians_per_second();

        rover_comm_node_->odom_read_pub_->publish(msg);

        RCLCPP_INFO(rover_comm_node_->get_logger(), "Heard odom msgs");
    }
    else{
        std::cout<<"Heard Odom"<<std::endl;
    }
}

void RoverCommsListener::HearLog(const char *const log)
{
    if(rover_comm_node_){
        RCLCPP_INFO(rover_comm_node_->get_logger(), "Heard log: %s", log);
    }
    else{
        std::cout<<"Heard log: "<<log<<std::endl;
    }
}
//void RoverCommsListener::HearConfigEncoder(const cave_talk::ConfigEncoder &encoder_wheel_0, const cave_talk::ConfigEncoder &encoder_wheel_1, const cave_talk::ConfigEncoder &encoder_wheel_2, const cave_talk::ConfigEncoder &encoder_wheel_3) {}
//void RoverCommsListener::HearArm(const bool arm){}
