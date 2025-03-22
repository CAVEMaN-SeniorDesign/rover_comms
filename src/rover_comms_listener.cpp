#include "rover_comms_listener.hpp"

// cave_talk::ListenerCallbacks::~ListenerCallbacks() = default;


// ListenerCallbacks implementation
RoverCommsListener::RoverCommsListener(std::shared_ptr<RoverComm> node):rover_comm_node_(node) // assigns to rover_comm_node_
{

}

void RoverCommsListener::HearOogaBooga(const cave_talk::Say ooga_booga)
{
    std::string output  = "Heard ";

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
        // if(rover_comm_node_->speak_timer_){
        //     rover_comm_node_->speak_timer_->cancel(); // stops timer callbacks
        // }
        rover_comm_node_->speak_timer_.reset(); // stops timer callbacks
        output = output + "Heard Booga, loop closed";
    }

    RCLCPP_INFO(rover_comm_node_->get_logger(), output);
}

void RoverCommsListener::HearMovement(const CaveTalk_MetersPerSecond_t speed, const CaveTalk_RadiansPerSecond_t turn_rate)
{
    RCLCPP_INFO(rover_comm_node_->get_logger(), "wagu!?");
}

void RoverCommsListener::HearCameraMovement(const CaveTalk_Radian_t pan, const CaveTalk_Radian_t tilt)
{
    RCLCPP_INFO(rover_comm_node_->get_logger(), "maka-keega");
}

void RoverCommsListener::HearLights(const bool headlights)
{
    RCLCPP_INFO(rover_comm_node_->get_logger(), "unk!");
}

void RoverCommsListener::HearConfigEncoder(const cave_talk::ConfigEncoder &encoder_wheel_0, const cave_talk::ConfigEncoder &encoder_wheel_1, const cave_talk::ConfigEncoder &encoder_wheel_2, const cave_talk::ConfigEncoder &encoder_wheel_3){}

void RoverCommsListener::HearArm(const bool arm)
{
    RCLCPP_INFO(rover_comm_node_->get_logger(), "oosha");
}

void RoverCommsListener::HearConfigServoWheels(const cave_talk::Servo &servo_wheel_0, const cave_talk::Servo &servo_wheel_1, const cave_talk::Servo &servo_wheel_2, const cave_talk::Servo &servo_wheel_3)
{
    RCLCPP_INFO(rover_comm_node_->get_logger(), "Hear config servowheels not impl.");
}

void RoverCommsListener::HearConfigServoCams(const cave_talk::Servo &servo_cam_pan, const cave_talk::Servo &servo_cam_tilt)
{
    RCLCPP_INFO(rover_comm_node_->get_logger(), "Hear config servocams not impl.");
}

void RoverCommsListener::HearConfigMotor(const cave_talk::Motor &motor_wheel_0, const cave_talk::Motor &motor_wheel_1, const cave_talk::Motor &motor_wheel_2, const cave_talk::Motor &motor_wheel_3)
{
    RCLCPP_INFO(rover_comm_node_->get_logger(), "Hear config motor not impl.");
}

void RoverCommsListener::HearOdometry(const cave_talk::Imu &IMU, const cave_talk::Encoder &encoder_wheel_0, const cave_talk::Encoder &encoder_wheel_1, const cave_talk::Encoder &encoder_wheel_2, const cave_talk::Encoder &encoder_wheel_3)
{
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

void RoverCommsListener::HearLog(const char *const log)
{
    RCLCPP_INFO(rover_comm_node_->get_logger(), "Heard log: %s", log);
}