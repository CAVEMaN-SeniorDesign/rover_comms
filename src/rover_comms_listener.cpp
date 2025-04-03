#include "rover_comms_listener.hpp"


// ListenerCallbacks implementation
RoverCommsListener::RoverCommsListener(std::shared_ptr<RoverComm> node) : rover_comm_node_(node) // assigns to rover_comm_node_
{

}

void RoverCommsListener::HearOogaBooga(const cave_talk::Say ooga_booga)
{
    std::string output = "Heard ";

    if (ooga_booga == cave_talk::SAY_OOGA)
    {
        CaveTalk_Error_t error_Ooga = (rover_comm_node_->talker)->SpeakOogaBooga(cave_talk::SAY_BOOGA);
        if (error_Ooga == CAVE_TALK_ERROR_NONE)
        {
            output = output + "OOGA, said BOOGA";
        }
        else
        {
            output = "Ooga error";
        }
    }
    else if (ooga_booga == cave_talk::SAY_BOOGA)
    {
        rover_comm_node_->waiting_booga = false;
        // if(rover_comm_node_->speak_timer_){
        //     rover_comm_node_->speak_timer_->cancel(); // stops timer callbacks
        // }
        rover_comm_node_->speak_timer_.reset(); // stops timer callbacks
        output = output + "Heard Booga, loop closed";
    }

    RCLCPP_INFO(rover_comm_node_->get_logger(), output.c_str());
}

void RoverCommsListener::HearMovement(const CaveTalk_MetersPerSecond_t speed, const CaveTalk_RadiansPerSecond_t turn_rate)
{
    (void)speed;
    (void)turn_rate;
    RCLCPP_INFO(rover_comm_node_->get_logger(), "wagu!?");
}

void RoverCommsListener::HearCameraMovement(const CaveTalk_Radian_t pan, const CaveTalk_Radian_t tilt)
{
    (void)pan;
    (void)tilt;
    RCLCPP_INFO(rover_comm_node_->get_logger(), "maka-keega");
}

void RoverCommsListener::HearLights(const bool headlights)
{
    (void)headlights;
    RCLCPP_INFO(rover_comm_node_->get_logger(), "unk!");
}

void RoverCommsListener::HearConfigEncoder(const cave_talk::ConfigEncoder &encoder_wheel_0, const cave_talk::ConfigEncoder &encoder_wheel_1, const cave_talk::ConfigEncoder &encoder_wheel_2, const cave_talk::ConfigEncoder &encoder_wheel_3)
{
    (void)encoder_wheel_0;
    (void)encoder_wheel_1;
    (void)encoder_wheel_2;
    (void)encoder_wheel_3;
}

void RoverCommsListener::HearArm(const bool arm)
{
    (void)arm;
    RCLCPP_INFO(rover_comm_node_->get_logger(), "oosha");
}

void RoverCommsListener::HearConfigServoWheels(const cave_talk::Servo &servo_wheel_0, const cave_talk::Servo &servo_wheel_1, const cave_talk::Servo &servo_wheel_2, const cave_talk::Servo &servo_wheel_3)
{
    (void)servo_wheel_0;
    (void)servo_wheel_1;
    (void)servo_wheel_2;
    (void)servo_wheel_3;
    RCLCPP_INFO(rover_comm_node_->get_logger(), "Hear config servowheels not impl.");
}

void RoverCommsListener::HearConfigServoCams(const cave_talk::Servo &servo_cam_pan, const cave_talk::Servo &servo_cam_tilt)
{
    (void)servo_cam_pan;
    (void)servo_cam_tilt;
    RCLCPP_INFO(rover_comm_node_->get_logger(), "Hear config servocams not impl.");
}

void RoverCommsListener::HearConfigMotor(const cave_talk::Motor &motor_wheel_0, const cave_talk::Motor &motor_wheel_1, const cave_talk::Motor &motor_wheel_2, const cave_talk::Motor &motor_wheel_3)
{
    (void)motor_wheel_0;
    (void)motor_wheel_1;
    (void)motor_wheel_2;
    (void)motor_wheel_3;
    RCLCPP_INFO(rover_comm_node_->get_logger(), "Hear config motor not impl.");
}

void RoverCommsListener::HearConfigLog(const cave_talk::LogLevel log_level)
{
    (void)log_level;
    RCLCPP_INFO(rover_comm_node_->get_logger(), "Heard log_level");
}

void RoverCommsListener::HearOdometry(const cave_talk::Imu &IMU, const cave_talk::Encoder &encoder_wheel_0, const cave_talk::Encoder &encoder_wheel_1, const cave_talk::Encoder &encoder_wheel_2, const cave_talk::Encoder &encoder_wheel_3)
{
    auto msg = rover_interfaces::msg::Encoders();

    msg.total_pulses_encoder_wheel_0 = encoder_wheel_0.total_pulses();
    msg.rate_rads_per_sec_encoder_wheel_0 = encoder_wheel_0.rate_radians_per_second();

    msg.total_pulses_encoder_wheel_1 = encoder_wheel_1.total_pulses();
    msg.rate_rads_per_sec_encoder_wheel_1 = encoder_wheel_1.rate_radians_per_second();

    msg.total_pulses_encoder_wheel_2 = encoder_wheel_2.total_pulses();
    msg.rate_rads_per_sec_encoder_wheel_2 = encoder_wheel_2.rate_radians_per_second();

    msg.total_pulses_encoder_wheel_3 = encoder_wheel_3.total_pulses();
    msg.rate_rads_per_sec_encoder_wheel_3 = encoder_wheel_3.rate_radians_per_second();

    // Removed to publish directly to imu topic
    // msg.x_accel_mpss = IMU.accel().x_meters_per_second_squared(); 
    // msg.y_accel_mpss = IMU.accel().y_meters_per_second_squared();
    // msg.z_accel_mpss = IMU.accel().z_meters_per_second_squared();
    // msg.roll_rads_per_sec = IMU.gyro().roll_radians_per_second();
    // msg.pitch_rads_per_sec = IMU.gyro().pitch_radians_per_second();
    // msg.yaw_rads_per_sec = IMU.gyro().yaw_radians_per_second();

    rover_comm_node_->odom_read_pub_->publish(msg);

    MadgwickAHRSupdateIMU(  
        IMU.accel().x_meters_per_second_squared(),
        IMU.accel().y_meters_per_second_squared(),
        IMU.accel().z_meters_per_second_squared(),
        IMU.gyro().roll_radians_per_second(),
        IMU.gyro().pitch_radians_per_second(),
        IMU.gyro().yaw_radians_per_second(),
        std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - prev_time_pt_)
    );

    std::string quat_str = "Quatw: " + std::to_string(q0_) + "Quatx: " + std::to_string(q1_) + "Quaty: " + std::to_string(q2_) + "Quatz: " + std::to_string(q3_);
    RCLCPP_INFO(rover_comm_node_->get_logger(), "Heard odom msgs");
    RCLCPP_INFO(rover_comm_node_->get_logger(), quat_str.c_str());
}

void RoverCommsListener::HearLog(const char *const log)
{
    RCLCPP_INFO(rover_comm_node_->get_logger(), "Heard log: %s", log);
}

void RoverCommsListener::MadgwickAHRSupdateIMU(double gx, double gy, double gz,
    double ax, double ay, double az, std::chrono::milliseconds dt)
{
    auto imu_msg = sensor_msgs::msg::Imu();

    double beta = 0.1;
    double recipNorm;
    double s0, s1, s2, s3;
    double qDot1, qDot2, qDot3, qDot4;
    double _2q0, _2q1, _2q2, _2q3;
    double _4q0, _4q1, _4q2;
    double _8q1, _8q2;
    double q0q0, q1q1, q2q2, q3q3;

    // Rate of change of quaternion from gyroscope
    qDot1 = 0.5 * (-q1_ * gx - q2_ * gy - q3_ * gz);
    qDot2 = 0.5 * (q0_ * gx + q2_ * gz - q3_ * gy);
    qDot3 = 0.5 * (q0_ * gy - q1_ * gz + q3_ * gx);
    qDot4 = 0.5 * (q0_ * gz + q1_ * gy - q2_ * gx);

    // Compute feedback only if accelerometer measurement valid
    if (!(ax == 0.0 && ay == 0.0 && az == 0.0))
    {
        // Normalize accelerometer measurement
        recipNorm = 1.0 / std::sqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Auxiliary variables to avoid repeated arithmetic
        _2q0 = 2.0 * q0_;
        _2q1 = 2.0 * q1_;
        _2q2 = 2.0 * q2_;
        _2q3 = 2.0 * q3_;
        _4q0 = 4.0 * q0_;
        _4q1 = 4.0 * q1_;
        _4q2 = 4.0 * q2_;
        _8q1 = 8.0 * q1_;
        _8q2 = 8.0 * q2_;
        q0q0 = q0_ * q0_;
        q1q1 = q1_ * q1_;
        q2q2 = q2_ * q2_;
        q3q3 = q3_ * q3_;

        // Gradient descent algorithm corrective step
        s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        s1 = _4q1 * q3q3 - _2q3 * ax + 4.0 * q0q0 * q1_ - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        s2 = 4.0 * q0q0 * q2_ + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        s3 = 4.0 * q1q1 * q3_ - _2q1 * ax + 4.0 * q2q2 * q3_ - _2q2 * ay;
        recipNorm = 1.0 / std::sqrt((s0 * s0) + (s1 * s1) + (s2 * s2) + (s3 * s3)); // normalize step magnitude
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        // Apply feedback step
        qDot1 -= beta * s0;
        qDot2 -= beta * s1;
        qDot3 -= beta * s2;
        qDot4 -= beta * s3;
    }

    // Integrate rate of change to yield quaternion
    q0_ += qDot1 * (dt.count() * .001);
    q1_ += qDot2 * (dt.count() * .001);
    q2_ += qDot3 * (dt.count() * .001);
    q3_ += qDot4 * (dt.count() * .001);

    // Normalize quaternion
    recipNorm = 1.0 / std::sqrt((q0_ * q0_) + (q1_ * q1_) + (q2_ * q2_) + (q3_ * q3_));
    q0_ *= recipNorm;
    q1_ *= recipNorm;
    q2_ *= recipNorm;
    q3_ *= recipNorm;

    // MARK: EXPORT TO RTAB
    imu_msg.header.stamp = rover_comm_node_->now();
    imu_msg.orientation.w = q0_;
    imu_msg.orientation.x = q1_;
    imu_msg.orientation.y = q2_;
    imu_msg.orientation.z = q3_;

    // TODO: TUNE THIS
    imu_msg.orientation_covariance = {
        0.01, 0.0,  0.0,
        0.0,  0.01, 0.0,
        0.0,  0.0,  0.01
    };

    imu_msg.angular_velocity.x = gx;
    imu_msg.angular_velocity.y = gy;
    imu_msg.angular_velocity.z = gz;

    // TODO: TUNE THIS
    imu_msg.angular_velocity_covariance = {
        0.001, 0.0,   0.0,
        0.0,   0.001, 0.0,
        0.0,   0.0,   0.001
    };

    imu_msg.linear_acceleration.x = ax;
    imu_msg.linear_acceleration.y = ay;
    imu_msg.linear_acceleration.z = az;

    // TODO: TUNE THIS
    imu_msg.linear_acceleration_covariance = {
        0.04, 0.0,  0.0,
        0.0,  0.04, 0.0,
        0.0,  0.0,  0.04
    };

    rover_comm_node_->imu_pub_->publish(imu_msg);
}
