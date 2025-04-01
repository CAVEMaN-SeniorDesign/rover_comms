#ifndef ROVER_COMM_LISTENER_HPP
#define ROVER_COMM_LISTENER_HPP

#include "rover_comm.hpp"

/*Common ports that are assigned on default (we should implement automatic port find and selection):
    /dev/ttyTHS1 - Jetson Nano built in UART pins on pin 8 (TX) and pin 10 (RX)
    /dev/ttyUSB0 - This should usually be the default for a FTDI adapter but...
    /dev/ttyUSB1 - Sometimes it switches to this for some reason.
*/

class RoverCommsListener : public cave_talk::ListenerCallbacks
{
public:
    RoverCommsListener(std::shared_ptr<RoverComm> node); //possibly need "rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr" instead
    void HearOogaBooga(const cave_talk::Say ooga_booga) override;
    void HearMovement(const CaveTalk_MetersPerSecond_t speed, const CaveTalk_RadiansPerSecond_t turn_rate) override;
    void HearCameraMovement(const CaveTalk_Radian_t pan, const CaveTalk_Radian_t tilt) override;
    void HearLights(const bool headlights) override;
    void HearArm(const bool arm) override;
    void HearConfigServoWheels(const cave_talk::Servo &servo_wheel_0, const cave_talk::Servo &servo_wheel_1, const cave_talk::Servo &servo_wheel_2, const cave_talk::Servo &servo_wheel_3) override;
    void HearConfigServoCams(const cave_talk::Servo &servo_cam_pan, const cave_talk::Servo &servo_cam_tilt) override;
    void HearConfigMotor(const cave_talk::Motor &motor_wheel_0, const cave_talk::Motor &motor_wheel_1, const cave_talk::Motor &motor_wheel_2, const cave_talk::Motor &motor_wheel_3) override;
	void HearConfigEncoder(const cave_talk::ConfigEncoder &encoder_wheel_0, const cave_talk::ConfigEncoder &encoder_wheel_1, const cave_talk::ConfigEncoder &encoder_wheel_2, const cave_talk::ConfigEncoder &encoder_wheel_3) override;
    void HearConfigLog(const cave_talk::LogLevel log_level) override;
    void HearOdometry(const cave_talk::Imu &IMU, const cave_talk::Encoder &encoder_wheel_0, const cave_talk::Encoder &encoder_wheel_1, const cave_talk::Encoder &encoder_wheel_2, const cave_talk::Encoder &encoder_wheel_3) override;
    void HearLog(const char *const log) override;

private:
    std::shared_ptr<RoverComm> rover_comm_node_;
};

#endif // ROVER_COMM_LISTENER_HPP