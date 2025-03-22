#define CATCH_CONFIG_MAIN
#include <catch2/catch_test_macros.hpp>
#include "rclcpp/rclcpp.hpp"
#include "rover_comm.hpp"
#include "rover_comms_listener.hpp"
#include <unistd.h>

// rover_comms ROS2 integration tests:


void signalHandler(int signum){
    std::cout << "\nInterrupt signal (" << signum << ") received. Exiting..." << std::endl;
    exit(signum);
}

static bool heard_booga = false;
static int movement_messages = 0;
static int odometry_messages = 0;

class Callbacks : public cave_talk::ListenerCallbacks
{
public:
    void HearOogaBooga(const cave_talk::Say ooga_booga)
    {
        switch (ooga_booga)
        {
        case cave_talk::Say::SAY_OOGA:
            std::cout << "Heard ooga" << std::endl;
            break;
        case cave_talk::Say::SAY_BOOGA:
            std::cout << "Heard booga" << std::endl;
            heard_booga = true;
            break;
        default:
            break;
        }
    }
    void HearMovement(const CaveTalk_MetersPerSecond_t speed, const CaveTalk_RadiansPerSecond_t turn_rate) {
        movement_messages++;
    }
    void HearCameraMovement(const CaveTalk_Radian_t pan, const CaveTalk_Radian_t tilt) {}
    void HearLights(const bool headlights) {}
    void HearArm(const bool arm) {}
    void HearOdometry(const cave_talk::Imu &IMU, const cave_talk::Encoder &encoder_wheel_0, const cave_talk::Encoder &encoder_wheel_1, const cave_talk::Encoder &encoder_wheel_2, const cave_talk::Encoder &encoder_wheel_3)
    {
        // std::cout << "Heard odometry" << std::endl;
        odometry_messages++;
    }
    void HearLog(const char *const log) {}
    void HearMode(const bool manual){}
    void HearConfigServoWheels(const cave_talk::Servo &servo_wheel_0, const cave_talk::Servo &servo_wheel_1, const cave_talk::Servo &servo_wheel_2, const cave_talk::Servo &servo_wheel_3) {}
    void HearConfigServoCams(const cave_talk::Servo &servo_cam_pan, const cave_talk::Servo &servo_cam_tilt) {}
    void HearConfigMotor(const cave_talk::Motor &motor_wheel_0, const cave_talk::Motor &motor_wheel_1, const cave_talk::Motor &motor_wheel_2, const cave_talk::Motor &motor_wheel_3) {}
    void HearConfigEncoder(const cave_talk::ConfigEncoder &encoder_wheel_0, const cave_talk::ConfigEncoder &encoder_wheel_1, const cave_talk::ConfigEncoder &encoder_wheel_2, const cave_talk::ConfigEncoder &encoder_wheel_3) {}
};

/*
TEST_CASE_METHOD(RoverCommTest, "RoverComm init", "[rover]"){
    REQUIRE(rover_node != nullptr);
}
*/

TEST_CASE("Test OogaBooga"){
    cave_talk::Talker talker(cave_talk::send);
    cave_talk::Listener listener(cave_talk::receive, std::make_shared<Callbacks>());

    cave_talk::init();
    cave_talk::flush();
    talker.SpeakOogaBooga(cave_talk::Say::SAY_OOGA);

    for(int i=0; i<20; i++){
        listener.Listen();
        usleep(10000);  // .1s delay
    }

    REQUIRE(heard_booga == true);
}

TEST_CASE("Test Odom"){
    cave_talk::Talker talker(cave_talk::send);
    cave_talk::Listener listener(cave_talk::receive, std::make_shared<Callbacks>());

    cave_talk::init();
    cave_talk::flush();
    talker.SpeakOogaBooga(cave_talk::Say::SAY_OOGA);
    
    for(int i=0; i<20; i++){
        listener.Listen();
        usleep(10000);  // .1s delay
    }
    REQUIRE(heard_booga == true);

    for(int i=0; i<200; i++){
        talker.SpeakMovement(0.5, 0.1);
        listener.Listen();
        usleep(10000);
    }

    std::cout << "Heard " << odometry_messages << " odometry messages" << std::endl;
    REQUIRE(odometry_messages > 0); // require heard odom
}