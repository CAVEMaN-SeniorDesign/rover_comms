#include <iostream>
#include <memory>

#include "jetson_cave_talk.h"
#include "cave_talk.h"
#include "ooga_booga.pb.h"

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
        case cave_talk::SAY_OOGA:
            std::cout << "Heard ooga" << std::endl;
            break;
        case cave_talk::SAY_BOOGA:
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
    void HearConfigLog(const cave_talk::LogLevel log_level) {}
};

std::string CaveTalk_ErrorToString(CaveTalk_Error_t error) {
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

int main(int argc, char *argv[])
{
    cave_talk::Listener listener(cave_talk::receive, std::make_shared<Callbacks>());
    cave_talk::Talker talker(cave_talk::send);

    cave_talk::init();
    cave_talk::flush();
    talker.SpeakOogaBooga(cave_talk::Say::SAY_OOGA);

    while (!heard_booga)
    {    
        CaveTalk_Error_t error = listener.Listen();
        if (CAVE_TALK_ERROR_NONE != error)
        {
            // std::string error_str = CaveTalk_ErrorToString(error);
            // std::cout << "Error line 79 " << error_str << std::endl;
        }
    }

    auto start = std::chrono::high_resolution_clock::now();
    auto start_odom = std::chrono::high_resolution_clock::now();
    while (true)
    {
        CaveTalk_Error_t error = listener.Listen();
        if (CAVE_TALK_ERROR_NONE != error)
        {
            switch (error)
            {
            case CAVE_TALK_ERROR_SOCKET_CLOSED:
                std::cout << "Socket closed" << std::endl;
                break;
            default:
                std::string error_str = CaveTalk_ErrorToString(error);
                //std::cout << "Error line 97 " << error_str << std::endl;
                //std::cout << "Error line 98 " << error_str << std::endl;
                break;
            }
        }

        if (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start) >= std::chrono::milliseconds(5))
        {
            talker.SpeakMovement(0.5, 0.1);
            start = std::chrono::high_resolution_clock::now();
        }

        if (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start_odom) >= std::chrono::milliseconds(1000))
        {
            std::cout << "in odom print" << std::endl;
            std::cout << "Heard " << odometry_messages << " odometry messages" << std::endl;
            odometry_messages = 0;
            std::cout << "Heard " << movement_messages << " movement messages" << std::endl;
            movement_messages = 0;
            start_odom = std::chrono::high_resolution_clock::now();
        }
    }

    return 0;
}