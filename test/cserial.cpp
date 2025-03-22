#define CATCH_CONFIG_MAIN
#include "serial/serial.h"
#include <catch2/catch_test_macros.hpp>
#include "cave_talk_link.h"
#include <string>
#include <iostream>
#include <cstdio>
#include <cstring>
#include <vector>
#include "cave_talk.h"
#include <thread>

using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;
using std::vector;

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
};


namespace new_serial_test{

    std::string port = "/dev/ttyTHS1";
    unsigned long baud = 1000000;
    serial::Serial new_serial_test;
    serial::Timeout timeout = serial::Timeout::simpleTimeout(0);

    void enumerate_ports()
    {
        vector<serial::PortInfo> devices_found = serial::list_ports();
    
        vector<serial::PortInfo>::iterator iter = devices_found.begin();
    
        while( iter != devices_found.end() )
        {
            serial::PortInfo device = *iter++;
    
            printf( "(%s, %s, %s)\n", device.port.c_str(), device.description.c_str(),
         device.hardware_id.c_str() );
        }
    }
    
    void print_usage()
    {
        cerr << "Usage: test_serial {-e|<serial port address>} ";
        cerr << "<baudrate> [test string]" << endl;
    }
    
    CaveTalk_Error_t init(){
        try{
            new_serial_test.setPort(port); // non-blocking essentially just 0s timeout
            new_serial_test.setBaudrate(baud);
            new_serial_test.setTimeout(timeout);
            new_serial_test.close();
            new_serial_test.open();

            if (new_serial_test.isOpen()) {
                cout << "Serial port opened successfully.\n";
                return CAVE_TALK_ERROR_NONE;
            } else {
                cerr << "Failed to open serial port.\n";
                return CAVE_TALK_ERROR_SOCKET_CLOSED;
            }

        }
        catch (const std::exception& e) {
            cerr << "Exception while initializing serial: " << e.what() << endl;
            return CAVE_TALK_ERROR_SOCKET_CLOSED;
        }
    }

    CaveTalk_Error_t deinit(){
        try{
            new_serial_test.close();
            return CAVE_TALK_ERROR_NONE;
        }
        catch (const std::exception& e) {
            cerr << "Exception while deinitializing serial: " << e.what() << endl;
            return CAVE_TALK_ERROR_SOCKET_CLOSED;
        }
    }

    CaveTalk_Error_t flush(){
        try{
            new_serial_test.flush();
            return CAVE_TALK_ERROR_NONE;
        }
        catch (const std::exception& e) {
            cerr << "Exception while flushing serial: " << e.what() << endl;
            return CAVE_TALK_ERROR_SOCKET_CLOSED;
        }
    }

    CaveTalk_Error_t send(const void *const data, const size_t size){
        CaveTalk_Error_t error = CAVE_TALK_ERROR_NONE;

        if(new_serial_test.isOpen()){
            try {
                size_t bytes_written = new_serial_test.write(static_cast<const uint8_t*>(data), size);
            }
            catch (const serial::IOException& e) {
                error = CAVE_TALK_ERROR_SOCKET_CLOSED;
            } 
        }
        else{
            error = CAVE_TALK_ERROR_SOCKET_CLOSED;
        }

    return error;
    }

    CaveTalk_Error_t receive(void *const data, const size_t size, size_t *const bytes_received){
        CaveTalk_Error_t error = CAVE_TALK_ERROR_NONE;

        if(new_serial_test.isOpen()){
            try {
                // read returns a std::string, so we need to convert
                std::string result = new_serial_test.read(size);
                *bytes_received = result.size();

                // Need to copy to the buffer
                memcpy(data, result.data(), *bytes_received);
            } 
            catch (const serial::IOException& e) {
                error = CAVE_TALK_ERROR_INCOMPLETE;
            } 
            catch (...) {
                error = CAVE_TALK_ERROR_INCOMPLETE;
            }
        }
        else{
            error = CAVE_TALK_ERROR_SOCKET_CLOSED;
        }

        return error;
    }

} //end new_serial_test namespace

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



TEST_CASE("Test OogaBooga"){
    cave_talk::Talker talker(new_serial_test::send);
    cave_talk::Listener listener(new_serial_test::receive, std::make_shared<Callbacks>());

    new_serial_test::init();
    new_serial_test::flush();
    auto start = std::chrono::high_resolution_clock::now();
    talker.SpeakOogaBooga(cave_talk::SAY_OOGA);
    while(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start) < std::chrono::milliseconds(3000)){
        CaveTalk_Error_t error =listener.Listen();
        if (error != CAVE_TALK_ERROR_NONE){
            std::string error_str = CaveTalk_ErrorToString(error);
            std::cout << "Listen error line 214 " << error_str << std::endl;
        }
    }

    REQUIRE(heard_booga == true);
}

TEST_CASE("Test Odom"){
    cave_talk::Talker talker(new_serial_test::send);
    cave_talk::Listener listener(new_serial_test::receive, std::make_shared<Callbacks>());

    new_serial_test::init();
    new_serial_test::flush();

    auto start = std::chrono::high_resolution_clock::now();
    talker.SpeakOogaBooga(cave_talk::SAY_OOGA);
    while(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start) < std::chrono::milliseconds(3000)){
        listener.Listen();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    REQUIRE(heard_booga == true);

    start = std::chrono::high_resolution_clock::now();
    while(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start) < std::chrono::milliseconds(20000)){
        talker.SpeakMovement(0.5, 0.1);
        listener.Listen();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    std::cout << "Heard " << odometry_messages << " odometry messages" << std::endl;

    // start = std::chrono::high_resolution_clock::now();
    // while(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start) < std::chrono::milliseconds(50000)){
    //     talker.SpeakOogaBooga(cave_talk::SAY_OOGA);
    //     listener.Listen();
    //     std::this_thread::sleep_for(std::chrono::milliseconds(10));
    // }

    REQUIRE(odometry_messages > 0); // require heard odom
}
