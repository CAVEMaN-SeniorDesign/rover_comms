#define CATCH_CONFIG_MAIN
#include "serial/serial.h"
#include <catch2/catch_test_macros.hpp>
#include "cave_talk_link.h"
#include <string>
#include <iostream>
#include <cstdio>
#include <cstring>
#include <vector>

using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;
using std::vector;


namespace new_serial{

    std::string port = "/dev/ttyUSB0";
    unsigned long baud = 1000000;
    serial::Serial new_serial;
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
            new_serial.setPort(port); // non-blocking essentially just 0s timeout
            new_serial.setBaudrate(baud);
            new_serial.setTimeout(timeout);
            new_serial.open();

            if (new_serial.isOpen()) {
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
            new_serial.close();
            return CAVE_TALK_ERROR_NONE;
        }
        catch (const std::exception& e) {
            cerr << "Exception while deinitializing serial: " << e.what() << endl;
            return CAVE_TALK_ERROR_SOCKET_CLOSED;
        }
    }

    CaveTalk_Error_t flush(){
        try{
            new_serial.flush();
            return CAVE_TALK_ERROR_NONE;
        }
        catch (const std::exception& e) {
            cerr << "Exception while flushing serial: " << e.what() << endl;
            return CAVE_TALK_ERROR_SOCKET_CLOSED;
        }
    }

    CaveTalk_Error_t send(const void *const data, const size_t size){
        CaveTalk_Error_t error = CAVE_TALK_ERROR_NONE;

        if(new_serial.isOpen()){
            try {
                size_t bytes_written = new_serial.write(static_cast<const uint8_t*>(data), size);
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

        if(new_serial.isOpen()){
            try {
                // read returns a std::string, so we need to convert
                std::string result = new_serial.read(size);
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

} //end new_serial namespace


TEST_CASE("Test serial init"){
    CaveTalk_Error_t error = new_serial::init();
    REQUIRE(error == CAVE_TALK_ERROR_NONE);
}


