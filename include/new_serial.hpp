#ifndef NEW_SERIAL_HPP
#define NEW_SERIAL_HPP

#include "serial/serial.h"
#include "cave_talk_link.h"
#include <string>
#include <iostream>
#include <cstdio>
#include <cstring>
#include <vector>
#include "cave_talk.h"
#include "tinyxml2.h"

#include "ament_index_cpp/get_package_share_directory.hpp"



namespace new_serial {

    // Default serial parameters (can be overwritten by XML config)
    extern std::string port;
    extern unsigned long baud;
    extern serial::Serial new_serial;
    extern serial::Timeout timeout;
    extern int stopbit;
    
    bool getSerialConfig(std::string file);
    void enumerate_ports();
    void print_usage();
    CaveTalk_Error_t init();
    CaveTalk_Error_t deinit();
    CaveTalk_Error_t flush();
    CaveTalk_Error_t send(const void *const data, const size_t size);
    CaveTalk_Error_t receive(void *const data, const size_t size, size_t *const bytes_received);
    
    } // namespace new_serial
    
    std::string CaveTalk_ErrorToString(CaveTalk_Error_t error);
    
#endif // NEW_SERIAL_HPP