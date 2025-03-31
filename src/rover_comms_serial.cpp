#include "rover_comms_serial.hpp"

#include <iostream>

#include "cave_talk.h"
#include "serial/serial.h"

namespace rover_comms
{

// default parameters, overwritten by the settings in src/Serial_Config.xml
std::string     port = "/dev/ttyUSB0";
unsigned long   baud = 1000000;
serial::Serial  new_serial;
serial::Timeout timeout = serial::Timeout::simpleTimeout(0);

// TODO move elsewhere
bool getSerialConfig(std::string file)
{
    tinyxml2::XMLDocument doc;
    tinyxml2::XMLError    error = doc.LoadFile(file.c_str());

    if (error != tinyxml2::XML_SUCCESS)
    {
        std::cout << "Serial Config error: " << file.c_str() << std::endl;
        return false;
    }

    tinyxml2::XMLElement *root = doc.FirstChildElement("Serial_Config");
    if (!root)
    {
        std::cerr << "Missing <Serial_Config> root element" << std::endl;
        return false;
    }

    tinyxml2::XMLElement *portElem = root->FirstChildElement("port");
    if (portElem && portElem->GetText())
    {
        port = portElem->GetText();
    }
    else
    {
        std::cerr << "Using default port: " << port << std::endl;
    }

    tinyxml2::XMLElement *baudElem = root->FirstChildElement("baud");
    if (baudElem && baudElem->GetText())
    {
        baud = std::stoul(baudElem->GetText());
    }
    else
    {
        std::cerr << "Using default baud: " << baud << std::endl;
    }

    return true;
}

// TODO move into constructor
CaveTalk_Error_t init()
{
    std::string config_path      = ament_index_cpp::get_package_share_directory("rover_comms") + "/src/Serial_Config.xml";
    bool        getConfigSuccess = getSerialConfig(config_path);

    try
    {
        new_serial.setPort(port);     // non-blocking essentially just 0s timeout
        new_serial.setBaudrate(baud);
        new_serial.setTimeout(timeout);
        new_serial.close();
        new_serial.open();

        if (new_serial.isOpen())
        {
            std::cout << "Serial port opened successfully.\n";
            return CAVE_TALK_ERROR_NONE;
        }
        else
        {
            std::cerr << "Failed to open serial port.\n";
            return CAVE_TALK_ERROR_SOCKET_CLOSED;
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << "Exception while initializing serial: " << e.what() << std::endl;
        return CAVE_TALK_ERROR_SOCKET_CLOSED;
    }
}

CaveTalkSerialPort::~CaveTalkSerialPort(void)
{
    if (isOpen())
    {
        flush();
        close();
    }
}

CaveTalk_Error_t CaveTalkSerialPort::Send(const void *const data, const size_t size)
{
    CaveTalk_Error_t error = CAVE_TALK_ERROR_SOCKET_CLOSED;

    if (isOpen())
    {
        try
        {
            if (write(static_cast<const uint8_t *>(data), size) < size)
            {
                error = CAVE_TALK_ERROR_INCOMPLETE;
            }
            else
            {
                error = CAVE_TALK_ERROR_NONE;
            }
        }
        catch (const serial::IOException &e)
        {
        }
    }

    return error;
}

CaveTalk_Error_t CaveTalkSerialPort::Receive(void *const data, const size_t size, size_t *const bytes_received)
{
    CaveTalk_Error_t error = CAVE_TALK_ERROR_SOCKET_CLOSED;

    if (isOpen())
    {
        try
        {
            *bytes_received = read(static_cast<const uint8_t *>(data), size);

            error = CAVE_TALK_ERROR_NONE;
        }
        catch (const serial::IOException &e)
        {
        }
    }

    return error;
}

} // namespace rover_comms