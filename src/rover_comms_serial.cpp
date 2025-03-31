#include "rover_comms_serial.hpp"

#include <cstdint>
#include <iostream>
#include <string>

#include "cave_talk.h"
#include "serial/serial.h"

namespace rover_comms
{

// TODO move elsewhere
// bool getSerialConfig(std::string file)
// {
//     tinyxml2::XMLDocument doc;
//     tinyxml2::XMLError    error = doc.LoadFile(file.c_str());

//     if (error != tinyxml2::XML_SUCCESS)
//     {
//         std::cout << "Serial Config error: " << file.c_str() << std::endl;
//         return false;
//     }

//     tinyxml2::XMLElement *root = doc.FirstChildElement("Serial_Config");
//     if (!root)
//     {
//         std::cerr << "Missing <Serial_Config> root element" << std::endl;
//         return false;
//     }

//     tinyxml2::XMLElement *portElem = root->FirstChildElement("port");
//     if (portElem && portElem->GetText())
//     {
//         port = portElem->GetText();
//     }
//     else
//     {
//         std::cerr << "Using default port: " << port << std::endl;
//     }

//     tinyxml2::XMLElement *baudElem = root->FirstChildElement("baud");
//     if (baudElem && baudElem->GetText())
//     {
//         baud = std::stoul(baudElem->GetText());
//     }
//     else
//     {
//         std::cerr << "Using default baud: " << baud << std::endl;
//     }

//     return true;
// }

static const uint32_t kDefaultBaudrate = 1000000U;
static serial::Serial serial_port("",
                                  kDefaultBaudrate,
                                  serial::Timeout::simpleTimeout(0),
                                  serial::eightbits,
                                  serial::parity_none,
                                  serial::stopbits_one,
                                  serial::flowcontrol_none);

void SerialStart(const std::string &port, const uint32_t baudrate)
{
    try
    {
        SerialSetPort(port);
        SerialSetBaudrate(baudrate);
        serial_port.open();

        if (serial_port.isOpen())
        {
            serial_port.flush();

            std::cout << "Serial port opened successfully." << std::endl;
        }
        else
        {
            std::cerr << "Failed to open serial port." << std::endl;
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << "Exception while initializing serial: " << e.what() << std::endl;
    }
}

void SerialStop(void)
{
    std::cout << "DESTRUCTOR CALLED" << std::endl;

    if (serial_port.isOpen())
    {
        serial_port.flush();
        serial_port.close();
    }
}

void SerialSetPort(const std::string &port)
{
    serial_port.setPort(port);

    if (serial_port.isOpen())
    {
        serial_port.flush();
    }
}

void SerialSetBaudrate(const uint32_t baudrate)
{
    serial_port.setBaudrate(baudrate);
}

CaveTalk_Error_t SerialSend(const void *const data, const size_t size)
{
    CaveTalk_Error_t error = CAVE_TALK_ERROR_SOCKET_CLOSED;

    if (serial_port.isOpen())
    {
        try
        {
            if (serial_port.write(static_cast<const uint8_t *>(data), size) < size)
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

CaveTalk_Error_t SerialReceive(void *const data, const size_t size, size_t *const bytes_received)
{
    CaveTalk_Error_t error = CAVE_TALK_ERROR_SOCKET_CLOSED;

    if (serial_port.isOpen())
    {
        try
        {
            *bytes_received = serial_port.read(static_cast<uint8_t *const>(data), size);

            error = CAVE_TALK_ERROR_NONE;
        }
        catch (const serial::IOException &e)
        {
        }
    }

    return error;
}

} // namespace rover_comms