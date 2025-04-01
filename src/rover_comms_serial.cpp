#include "rover_comms_serial.hpp"

#include <cstdint>
#include <iostream>
#include <string>

#include "cave_talk.h"
#include "serial/serial.h"

namespace rover_comms_serial
{

static const uint32_t kDefaultBaudrate = 1000000U;
static serial::Serial serial_port("",
                                  kDefaultBaudrate,
                                  serial::Timeout::simpleTimeout(0),
                                  serial::eightbits,
                                  serial::parity_none,
                                  serial::stopbits_one,
                                  serial::flowcontrol_none);

void Start(const std::string &port, const uint32_t baudrate)
{
    try
    {
        SetPort(port);
        SetBaudrate(baudrate);
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

void Stop(void)
{
    if (serial_port.isOpen())
    {
        serial_port.flush();
        serial_port.close();
    }
}

void SetPort(const std::string &port)
{
    serial_port.setPort(port);

    if (serial_port.isOpen())
    {
        serial_port.flush();
    }
}

void SetBaudrate(const uint32_t baudrate)
{
    serial_port.setBaudrate(baudrate);
}

CaveTalk_Error_t Send(const void *const data, const size_t size)
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

CaveTalk_Error_t Receive(void *const data, const size_t size, size_t *const bytes_received)
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

} // namespace rover_comms_serial