#ifndef ROVER_COMMS_SERIAL_HPP
#define ROVER_COMMS_SERIAL_HPP

#include <cstdint>

#include "cave_talk.h"
#include "serial/serial.h"

namespace rover_comms
{

class CaveTalkSerialPort : public serial::Serial
{
    public:
        CaveTalkSerialPort(const std::string &port           = "",
                           uint32_t baudrate                 = 1e6,
                           serial::Timeout timeout           = serial::Timeout::simpleTimeout(0),
                           serial::bytesize_t bytesize       = serial::eightbits,
                           serial::parity_t parity           = serial::parity_none,
                           serial::stopbits_t stopbits       = serial::stopbits_one,
                           serial::flowcontrol_t flowcontrol = serial::flowcontrol_none);
        ~CaveTalkSerialPort(void);
        CaveTalk_Error_t Send(const void *const data, const size_t size);
        CaveTalk_Error_t Receive(void *const data, const size_t size, size_t *const bytes_received);
};

} // namespace rover_comms

#endif // ROVER_COMMS_SERIAL_HPP