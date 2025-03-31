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
        CaveTalkSerialPort(const std::string &port   = "",
                           uint32_t baudrate         = 9600,
                           Timeout timeout           = Timeout(),
                           bytesize_t bytesize       = eightbits,
                           parity_t parity           = parity_none,
                           stopbits_t stopbits       = stopbits_one,
                           flowcontrol_t flowcontrol = flowcontrol_none);
        ~CaveTalkSerialPort(void);
        CaveTalk_Error_t Send(const void *const data, const size_t size);
        CaveTalk_Error_t Receive(void *const data, const size_t size, size_t *const bytes_received);
};

} // namespace rover_comms

#endif // ROVER_COMMS_SERIAL_HPP