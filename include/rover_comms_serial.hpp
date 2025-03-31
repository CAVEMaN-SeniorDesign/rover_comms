#ifndef ROVER_COMMS_SERIAL_HPP
#define ROVER_COMMS_SERIAL_HPP

#include <cstdint>
#include <memory>

#include "cave_talk.h"
#include "serial/serial.h"

namespace rover_comms
{

class CaveTalkSerialPort : public serial::Serial
{
    public:
        CaveTalkSerialPort(const std::string &port,
                           const uint32_t baudrate,
                           const serial::Timeout timeout,
                           const serial::bytesize_t bytesize,
                           const serial::parity_t parity,
                           const serial::stopbits_t stopbits,
                           const serial::flowcontrol_t flowcontrol);
        ~CaveTalkSerialPort(void);
        CaveTalk_Error_t Send(const void *const data, const size_t size);
        CaveTalk_Error_t Receive(void *const data, const size_t size, size_t *const bytes_received);
};

class TalkerWrapper : public cave_talk::Talker
{
    public:
        TalkerWrapper(std::shared_ptr<CaveTalkSerialPort> serial_port);

    private:
        CaveTalk_Error_t Send(const void *const data, const size_t size);
        CaveTalkSerialPort serial_port_;
};

class ListenerWrapper : public cave_talk::Listener
{
    public:
        ListenerWrapper(std::shared_ptr<CaveTalkSerialPort> serial_port, std::shared_ptr<cave_talk::ListenerCallbacks> listener_callbacks);

    private:
        CaveTalk_Error_t Receive(void *const data, const size_t size, size_t *const bytes_received);
        CaveTalkSerialPort serial_port_;
};

} // namespace rover_comms

#endif // ROVER_COMMS_SERIAL_HPP