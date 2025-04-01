#ifndef ROVER_COMMS_SERIAL_HPP
#define ROVER_COMMS_SERIAL_HPP

#include <cstdint>
#include <string>

#include "cave_talk.h"

namespace rover_comms_serial
{

void Start(const std::string &port, const uint32_t baudrate);
void Stop(void);
void SetPort(const std::string &port);
void SetBaudrate(const uint32_t baudrate);
CaveTalk_Error_t Send(const void *const data, const size_t size);
CaveTalk_Error_t Receive(void *const data, const size_t size, size_t *const bytes_received);

} // namespace rover_comms_serial

#endif // ROVER_COMMS_SERIAL_HPP