#ifndef ROVER_COMMS_SERIAL_HPP
#define ROVER_COMMS_SERIAL_HPP

#include <cstdint>
#include <string>

#include "cave_talk.h"

namespace rover_comms
{

void SerialStart(const std::string &port, const uint32_t baudrate);
void SerialStop(void);
void SerialSetPort(const std::string &port);
void SerialSetBaudrate(const uint32_t baudrate);
CaveTalk_Error_t SerialSend(const void *const data, const size_t size);
CaveTalk_Error_t SerialReceive(void *const data, const size_t size, size_t *const bytes_received);

} // namespace rover_comms

#endif // ROVER_COMMS_SERIAL_HPP