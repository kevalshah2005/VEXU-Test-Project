#ifndef __COMETS_COMMS_UART__
#define __COMETS_COMMS_UART__

#ifndef PROS_USE_SIMPLE_NAMES
#define PROS_USE_SIMPLE_NAMES
#endif

#ifndef PROS_USE_LITERALS
#define PROS_USE_LITERALS
#endif

#include <cassert>
#include <string>
#include "pros/apix.h"
#include "comets/utils/file.h"


namespace comets
{
    /**
     * BareSerialDevice opens a comms smart port with a baudrate of 115200.
     * Lifetime is managed through unique_fd. It is undefined behavior to send
     * anything other than POD structs or basic objects (such as ints).
     * Be aware when sending data that items are little endian or big endian.
    */
    class BareSerialDevice
    {
    public:
        explicit BareSerialDevice(uint8_t port) {
            if (port < 0 || port > 20) {
                throw std::runtime_error("Port value out of range");
            }
            auto device_path = format_device_path(port);
            fd = unique_fd::open(device_path.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
        }

        auto read(void* data, size_t size) {
            return ::read(fd.get(), data, size);
        }

        auto write(void const* data, size_t size) {
            return ::write(fd.get(), data, size);
        }

    private:
        unique_fd fd;

        static auto format_device_path(uint8_t port) -> std::string {
            return std::string("/dev/") + std::to_string(port);
        }
    };

} // namespace comets

#endif
