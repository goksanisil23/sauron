#include "serialib.h"
#include <chrono>
#include <iostream>
#include <sstream>
#include <stdio.h>
#include <thread>
#include <unistd.h>
#include <vector>

namespace motor_comm
{
const char   *SERIAL_PORT{"/dev/ttyUSB0"};
constexpr int BAUD_RATE{921600};
constexpr int kSerialReadWriteTimeoutMs{1000};

serialib serial;

enum class Direction
{
    LEFT,
    RIGHT,
    UP,
    DOWN
};

bool readEncoder(int &cam_pos, int &base_pos)
{
    // Check if data is available
    if (serial.available() > 0)
    {
        char buffer[256];
        // Read until newline, up to 255 chars, timeout 1000ms
        int readResult = serial.readString(buffer, '\n', 255, kSerialReadWriteTimeoutMs);
        if (readResult > 0)
        {
            std::string response(buffer);

            // We expect something like: "cam_enc:123,base_enc:456\n"
            if (response.rfind("cam_enc:", 0) == 0 && response.back() == '\n')
            {
                // Strip trailing newline
                response.pop_back();

                // Split by comma => ["cam_enc:123", "base_enc:456"]
                auto commaPos = response.find(',');
                if (commaPos != std::string::npos)
                {
                    std::string cam_part  = response.substr(0, commaPos);
                    std::string base_part = response.substr(commaPos + 1);

                    // Parse out cam value
                    auto colon_pos_cam = cam_part.find(':');
                    if (colon_pos_cam != std::string::npos)
                    {
                        cam_pos = std::stoi(cam_part.substr(colon_pos_cam + 1));
                    }

                    // Parse out base value
                    auto colon_pos_base = base_part.find(':');
                    if (colon_pos_base != std::string::npos)
                    {
                        base_pos = std::stoi(base_part.substr(colon_pos_base + 1));
                    }
                    return true;
                }
            }
            else
            {
                // std::cout << "Received unexpected message: " << response << std::endl;
            }
        }
    }
    return false;
}

void write_serial_setpoint(int cam_val, int base_val)
{
    // serial.writeBytes((uint8_t *)"DIR:left\0", 9);
    // serial.writeBytes((uint8_t *)"VAL:cam=2200,base=1800\0", 23);
    std::string    str  = "VAL:cam=" + std::to_string(cam_val) + ",base=" + std::to_string(base_val);
    const uint8_t *data = reinterpret_cast<const uint8_t *>(str.c_str());
    serial.writeBytes(data, str.size() + 1); // +1 is for /0
}

void write_serial_direction(Direction dir)
{
    std::string str;
    switch (dir)
    {
    case Direction::LEFT:
        str = "DIR:left";
        break;
    case Direction::RIGHT:
        str = "DIR:right";
        break;
    case Direction::UP:
        str = "DIR:up";
        break;
    case Direction::DOWN:
        str = "DIR:down";
        break;
    default:
        break;
    }
    const uint8_t *data = reinterpret_cast<const uint8_t *>(str.c_str());
    serial.writeBytes(data, str.size() + 1); // +1 is for /0
}

void initSerial()
{
    char errorOpening = serial.openDevice(SERIAL_PORT, BAUD_RATE);
    if (errorOpening != 1)
    {
        std::cerr << "errorOpening: " << errorOpening << std::endl;
        exit(1);
    }
    std::cout << "Successful connection to " << SERIAL_PORT << std::endl;
}

void closeSerial()
{
    serial.closeDevice();
}

} // namespace motor_comm