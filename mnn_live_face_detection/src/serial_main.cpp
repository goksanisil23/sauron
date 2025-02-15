#include "MotorCommunication.hpp"

int main(/*int argc, char *argv[]*/)
{
    motor_comm::initSerial();

    int inc = 0;
    while (true)
    {
        int camPos, basePos;
        if (motor_comm::readEncoder(camPos, basePos))
        {
            std::cout << "cam = " << camPos << ", base = " << basePos << std::endl;
        }
        if (inc % 10 == 0)
            // motor_comm::write_serial_setpoint(2000 + inc, 2000 + inc);
            motor_comm::write_serial_direction(motor_comm::Direction::DOWN);
        inc++;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    // Close the serial device
    motor_comm::closeSerial();

    return 0;
}