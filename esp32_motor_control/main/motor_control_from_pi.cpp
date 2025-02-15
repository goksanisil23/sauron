#include "esp_chip_info.h"
#include "esp_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include <inttypes.h>
#include <stdio.h>

#include "SCServo.h"

#include <cassert>
#include <chrono>
#include <iostream>
#include <string.h>
#include <thread>

#include "esp_exception.hpp"
#include "esp_timer_cxx.hpp"

using namespace std;
using namespace idf;
using namespace idf::esp_timer;

// the uart used to control servos.
// GPIO 18 - S_RXD, GPIO 19 - S_TXD, as default.
#define S_RXD_MOTOR_SERIAL 18
#define S_TXD_MOTOR_SERIAL 19
#define MOTOR_UART UART_NUM_1
#define PI_UART UART_NUM_0

// Hard limits for the cameras after homing with st.CalibrationOfs(1 & 2) :
constexpr int kCamServoUpLimit{1600};
constexpr int kCamServoDownLimit{2300};
// assuming forward of the robot is looking opposite to usb ports
constexpr int kBaseServoLeftLimit{1000};
constexpr int kBaseServoRightLimit{3000};

constexpr int kCamServoId{1};
constexpr int kBaseServoId{2};

constexpr unsigned long kMotorSerialBaudRate{1'000'000};
constexpr unsigned long kPiSerialBaudRate{921'600};

constexpr int kServoIncrement{20};
constexpr int kServoAccel{100};
constexpr int kServoSpeed{500};

enum class ServoDirectionCmd
{
    kInvalid,
    kLeft,
    kRight,
    kUp,
    kDown
};

struct ServoSetpoints
{
    int cam_pos{-1};
    int base_pos{-1};
};

namespace
{
// If the given servo is within hard limits
bool isWithinHardLimits(const int servo_id, const int servo_pos)
{
    switch (servo_id)
    {
    case kCamServoId:
    {
        return (servo_pos > kCamServoUpLimit) && (servo_pos < kCamServoDownLimit);
    }
    case kBaseServoId:
    {
        return (servo_pos > kBaseServoLeftLimit) && (servo_pos < kBaseServoRightLimit);
    }
    default:
    {
        assert(false && "Unhandled servo motor id");
        return false;
    }
    }
}

// Check if the incoming serial message is a direction message, and if so, return the type of direction
ServoDirectionCmd checkDirMessage(const uint8_t *buffer, const int bytes_read)
{
    const char *buffer_str = reinterpret_cast<const char *>(buffer);
    printf("[ESP] Received: %s\n", buffer_str);

    // Check if the message starts with "DIR:"
    if ((bytes_read < 4) || (strncmp(buffer_str, "DIR:", 4) != 0))
    {
        return ServoDirectionCmd::kInvalid;
    }
    if (buffer[bytes_read - 1] == '\0')
    {
        if (strncmp(buffer_str, "DIR:right\0", bytes_read) == 0)
        {
            return ServoDirectionCmd::kRight;
        }
        else if (strncmp(buffer_str, "DIR:left\0", bytes_read) == 0)
        {
            return ServoDirectionCmd::kLeft;
        }
        else if (strncmp(buffer_str, "DIR:up\0", bytes_read) == 0)
        {
            return ServoDirectionCmd::kUp;
        }
        else if (strncmp(buffer_str, "DIR:down\0", bytes_read) == 0)
        {
            return ServoDirectionCmd::kDown;
        }
        else
        {
            return ServoDirectionCmd::kInvalid;
        }
    }
    return ServoDirectionCmd::kInvalid;
}

// Check if incoming serial message is a value message, and if so, set the servo setpoints
ServoSetpoints checkValMessage(const uint8_t *buffer, const int bytes_read)
{
    const char *buffer_str = reinterpret_cast<const char *>(buffer);
    // Check if the message starts with "VAL:"
    if ((bytes_read < 4) || (strncmp(buffer_str, "VAL:", 4) != 0))
    {
        return {};
    }
    if (buffer[bytes_read - 1] == '\0')
    {
        int cam_pos{-1};
        int base_pos{-1};
        sscanf(buffer_str, "VAL:cam=%d,base=%d", &cam_pos, &base_pos);
        printf("[ESP] cam setpoint: %d, base setpoint: %d\n", cam_pos, base_pos);
        return {cam_pos, base_pos};
    }
    return {};
}

} // namespace

extern "C" void app_main(void)
{
    uint8_t receive_buffer[ESP32Serial::kUartBufferSize];
    char    message[ESP32Serial::kUartBufferSize];

    int cam_pos, base_pos;
    int cam_pos_to_send = 0, base_pos_to_send = 0;

    SMS_STS     motor_control;
    ESP32Serial motor_control_serial(MOTOR_UART);
    ESP32Serial esp_pi_serial(PI_UART);

    try
    {
        motor_control_serial.begin(kMotorSerialBaudRate, S_RXD_MOTOR_SERIAL, S_TXD_MOTOR_SERIAL);
        motor_control.pSerial = &motor_control_serial;
        esp_pi_serial.begin(kPiSerialBaudRate);

        int ctr{0};
        while (true)
        {
            int               base_servo_increment = 0;
            int               cam_servo_increment  = 0;
            ServoDirectionCmd dir_cmd{ServoDirectionCmd::kInvalid};
            ServoSetpoints    setpoints{};

            // ---------- Pi communication ---------
            int bytes_read = esp_pi_serial.read(receive_buffer, sizeof(receive_buffer));
            if (bytes_read > 0)
            {
                dir_cmd   = checkDirMessage(receive_buffer, bytes_read);
                setpoints = checkValMessage(receive_buffer, bytes_read);
                switch (dir_cmd)
                {
                case ServoDirectionCmd::kRight:
                {
                    base_servo_increment = kServoIncrement;
                    break;
                }
                case ServoDirectionCmd::kLeft:
                {
                    base_servo_increment = -kServoIncrement;
                    break;
                }
                case ServoDirectionCmd::kUp:
                {
                    cam_servo_increment = -kServoIncrement;
                    break;
                }
                case ServoDirectionCmd::kDown:
                {
                    cam_servo_increment = kServoIncrement;
                    break;
                }
                case ServoDirectionCmd::kInvalid:
                {
                    break;
                }
                }
            }

            // ---------- Motor control -----------
            base_pos = motor_control.ReadPos(kBaseServoId);
            cam_pos  = motor_control.ReadPos(kCamServoId);
            const bool is_dir_mode{dir_cmd != ServoDirectionCmd::kInvalid};
            const bool is_val_mode{(setpoints.cam_pos != -1) && (setpoints.base_pos != -1)};

            if ((base_pos != -1))
            {
                base_pos_to_send = base_pos;
                if (is_dir_mode && isWithinHardLimits(kBaseServoId, base_pos + base_servo_increment))
                {
                    // Move the motor based on the message received from pi
                    motor_control.WritePosEx(kBaseServoId, base_pos + base_servo_increment, kServoSpeed, kServoAccel);
                }
            }
            else
            {
                printf("base encoder read position err\n");
            }
            if ((cam_pos != -1))
            {
                cam_pos_to_send = cam_pos;
                if (is_dir_mode && isWithinHardLimits(kCamServoId, cam_pos + cam_servo_increment))
                {
                    // Move the motor based on the message received from pi
                    motor_control.WritePosEx(kCamServoId, cam_pos + cam_servo_increment, kServoSpeed, kServoAccel);
                }
            }
            else
            {
                printf("cam encoder read position err\n");
            }

            if (is_val_mode)
            {
                if (isWithinHardLimits(kBaseServoId, setpoints.base_pos))
                {
                    motor_control.WritePosEx(kBaseServoId, setpoints.base_pos, kServoSpeed, kServoAccel);
                }
                if (isWithinHardLimits(kCamServoId, setpoints.cam_pos))
                {
                    motor_control.WritePosEx(kCamServoId, setpoints.cam_pos, kServoSpeed, kServoAccel);
                }
            }

            // Write both cam and base servo positions to pi
            sprintf(message, "cam_enc:%d,base_enc:%d\n", cam_pos_to_send, base_pos_to_send);
            int ret = esp_pi_serial.write((uint8_t *)message, strlen(message));
            esp_pi_serial.flush();
            if (ret == -1)
            {
                printf("[ESP] write error\n");
            }

            ctr++;
            vTaskDelay(100 / portTICK_PERIOD_MS);
        }

        printf("Setting up timer to trigger in 500ms\n");
        ESPTimer timer([]() { printf("timeout\n"); });
        timer.start(chrono::microseconds(200 * 1000));

        this_thread::sleep_for(std::chrono::milliseconds(550));

        printf("Setting up timer to trigger periodically every 200ms\n");
        ESPTimer timer2([]() { printf("periodic timeout\n"); });
        timer2.start_periodic(chrono::microseconds(200 * 1000));

        this_thread::sleep_for(std::chrono::milliseconds(1050));
    }
    catch (const ESPException &e)
    {
        printf("Exception with error: %d\n", e.error);
    }
    printf("Finished\n");
}