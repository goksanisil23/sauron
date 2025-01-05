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
constexpr int kBaseServoLeftLimit{1200};
constexpr int kBaseServoRightLimit{2700};

constexpr int kCamServoId{1};
constexpr int kBaseServoId{2};

constexpr unsigned long kMotorSerialBaudRate{1000000};
constexpr unsigned long kPiSerialBaudRate{921600};

constexpr int kServoIncrement{50};

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

} // namespace

extern "C" void app_main(void)
{
    uint8_t receive_buffer[ESP32Serial::kUartBufferSize];
    // uint8_t send_buffer[ESP32Serial::kUartBufferSize + 32];
    char message[ESP32Serial::kUartBufferSize];

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
            int base_servo_increment = 0;
            int cam_servo_increment  = 0;

            // ---------- Pi communication ---------
            int bytes_read = esp_pi_serial.read(receive_buffer, sizeof(receive_buffer));
            if (bytes_read > 0)
            {
                if (receive_buffer[bytes_read - 1] == '\0')
                {
                    memcpy(message, receive_buffer, bytes_read);
                    const char *receive_buf_char = reinterpret_cast<char *>(receive_buffer);
                    if (strncmp(receive_buf_char, "right\0", bytes_read) == 0)
                    {
                        base_servo_increment = kServoIncrement;
                    }
                    else if (strncmp(receive_buf_char, "left\0", bytes_read) == 0)
                    {
                        base_servo_increment = -kServoIncrement;
                    }
                    else if (strncmp(receive_buf_char, "up\0", bytes_read) == 0)
                    {
                        cam_servo_increment = -kServoIncrement;
                    }
                    else if (strncmp(receive_buf_char, "down\0", bytes_read) == 0)
                    {
                        cam_servo_increment = kServoIncrement;
                    }
                    // esp_pi_serial.flush();
                }
            }

            // ---------- Motor control -----------
            base_pos = motor_control.ReadPos(kBaseServoId);
            cam_pos  = motor_control.ReadPos(kCamServoId);

            if ((base_pos != -1))
            {
                base_pos_to_send = base_pos;
                // printf("[ESP] base servo position: %d\n", base_pos);
                if (isWithinHardLimits(kBaseServoId, base_pos + base_servo_increment))
                {
                    // Move the motor based on the message received from pi
                    motor_control.WritePosEx(kBaseServoId, base_pos + base_servo_increment, 2000, 100);
                }
            }
            else
            {
                printf("base encoder read position err\n");
            }
            if ((cam_pos != -1))
            {
                cam_pos_to_send = cam_pos;
                // printf("[ESP] cam servo position: %d\n", cam_pos);
                if (isWithinHardLimits(kCamServoId, cam_pos + cam_servo_increment))
                {
                    // Move the motor based on the message received from pi
                    motor_control.WritePosEx(kCamServoId, cam_pos + cam_servo_increment, 2000, 100);
                }
            }
            else
            {
                printf("cam encoder read position err\n");
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