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
constexpr unsigned long kPiSerialBaudRate{115200};

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
    uint8_t send_buffer[ESP32Serial::kUartBufferSize + 32];
    char    message[ESP32Serial::kUartBufferSize];

    int cam_pos, base_pos;

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
            // ---------- Pi communication ---------
            int bytes_read = esp_pi_serial.read(receive_buffer, sizeof(receive_buffer));
            if (bytes_read > 0)
            {
                // memcpy(send_buffer, receive_buffer, bytes_read);
                // uint8_t *int_ptr = reinterpret_cast<uint8_t *>(&ctr);
                // memcpy(send_buffer + bytes_read, int_ptr, sizeof(ctr));
                // int ret = esp_pi_serial.write(send_buffer, sizeof(ctr) + bytes_read);
                // if (ret == -1)
                // {
                //     printf("[ESP] write error\n");
                // }

                memcpy(message, receive_buffer, bytes_read);
                // message[bytes_read] = '\0'; // Null-terminate the string
                printf("[ESP] Received %d bytes: %s\n", bytes_read, message);
                int ret = esp_pi_serial.write((uint8_t *)message, bytes_read);
                if (ret == -1)
                {
                    printf("[ESP] write error\n");
                }
            }
            else
            {
                printf("[ESP] No data %u\n", ctr);
            }

            // ---------- Motor control -----------
            base_pos = motor_control.ReadPos(kBaseServoId);
            if (base_pos != -1)
            {
                printf("base servo position: %d\n", base_pos);

                if (isWithinHardLimits(kBaseServoId, base_pos))
                {
                    // Move the motor based on the message received from pi

                    motor_control.WritePosEx(kBaseServoId, base_pos + 10, 2000, 100);
                }

                vTaskDelay(100 / portTICK_PERIOD_MS);
            }
            else
            {
                printf("read position err\n");
                vTaskDelay(1000 / portTICK_PERIOD_MS);
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