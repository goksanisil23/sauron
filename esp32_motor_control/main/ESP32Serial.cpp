#include "ESP32Serial.h"

#include <cstring>
#include <string.h>

#include <driver/uart.h>

namespace
{
constexpr int kReadTimeoutMs = 100;
}

ESP32Serial::ESP32Serial(int port) : m_port((uart_port_t)port)
{
}

ESP32Serial::~ESP32Serial()
{
}

int ESP32Serial::begin(unsigned long baud)
{
    uart_config_t uart_config = {.baud_rate = (int)baud,
                                 .data_bits = UART_DATA_8_BITS,
                                 .parity    = UART_PARITY_DISABLE,
                                 .stop_bits = UART_STOP_BITS_1,
                                 .flow_ctrl = UART_HW_FLOWCTRL_DISABLE};
    if (uart_param_config(m_port, &uart_config) != ESP_OK)
    {
        return -1;
    }

    if (uart_driver_install(m_port, kUartBufferSize * 2, 0, 0, NULL, 0) != ESP_OK)
    {
        return -3;
    }

    return 0;
}

int ESP32Serial::begin(unsigned long baud, int8_t rxPin, int8_t txPin)
{

    /* Configure parameters of an UART driver,
   * communication pins and install the driver */
    // Values below are to mimic SERIAL_8N1 setting in arduino which means
    // ( data bits, no parity, 1 stop bit)
    uart_config_t uart_config = {.baud_rate = (int)baud,
                                 .data_bits = UART_DATA_8_BITS,
                                 .parity    = UART_PARITY_DISABLE,
                                 .stop_bits = UART_STOP_BITS_1,
                                 .flow_ctrl = UART_HW_FLOWCTRL_DISABLE};
    if (uart_param_config(m_port, &uart_config) != ESP_OK)
    {
        return -1;
    }

    //Set UART pins (using UART0 default pins ie no changes.)
    if (uart_set_pin(m_port, txPin, rxPin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) != ESP_OK)
    {
        return -2;
    }
    //Install UART driver, and get the queue.
    const int uart_buffer_size = (1024 * 2);
    const int uart_queue_size  = 20;
    if (uart_driver_install(m_port, uart_buffer_size, uart_buffer_size, uart_queue_size, &m_uart_queue, 0) != ESP_OK)
    {
        return -3;
    }

    return 0;
}

int ESP32Serial::available()
{
    size_t result;
    if (uart_get_buffered_data_len(m_port, &result) != ESP_OK)
    {
    }
    return (int)result;
}

void ESP32Serial::flush()
{
    if (uart_flush(m_port) != ESP_OK)
    {
    }
}

void ESP32Serial::end()
{
    if (uart_driver_delete(m_port) != ESP_OK)
    {
    }
}

int ESP32Serial::read(void)
{
    uint8_t data[1];
    if (available() == 0)
    {
        return -1;
    }
    uart_read_bytes(m_port, &data[0], 1, 100);
    return (int)(data[0]);
}

int ESP32Serial::read(uint8_t *buffer, const size_t buffer_size)
{
    size_t available_bytes{0U};
    if (uart_get_buffered_data_len(m_port, &available_bytes) != ESP_OK)
    {
        return -1;
    }
    if (available_bytes > 0)
    {
        size_t bytes_to_read = (available_bytes > buffer_size) ? buffer_size : available_bytes;
        int    bytes_read    = uart_read_bytes(m_port, buffer, bytes_to_read, pdMS_TO_TICKS(kReadTimeoutMs));

        if (bytes_read < 0)
        {
            printf("Error reading UART data\n");
            return -1;
        }

        return bytes_read; // Return the number of bytes actually read
    }

    return 0;
}

size_t ESP32Serial::write(uint8_t c)
{
    size_t retsize = uart_write_bytes(m_port, (const char *)&c, 1);
    if (retsize == -1)
    {
    }
    return retsize;
}

size_t ESP32Serial::write(const uint8_t *buffer, size_t size)
{
    size_t retsize = uart_write_bytes(m_port, (const char *)buffer, size);
    if (retsize == -1)
    {
    }
    return retsize;
}

size_t ESP32Serial::print(const char *text)
{
    return write((const uint8_t *)text, strlen(text));
}

int ESP32Serial::peek(void)
{
    uint8_t c;
    if (xQueuePeek(m_uart_queue, &c, 0))
    {
        return c;
    }
    return -1;
}