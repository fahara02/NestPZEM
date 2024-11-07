
#ifndef PZEM_CONSTANTS_H
#define PZEM_CONSTANTS_H

#include <cstdint>

namespace pzemCore
{ // Default Configuration
const uint8_t PZEM_DEFAULT_ADDR = 0xF8;
#define PZEM_UART UART_NUM_1
const uint16_t PZEM_BAUD_RATE = 9600;
const uint16_t PZEM_DEFAULT_READ_TIMEOUT = 1000;
// HW Serial Port 2 on ESP32

#define RX_BUFFER_SIZE (UART_FIFO_LEN * 2) // 2xUART_FIFO_LEN is enough to fit 10 PZEM msg's
#define TX_BUFFER_SIZE (0)

// Timeout in milliseconds

} // namespace pzemCore

#endif