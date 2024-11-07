#ifndef PZEM_DEFAULTS_H
#define PZEM_DEFAULTS_H

#include "ModbusDefaults.h"

namespace pzemCore
{
const uint16_t PZEM_UART_TIMEOUT = 100;      // ms to wait for PZEM RX/TX messaging
const uint16_t PZEM_UART_RX_READ_TICKS = 10; // ticks to wait for RX byte read from buffer

#define PZEM_REFRESH_PERIOD 2000 // ms, PZEM updates it's internal register data every ~1 sec
#define POLLER_PERIOD PZEM_REFRESH_PERIOD       // auto update period in ms
#define POLLER_MIN_PERIOD 2 * PZEM_UART_TIMEOUT // minimal poller period

#define POOL_POLLER_NAME "PZP_Poll"
#define TIMER_CMD_TIMEOUT 10 // block up to x ticks trying to change timer params
enum class PZEMModel : uint8_t { PZEM004T, PZEM003, NOT_SET };
enum class Phase { SINGLE_PHASE, RED_PHASE, YELLOW_PHASE, BLUE_PHASE };
enum class pzem_err_t : uint8_t {
    err_ok = 0,
    err_func = static_cast<uint8_t>(tmbus::ErrorCode::ERR_FUNC),
    err_addr = static_cast<uint8_t>(tmbus::ErrorCode::ERR_ADDR),
    err_data = static_cast<uint8_t>(tmbus::ErrorCode::ERR_DATA),
    err_slave = static_cast<uint8_t>(tmbus::ErrorCode::ERR_SLAVE),
    err_parse // error parsing reply
};

enum class Update { FULL, PARTIAL, SINGLE, INVALID };
enum class State { INIT, ACTIVE, IDLE, BUSY, POLL, UPDATE_FAIL, NOLOAD, NO_VOLTAGE };
struct DeviceConfig
{
    uint16_t maxWatt = 2500;
    uint16_t curRange = 10;
    uint16_t alarmThresh = 2250;
};
enum class shunt_t : uint8_t { type_100A = 0, type_50A = 1, type_200A = 2, type_300A = 3 };
} // namespace pzemCore

#endif