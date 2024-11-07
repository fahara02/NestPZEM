#ifndef MODBUS_DEFAULT_H
#define MODBUS_DEFAULT_H
#include <cstdint>
#include <cstring>
#include <string>
namespace tmbus
{
static const size_t MAX_COMMANDS = 3;
static constexpr size_t METER_MAP_004T_SIZE = 10;
static constexpr size_t METER_MAP_003_SIZE = 10;

static constexpr size_t MAX_UNIQUE_REGSITERS = 13; // any model 16 bit or 32 bit
static constexpr size_t MAX_IR_NUMS = 7;           // any model
static constexpr size_t MAX_HR_NUMS = 3;           // any model
static constexpr size_t MAX_PRG_NUMS = 2;          // any model

static constexpr size_t MAX_RIR_RESPONSE_LENGTH = 20; // any model
static constexpr size_t MAX_RESPONSE_SIZE = 25;       // any model = 25

const uint8_t TTL_MODBUS_NORMAL_FRAME_SIZE = 8;
const uint8_t TTL_MODBUS_REPORT_ADDR_FRAME_SIZE = 5;
const uint8_t TTL_MODBUS_ENERGY_RST_FRAME_SIZE = 4;

// Additional PZEM004T Register Handling Constants
static constexpr size_t PZ004T_IR_NUMS = 7; // Maximum Unique IR Numbers
static constexpr size_t PZ004T_HR_NUMS = 2; // Maximum Unique IR Numbers
static constexpr size_t PZ004T_PRG_NUMS = 1;
constexpr uint16_t PZ004T_RIR_DATA_BEGIN = 0x0000;
constexpr uint16_t PZ004T_RHR_DATA_BEGIN = 0x0001;
constexpr uint16_t PZ004T_RIR_DATA_LEN = 0x0A;
constexpr uint16_t PZ004_RHR_DATA_LEN = 2;
constexpr uint16_t PZ004T_RIR_RESP_LEN = 0x14;

// Additional PZEM003 Register Handling Constants
static constexpr size_t PZ003_IR_NUMS = 5; // Maximum Unique IR Numbers
static constexpr size_t PZ003_HR_NUMS = 3; // Maximum Unique IR Numbers
static constexpr size_t PZ003_PRG_NUMS = 2;
constexpr uint16_t PZ003_RIR_DATA_BEGIN = 0x00; // Start of register data
constexpr uint16_t PZ003_RHR_DATA_BEGIN = 0x00;
constexpr uint16_t PZ003_RIR_DATA_LEN = 0x08; // Total registers to read
constexpr uint16_t PZ003_RHR_DATA_LEN = 4;    // number of RHR regs
constexpr uint16_t PZ003_RIR_RESP_LEN = 0x10; // Response length in bytes (16 bytes)

// Factory calibration
#define CAL_ADDR ADDR_ANY // Calibration address
#define CAL_PWD 0x3721    // Calibration password

// Power Alarm

#define GENERIC_MSG_SIZE 8
#define ENERGY_RST_MSG_SIZE 4
#define REPORT_ADDR_MSG_SIZE 5

// RX
#define rx_msg_q_DEPTH 10
#define EVT_TASK_PRIO 4
#define EVT_TASK_STACK 3072
#define EVT_TASK_NAME "UART_EVQ"

// TX
#define tx_msg_q_DEPTH 8
#define TXQ_TASK_PRIO 2
#define TXQ_TASK_STACK 2048
#define TXQ_TASK_NAME "UART_TXQ"

// enum class RegisterType { IR, HR, DI, DO, NONE };
enum class MeterType {
    VOLTAGE = 0,            // IR
    CURRENT = 1,            // IR
    POWER = 2,              // IR
    ENERGY = 3,             // IR
    FREQUENCY = 4,          // IR
    PF = 5,                 // IR
    ALARM_STATUS = 6,       // IR
    ALARM_THRESHOLD = 7,    // HR only for PZE04T
    HV_ALARM_THRESHOLD = 8, // HR only for PZE003
    LV_ALARM_THRESHOLD = 9, // HR only for PZE003
    SLAVE_ADDRESS = 10,     // HR
    CURRENT_RANGE = 11,     // HR  only for PZE003
    RESET = 12,             // PRG
    CALIBRATE = 13,         // PRG

};
enum class RegisterType { IR, HR, PRG, NONE };
enum class OperationType { READONLY, WRITEONLY, READ_WRITE, NONE };
struct MeterInfo
{
    RegisterType regType : 4; // Bitfield for compact size
    OperationType opType : 4; // Bitfield for compact size
    uint16_t startAddress;
    uint16_t length;
    uint16_t scaleFactor;
    char name[20]; // Fixed-size char array for embedded systems
    char unit[10]; // Fixed-size char array for embedded systems

    // Default constructor
    MeterInfo() :
        regType(RegisterType::NONE),
        opType(OperationType::NONE),
        startAddress(0),
        length(0),
        scaleFactor(1)
    {
        std::memset(name, 0, sizeof(name)); // Initialize to empty string
        std::memset(unit, 0, sizeof(unit)); // Initialize to empty string
    }

    // Parameterized constructor
    MeterInfo(RegisterType reg, OperationType op, uint16_t start, uint16_t len, uint16_t scale,
              const char* n, const char* u) :
        regType(reg), opType(op), startAddress(start), length(len), scaleFactor(scale)
    {
        std::strncpy(name, n, sizeof(name) - 1); // Safely copy name
        std::strncpy(unit, u, sizeof(unit) - 1); // Safely copy unit
        name[sizeof(name) - 1] = '\0';           // Ensure null-termination
        unit[sizeof(unit) - 1] = '\0';           // Ensure null-termination
    }
};

enum class FunctionCode : uint8_t {
    READ_HOLDING_REG = 0x03, // Read Holding Register
    READ_INPUT_REG = 0x04,   // Read Input Register
    WRITE_SINGLE_REG = 0x06, // Write Single Register
    CALIBRATE = 0x41,        // Calibration
    RESET = 0x42,            // Reset energy
    CMD_RERR = 0x84,         // Read  Command error
    CMD_WERR = 0x86,         // Write Command error
    CMD_CALERR = 0xC1,       // Calibration Command error
    CMD_RSTERR = 0xC2        // Reset Energy Command error
};

enum class SLAVE_ADDRESS : uint8_t {
    ADDR_BCAST = 0x00, // broadcast address    (slaves are not supposed to answer here)
    ADDR_MIN = 0x01,   // lowest slave address
    ADDR_MAX = 0xF7,   // highest slave address
    ADDR_ANY = 0xF8    // default catch-all address
};
enum class ErrorCode : uint8_t {
    ERR_FUNC = 0x01,  // Illegal function
    ERR_ADDR = 0x02,  // Illegal address
    ERR_DATA = 0x03,  // Illegal data
    ERR_SLAVE = 0x04, // Slave error
};
enum class AlarmState : uint16_t { ALARM_PRESENT = 0xffff, ALARM_ABSENT = 0x0000 };

} // namespace tmbus
#endif