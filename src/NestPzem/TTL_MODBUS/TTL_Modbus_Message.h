#ifndef TTL_MODBUS_MESSAGE_H
#define TTL_MODBUS_MESSAGE_H
#include "Arduino.h"
#include <cstdint>
#include <cstring>
#include "NestUtility.hpp"
#include "ModbusDefaults.h"
#include "esp_log.h"

namespace tmbus
{
static const char* MODBUS_MSG_TAG __attribute__((unused)) = "MODBUS_MESSAGE";

struct TTLModbusMessage
{
    struct Frame
    {
        struct TxFrame
        {
            struct Initials
            {
                uint8_t slaveAddr = 0;
                uint8_t cmd = 0;
            } __attribute__((packed)) initials;
            uint16_t regAddr = 0;
            uint16_t valueOrNumRegs = 0;
        } __attribute__((packed)) txFrame;

        struct RxFrame
        {
            struct Initials
            {
                uint8_t slaveAddr = 0;
                uint8_t cmd = 0;
            } __attribute__((packed)) initials;
            uint8_t byteCount = 0;
            uint8_t data[MAX_RIR_RESPONSE_LENGTH + 1];

        } __attribute__((packed)) rxFrame;
    } __attribute__((packed)) frame;
    ~TTLModbusMessage() = default;

   private:
    uint16_t crcRX = 0xFFFF; // CRC field
    uint16_t crcTX = 0xFFFF; // CRC field
    bool isRXValid = false;

    void updateCRC_RX(uint16_t crcNew) { crcRX = crcNew; }
    void updateCRC_TX(uint16_t crcNew) { crcTX = crcNew; }

   public:
    // Default constructor
    TTLModbusMessage() = default;

    // Constructor for TX message
    TTLModbusMessage(const uint8_t slaveAddr, const uint8_t cmd);
    TTLModbusMessage(const uint8_t slaveAddr, const uint8_t cmd, const uint16_t regAddr,
                     const uint16_t valueOrNumRegs);
    // Constructor for RX message
    TTLModbusMessage(const uint8_t slaveAddr, const uint8_t cmd, const bool isValid);
    TTLModbusMessage(const uint8_t slaveAddr, const uint8_t cmd, const uint8_t byteCount,
                     const uint8_t* data, const bool isValid);

    // Function declarations
    bool decodeMessage(const uint8_t* buf, const uint16_t len, bool checkLength = true);
    bool serializeData(uint8_t* buffer, uint16_t datalength, bool isReceiverBigEndian = true);
    bool setCRC(uint8_t* buffer, uint16_t len);
    bool serializeMessage(uint8_t* buffer, uint16_t len, bool isReceiverBigEndian = true);
    bool serializeRESETMessage(uint8_t* buffer, uint16_t len, bool isReceiverBigEndian = true);
    void printSerializedMessage(const uint8_t* buffer, uint16_t len);

    // Getters
    uint8_t getSlaveAddress() const;
    uint8_t getCommand() const;
    uint16_t getRegAddress() const;
    uint16_t getValues() const;
    uint8_t getByteCount() const;
    const uint8_t* getValidRawData() const;
    const Frame::TxFrame* getTXFrame() const;
    const Frame::RxFrame* getRXFrame() const;
    uint16_t getCRC_RX() const;
    uint16_t getCRC_TX() const;
    uint8_t getRXDataSize() const;

    uint8_t get_TX_FrameSize() const;
    uint8_t get_RX_FrameSize() const;

   private:
    bool isValidFunctionCode(const uint8_t* buf);
};

} // namespace tmbus

#endif