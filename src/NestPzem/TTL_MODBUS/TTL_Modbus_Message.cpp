#include "TTL_Modbus_Message.h"
#include <cstring> // For memcpy
static const char* MODBUS__MSG = "ModbusMsg";
namespace tmbus
{
// Constructor for TX message
TTLModbusMessage::TTLModbusMessage(const uint8_t slaveAddr, const uint8_t cmd)
{
    frame.txFrame.initials.slaveAddr = slaveAddr;
    frame.txFrame.initials.cmd = cmd;
}

TTLModbusMessage::TTLModbusMessage(const uint8_t slaveAddr, const uint8_t cmd,
                                   const uint16_t regAddr, const uint16_t valueOrNumRegs) :
    TTLModbusMessage(slaveAddr, cmd)
{
    frame.txFrame.regAddr = regAddr;
    frame.txFrame.valueOrNumRegs = valueOrNumRegs;
}

// Constructor for RX message
TTLModbusMessage::TTLModbusMessage(const uint8_t slaveAddr, const uint8_t cmd, const bool isValid)
{
    frame.rxFrame.initials.slaveAddr = slaveAddr;
    frame.rxFrame.initials.cmd = cmd;
    isRXValid = isValid;
}

TTLModbusMessage::TTLModbusMessage(uint8_t slaveAddr, uint8_t cmd, uint8_t byteCount,
                                   const uint8_t* data, bool isValid) :
    TTLModbusMessage(slaveAddr, cmd, isValid)
{
    frame.rxFrame.byteCount = byteCount;
    if(byteCount <= MAX_RIR_RESPONSE_LENGTH)
    {
        memcpy(frame.rxFrame.data, data, byteCount);
    }
}
bool TTLModbusMessage::isValidFunctionCode(const uint8_t* buf)
{
    if(buf == nullptr)
    {
        return false;
    }

    uint8_t code = buf[1];

    // Define a sparse lookup table for only the specific function codes
    static constexpr std::pair<uint8_t, bool> validFunctionCodes[] = {
        {0x03, true}, // READ_HOLDING_REG
        {0x04, true}, // READ_INPUT_REG
        {0x06, true}, // WRITE_SINGLE_REG
        {0x41, true}, // CALIBRATE
        {0x42, true}, // RESET
        {0x84, true}, // CMD_RERR (Read Command error)
        {0x86, true}, // CMD_WERR (Write Command error)
        {0xC1, true}, // CMD_CALERR (Calibration Command error)
        {0xC2, true}  // CMD_RSTERR (Reset Energy Command error)
    };

    for(const auto& entry: validFunctionCodes)
    {
        if(entry.first == code)
        {
            return entry.second;
        }
    }

    return false;
}

bool TTLModbusMessage::decodeMessage(const uint8_t* buf, const uint16_t len, bool checkLength)
{
    if(buf == nullptr)
    {
        ESP_LOGW(MODBUS_MSG_TAG, "buffer is nullptr");
        return false;
    }
    if(checkLength)
    {
        if(len < TTL_MODBUS_ENERGY_RST_FRAME_SIZE)
        {
            ESP_LOGW(MODBUS_MSG_TAG, "buffer length %d is too low ", len);
            return false;
        }
    }
    if(isValidFunctionCode(buf))
    {
        frame.rxFrame.initials.slaveAddr = buf[0];
        frame.rxFrame.initials.cmd = buf[1];
        frame.rxFrame.byteCount = buf[2]; // 3rd byte in RX frame is the byte count

        // Calculate the length of data to copy, ensuring it does not exceed the buffer size
        uint8_t dataLen = (frame.rxFrame.byteCount < sizeof(frame.rxFrame.data)) ?
                              frame.rxFrame.byteCount :
                              sizeof(frame.rxFrame.data);

        if(len < (3 + dataLen + sizeof(crcRX)))
        {
            ESP_LOGW(MODBUS_MSG_TAG, "Invalid message length");
            return false;
        }

        memcpy(frame.rxFrame.data, &buf[3], dataLen);
        memcpy(&crcRX, &buf[3 + dataLen], sizeof(crcRX));

        return true;
    }
    else
    {
        ESP_LOGW(MODBUS_MSG_TAG, "invalid function code ,  data corrupted");
        return false;
    }
}
bool TTLModbusMessage::serializeRESETMessage(uint8_t* buffer, uint16_t len,
                                             bool isReceiverBigEndian)
{
    if(len != TTL_MODBUS_ENERGY_RST_FRAME_SIZE)
    {
        return false;
    }
    else
    {
        uint16_t bufferLength = TTL_MODBUS_ENERGY_RST_FRAME_SIZE;
        buffer[0] = frame.txFrame.initials.slaveAddr;
        buffer[1] = frame.txFrame.initials.cmd;
        if(setCRC(buffer, bufferLength))
        {
            printSerializedMessage(buffer, len);
            return true;
        }
        else
        {
            return false;
        }
    }
}

bool TTLModbusMessage::serializeMessage(uint8_t* buffer, uint16_t len, bool isReceiverBigEndian)
{
    uint16_t dataLength = len - 2;

    if(serializeData(buffer, dataLength, isReceiverBigEndian))
    {
        if(setCRC(buffer, len))
        {
            printSerializedMessage(buffer, len);
            return true;
        }
        return false;
    }
    else
    {
        return false;
    }
}

bool TTLModbusMessage::serializeData(uint8_t* buffer, uint16_t datalength, bool isReceiverBigEndian)
{
    if(datalength < TTL_MODBUS_REPORT_ADDR_FRAME_SIZE - 2)
    {
        ESP_LOGE(MODBUS__MSG, "Unexpected Error, data length is shorter than default");
        return false;
    }
    else
    {
        // One to one map as single byte
        buffer[0] = frame.txFrame.initials.slaveAddr;
        buffer[1] = frame.txFrame.initials.cmd;
        // Handle CRC endianness
        if(isReceiverBigEndian)
        {
            // PZEM device is BIG Endian so swap byetes before transmission
            *(uint16_t*)&buffer[2] = __builtin_bswap16(frame.txFrame.regAddr);
            *(uint16_t*)&buffer[4] = __builtin_bswap16(frame.txFrame.valueOrNumRegs);
            return true;
        }
        else

        { // No change in data
            buffer[2] = frame.txFrame.regAddr;
            buffer[4] = frame.txFrame.valueOrNumRegs;
            return true;
        }
    }
}
bool TTLModbusMessage::setCRC(uint8_t* buffer, uint16_t len)
{
    uint16_t crc = Utility::CRC::calculate(buffer, len - 2);

    if(len > 2)
    {
        // stackoverflow.com/questions/73455678/crc-does-a-little-endian-bytestream-have-a-big-endian-crc-at-the-end
        // MODBUS over serial line specification and implementation guide V1.01  Page 39 CRC
        // Always LSB first and table reflected
        crc = Utility::CRC::calculate(buffer, len - 2);
        buffer[len - 2] = static_cast<uint8_t>(crc & 0xFF);      // CRC low byte
        buffer[len - 1] = static_cast<uint8_t>(crc >> 8) & 0xFF; // CRC high byte

        return true;
    }

    else
    {
        return false;
    }
    updateCRC_TX(crc);
    return true;
}
uint8_t TTLModbusMessage::getSlaveAddress() const { return frame.txFrame.initials.slaveAddr; }
uint8_t TTLModbusMessage::getCommand() const { return frame.txFrame.initials.cmd; }
uint16_t TTLModbusMessage::getRegAddress() const { return frame.txFrame.regAddr; }
uint16_t TTLModbusMessage::getValues() const { return frame.txFrame.valueOrNumRegs; }

const uint8_t* TTLModbusMessage::getValidRawData() const { return frame.rxFrame.data; }
const TTLModbusMessage::Frame::TxFrame* TTLModbusMessage::getTXFrame() const
{
    return &frame.txFrame;
}
const TTLModbusMessage::Frame::RxFrame* TTLModbusMessage::getRXFrame() const
{
    return &frame.rxFrame;
}

uint16_t TTLModbusMessage::getCRC_RX() const { return crcRX; }
uint16_t TTLModbusMessage::getCRC_TX() const { return crcTX; }

uint8_t TTLModbusMessage::getRXDataSize() const { return sizeof(frame.rxFrame.data); }
uint8_t TTLModbusMessage::getByteCount() const { return frame.rxFrame.byteCount; }
uint8_t TTLModbusMessage::get_TX_FrameSize() const
{
    return 3 + sizeof(crcTX); // slaveAddr + cmd + data + crc
}
uint8_t TTLModbusMessage::get_RX_FrameSize() const
{
    return 3 + frame.rxFrame.byteCount + sizeof(crcRX); // slaveAddr + cmd + byteCount + data + crc
}
void TTLModbusMessage::printSerializedMessage(const uint8_t* buffer, uint16_t len)
{
    for(uint16_t i = 0; i < len; ++i)
    {
        // Print each byte in HEX format with leading zeros for single-digit values
        if(buffer[i] < 0x10) Serial.print("0");
        Serial.print(buffer[i], HEX);

        // Add a comma after each byte, except for the last one
        if(i < len - 1)
        {
            Serial.print(",");
        }
    }
    Serial.println(); // End the line after printing the message
}

} // namespace tmbus