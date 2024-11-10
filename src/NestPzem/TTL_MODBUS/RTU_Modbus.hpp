#ifndef RTU_MODBUS_HPP
#define RTU_MODBUS_HPP

#include "UARTMsgQueue.hpp"
#include "ModbusDefaults.h"
#include "IModbusDevice.hpp"
#include "esp_log.h"
#include "TTL_Modbus_Message.h"

namespace tmbus
{
static const char* MODBUS_TAG = "ttlModbus";

class RTUModbus
{
   private:
    pzemCore::IModbusDevice* _device; // Device to communicate with
    uint8_t _slaveAddress;            // Modbus slave address of the device

   public:
    RTUModbus(pzemCore::IModbusDevice* device, uint8_t addr) : _device(device), _slaveAddress(addr)
    {
        initializeModbus();
    }
    transmit* sendMsg(FunctionCode cmd, uint16_t regAddr, uint16_t numRegs, uint8_t slaveAddr,
                      bool waitForReply = true);
    transmit* sendEnergyResetMsg(uint8_t slaveAddr, uint16_t regAddr);

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
    const TTLModbusMessage::Frame::TxFrame* getTXFrame() const;
    const TTLModbusMessage::Frame::RxFrame* getRXFrame() const;
    uint16_t getCRC_RX() const;
    uint16_t getCRC_TX() const;
    uint8_t getRXDataSize() const;
    uint8_t get_TX_FrameSize() const;
    uint8_t get_RX_FrameSize() const;

   private:
    void initializeModbus()
    {
        ESP_LOGI(MODBUS_TAG, "Initializing Modbus communication for device at address %zu",
                 static_cast<size_t>(_slaveAddress));
    }
};

} // namespace tmbus
#endif