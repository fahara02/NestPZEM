#ifndef PZEM_DEVICE_HPP
#define PZEM_DEVICE_HPP

#include <cstdint>
#include <list>
#include "PZEM_constants.h"
#include "powerMeasure.h"
#include "UARTMsgQueue.hpp"
#include "ModbusRegisters.hpp"
#include "esp_log.h"
#include "PZEMClient.hpp"
#include "IModbusDevice.hpp"
#include "RTU_Modbus.hpp"
#include "PowerMeter.hpp"

using namespace tmbus;

namespace pzemCore
{

static const char* DEVICE_TAG __attribute__((unused)) = "PZEMDevice";

class PZEMDevice : public PZEMClient, public IModbusDevice, public PowerMeter
{
    PZEMModel _model;
    uint8_t _id;
    uint8_t _addr;
    DeviceConfig _cfg;
    RTUModbus _rtu;

   public:
    explicit PZEMDevice(PZEMModel model, uint8_t id, uint8_t addr, uint8_t lineNo = 0,
                        Phase phase = Phase::SINGLE_PHASE, const char* nametag = nullptr);
    ~PZEMDevice() override;

    // Override from PZEM
    void rx_sink(const tmbus::receive* msg) override;
    // Override from IModbusDevice
    void init() override;
    bool updateRegisters(RegisterType regType, Update updateType, const uint8_t* data,
                         const uint8_t length = 0, const uint16_t startAddress = 0x0000) override;

    // Ovverride from power meters
    bool updateMeters() override;

    bool resetEnergyCounter() override;
    uint8_t getaddr() const override;

    transmit* resetEnergy();
    transmit* setShunt(shunt_t shunt);
    transmit* getAlarmThreshold();
    transmit* setAlarmThreshold(uint16_t value);
    transmit* getModbusAddress();
    transmit* setModbusAddress(uint8_t newAddr);
    bool parse_RXMessage(const receive* rxMessage);

    bool isValidAddress(uint8_t address);
    const char* getNameTag() const { return _nameTag; }
    PZEMModel getModel() const { return _model; }

    void updateMetersTask();

   private:
    bool _initialised = false;
    bool isBigEndian;
    std::shared_ptr<tmbus::ModbusRegisters> modbusRegisters;
    TaskHandle_t _updateTaskHandle;
    std::atomic<bool> _stopTask;
    transmit* readSingleRegister(tmbus::MeterType meterType, bool waitForReply = true);
    transmit* writeSingleRegister(tmbus::MeterType meterType, uint16_t value,
                                  bool waitForReply = true);
    transmit* readInputRegisters();
    transmit* readHoldingRegisters();
    bool parseModbusData(const uint8_t* data, const uint8_t length);
    bool parseDataByRegister(const uint8_t* data, const uint8_t length, RegisterType regType);

    bool isValidThreshHold(uint16_t value);
};

} // namespace pzemCore

#endif // PZEM_DEVICE_H
