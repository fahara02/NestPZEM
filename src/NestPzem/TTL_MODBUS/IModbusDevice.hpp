#ifndef IMODBUS_DEVICE_HPP
#define IMODBUS_DEVICE_HPP

#include <utility>
#include "ModbusRegisters.hpp"
#include "PZEM_Defaults.h"
#include <memory>
#include <utility>
#include <optional>
#include <exception>
#include <new>

using namespace tmbus;

namespace pzemCore
{
static const char* IMODBUS_TAG __attribute__((unused)) = "IModbusDevice";

class IModbusDevice
{
   private:
    uint8_t _id;
    PZEMModel _model;
    std::array<std::pair<uint8_t, uint16_t>, MAX_COMMANDS> _lastSendCommands;

   public:
    explicit IModbusDevice(uint8_t id, PZEMModel model) :
        _id(id),
        _model(model),
        _modbusRegisters(std::make_shared<tmbus::ModbusRegisters>(model, id))
    {
        // ESP_LOGI(IMODBUS_TAG, "Modbus register ptr in IModbusDevice: %p",
        // _modbusRegisters.get());
        setRegisters(model);
    }

    std::shared_ptr<tmbus::ModbusRegisters> getModbusRegisters() { return _modbusRegisters; }

    virtual void init() = 0;
    virtual bool updateRegisters(RegisterType regType, Update updateType, const uint8_t* data,
                                 const uint8_t length, const uint16_t startAddress = 0x0000) = 0;

    void manageList(uint8_t cmd, uint16_t regAddr);
    void setRegisters(PZEMModel m);

   protected:
    std::shared_ptr<tmbus::ModbusRegisters> _modbusRegisters;
    size_t _commandIndex = 0;
    const tmbus::ModbusRegisters::Register* getRegister(MeterType meterType) const;
    const std::pair<uint16_t, uint16_t> getallIR() const;
    const std::pair<uint16_t, uint16_t> getallHR() const;
    std::optional<uint16_t> getMatchingStartAddress(uint8_t cmd) const;
    size_t numRIR;
    size_t numRHR;
    size_t numPRG;
    uint16_t RIR_MAX_RESP_LENGTH;
    uint16_t RIR_BEGIN;
    uint16_t RHR_BEGIN;
    uint16_t RIR_LENGTH;
    uint16_t RHR_LENGTH;

   private:
    bool _initialised = false;
    std::array<std::pair<tmbus::MeterType, tmbus::MeterInfo>, tmbus::METER_MAP_004T_SIZE>*
        meterMapPtr = nullptr;
};

} // namespace pzemCore

#endif // IMODBUS_DEVICE_H
