#ifndef MODBUS_REGISTER_HPP
#define MODBUS_REGISTER_HPP
#include <cstdint>
#include <map>
#include <unordered_set>
#include <memory>
#include <unordered_map>
#include <vector>
#include <utility>
#include "ModbusDefaults.h"
#include "PZEM_Defaults.h"
#include "powerMeasure.h"
#include "pgmspace.h"
#include <functional>
#include "NestUtility.hpp"
namespace tmbus
{

static const char* MODBUS_REG_TAG __attribute__((unused)) = "MODBUS_REGISTER";

class ModbusRegisters
{
   public:
    struct Register
    {
        MeterType meterType;
        const MeterInfo* info;
        uint16_t startAddress;
        uint16_t length;
        uint32_t value;
        uint16_t scaleFactor;

        Register() :
            meterType(MeterType::VOLTAGE),
            info(nullptr),
            startAddress(0),
            length(0),
            value(0),
            scaleFactor(0)

        {
        }

        Register(MeterType mType, const MeterInfo* info, uint16_t addr, uint16_t len, uint32_t val,
                 uint16_t scval) :
            meterType(mType),
            info(info),
            startAddress(addr),
            length(len),
            value(val),
            scaleFactor(scval)

        {
        }

        // Set the value and update the scaled value based on the scale factor
        void setValue(uint32_t newValue);
        float getScaledValue() const
        {
            return static_cast<float>(value) / static_cast<float>(scaleFactor);
        }
    };
    ModbusRegisters(pzemCore::PZEMModel model, uint8_t id) :
        _model(model), _id(id), _callback(nullptr)
    {
        for(size_t i = 0; i < MAX_UNIQUE_REGISTERS; ++i)
        {
            _registers[i] = Register();
        }
    }

    float readScaledValue(MeterType meterType) const;

    void loadRegisters(const std::array<std::pair<tmbus::MeterType, tmbus::MeterInfo>,
                                        tmbus::METER_MAP_004T_SIZE>& metermap);
    bool update(const uint16_t address, const uint8_t numRegs, const uint8_t* data,
                bool swapBytes = true);
    bool updateSingle(MeterType type, const uint8_t* buf, bool swapBytes);
    bool updateSingleWrite(const uint8_t* buf, bool swapBytes);

    // pzemCore::powerMeasure getPowerMeasures() const;
    uint16_t adress(MeterType type) const;
    uint16_t length(MeterType type) const;
    pzemCore::PZEMModel getModel() { return _model; }
    const ModbusRegisters::Register* getRegister(MeterType type) const;
    const ModbusRegisters::Register* getRegister(const uint16_t startAddress) const;
    const void getPowerMeasures(pzemCore::powerMeasure& pm);

    void printAllRegisters();
    void printRegisters(RegisterType regType = RegisterType::IR);

   private:
    pzemCore::PZEMModel _model;
    uint8_t _id;
    size_t _validAddressCount = 0;

    using OnRegisterUpdateCallback =
        std::function<void(bool registerUpdated, ModbusRegisters::Register* reg)>;
    OnRegisterUpdateCallback _callback;
    std::array<tmbus::ModbusRegisters::Register, MAX_UNIQUE_REGISTERS> _registers;
    std::array<uint16_t, MAX_UNIQUE_REGISTERS> _validAddress;
    bool isValidAddress(uint16_t address) const;

   public:
    void registerUpdateCallback(OnRegisterUpdateCallback callback);
};

} // namespace tmbus

#endif // MODBUS_REGISTER_HPP
