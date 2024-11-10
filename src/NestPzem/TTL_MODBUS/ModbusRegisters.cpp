
#include <Arduino.h>
#include <cstring> // For memcpy
#include "esp_log.h"
#include "ModbusRegisters.hpp"
#include "NestUtility.hpp"

namespace tmbus
{

float ModbusRegisters::readScaledValue(MeterType meterType) const
{
    return _registers[static_cast<uint8_t>(meterType)].getScaledValue();
}
void ModbusRegisters::Register::setValue(uint32_t newValue) { value = newValue; }

void ModbusRegisters::registerUpdateCallback(OnRegisterUpdateCallback callback)
{
    _callback = callback;
}
bool ModbusRegisters::updateSingleWrite(const uint8_t* buf, bool swapBytes)
{
    if(buf[1] != static_cast<uint8_t>(FunctionCode::WRITE_SINGLE_REG))
    {
        return false;
    }

    uint16_t registerAddress =
        swapBytes ? __builtin_bswap16(*(const uint16_t*)&buf[2]) : *(const uint16_t*)&buf[2];

    for(auto& reg: _registers)
    {
        if(reg.info == nullptr)
        {
            ESP_LOGW(MODBUS_REG_TAG, "Skipping register with nullptr info");
            continue;
        }

        if(reg.startAddress == registerAddress)
        {
            uint16_t newValue = swapBytes ? __builtin_bswap16(*(const uint16_t*)&buf[4]) :
                                            *(const uint16_t*)&buf[4];
            reg.value = newValue;

            if(_callback)
            {
                _callback(true, &reg);
            }
            return true;
        }
    }

    return false;
}

bool ModbusRegisters::updateSingle(MeterType type, const uint8_t* buf, bool swapBytes)
{
    if(buf[2] != 2 && buf[2] != 4) // Ensure byte count is 2 (for 16-bit) or 4 (for 32-bit)
    {
        return false;
    }

    ModbusRegisters::Register reg = _registers[static_cast<uint8_t>(type)];
    uint16_t length = reg.length;

    // For 16-bit registers (length == 1) with byte count 2
    if(length == 1 && buf[2] == 2)
    {
        uint16_t value = *(const uint16_t*)&buf[3]; // Extract 16-bit value
        reg.value = swapBytes ? __builtin_bswap16(value) : value;

        if(_callback)
        {
            _callback(true, &reg);
        }
        return true;
    }
    // For 32-bit registers (length == 2) with byte count 4
    else if(length == 2 && buf[2] == 4)
    {
        uint16_t low = *(const uint16_t*)&buf[3];  // Lower 16 bits
        uint16_t high = *(const uint16_t*)&buf[5]; // Upper 16 bits

        reg.value = swapBytes ? (__builtin_bswap16(high) << 16 | __builtin_bswap16(low)) :
                                (high << 16 | low);

        if(_callback)
        {
            _callback(true, &reg);
        }
        return true;
    }

    return false; // Unsupported length or invalid byte count
}

bool ModbusRegisters::isValidAddress(uint16_t address) const
{
    for(const auto& validAddr: _validAddress)
    {
        if(validAddr == address)
        {
            return true;
        }
    }
    return false;
}

bool ModbusRegisters::update(const uint16_t address, const uint8_t numRegs, const uint8_t* data,
                             bool swapBytes)
{
    uint16_t dataIndex = 0;
    uint16_t expectedAddress = address;
    uint8_t processedRegisters = 0;

    if(data == nullptr)
    {
        ESP_LOGW(MODBUS_REG_TAG, "aborting for null data");
        return false;
    }
    if(!isValidAddress(address))
    {
        ESP_LOGW(MODBUS_REG_TAG, "aborting for invalid register address");
        return false;
    }

    for(auto& reg: _registers)
    {
        if(processedRegisters >= numRegs) break;

        if(reg.info == nullptr)
        {
            ESP_LOGW(MODBUS_REG_TAG, "Skipping register with nullptr info");
            break;
        }

        if(reg.startAddress != expectedAddress)
        {
            continue;
        }

        uint16_t length = reg.length;
        uint32_t rawValue = 0;

        if(length == 1) // 16-bit register
        {
            rawValue = swapBytes ? __builtin_bswap16(*(const uint16_t*)&data[dataIndex]) :
                                   *(const uint16_t*)&data[dataIndex];
        }
        else if(length == 2) // 32-bit register
        {
            uint16_t low = *(const uint16_t*)&data[dataIndex];
            uint16_t high = *(const uint16_t*)&data[dataIndex + 2];
            rawValue = swapBytes ? (__builtin_bswap16(high) << 16 | __builtin_bswap16(low)) :
                                   (high << 16 | low);
        }

        reg.setValue(rawValue);

        // If a callback exists, trigger it
        if(_callback)
        {
            _callback(true, &reg);
        }

        // Update indexes for the next register
        dataIndex += (length * 2);
        expectedAddress += length;
        processedRegisters++; // Increment the count of processed registers
    }

    return true;
}
void ModbusRegisters::loadRegisters(
    const std::array<std::pair<MeterType, MeterInfo>, METER_MAP_004T_SIZE>& meterMap)
{
    std::unordered_set<std::pair<RegisterType, uint16_t>, Utility::PairHash> uniqueAddressSet;

    for(const auto& [meterType, meterInfo]: meterMap)
    {
        uint8_t index = static_cast<uint8_t>(meterType);
        if(index >= MAX_UNIQUE_REGISTERS)
        {
            ESP_LOGW(MODBUS_REG_TAG, "Skipping register %s, meterType index %d exceeds bounds.",
                     meterInfo.name, index);
            continue;
        }

        if(meterInfo.regType == RegisterType::IR || meterInfo.regType == RegisterType::HR
           || meterInfo.regType == RegisterType::PRG)
        {
            // Initialize the register
            _registers[static_cast<uint8_t>(meterType)] =
                Register(meterType, &meterInfo, meterInfo.startAddress, meterInfo.length, 0,
                         meterInfo.scaleFactor);

            // Create a unique (regType, startAddress) key
            auto addressKey = std::make_pair(meterInfo.regType, meterInfo.startAddress);
            auto result = uniqueAddressSet.insert(addressKey);

            // Insert into the unique set based on (regType, startAddress)
            if(result.second)
            {
                // Only add to _validAddress if space is available
                if(_validAddressCount < MAX_UNIQUE_REGISTERS)
                {
                    _validAddress[_validAddressCount++] = meterInfo.startAddress;
                }
                else
                {
                    ESP_LOGE(MODBUS_REG_TAG,
                             "Max valid addresses reached. Cannot add address: 0x%04X",
                             meterInfo.startAddress);
                }
            }
        }
        else
        {
            ESP_LOGE(MODBUS_REG_TAG, "The register type doesn't match!");
        }
    }
}

const ModbusRegisters::Register* ModbusRegisters::getRegister(MeterType type) const
{
    return &_registers[static_cast<uint8_t>(type)];
}
const ModbusRegisters::Register* ModbusRegisters::getRegister(const uint16_t startAddress) const
{
    for(const auto& reg: _registers)
    {
        if(reg.startAddress == startAddress)
        {
            return &reg;
        }
    }
    return nullptr;
}

uint16_t ModbusRegisters::adress(MeterType type) const
{
    return _registers[static_cast<uint8_t>(type)].startAddress;
}
uint16_t ModbusRegisters::length(MeterType type) const
{
    return _registers[static_cast<uint8_t>(type)].length;
}

const void tmbus::ModbusRegisters::getPowerMeasures(pzemCore::powerMeasure& pm)
{
    std::array<float, 7> values;

    for(size_t i = 0; i <= static_cast<size_t>(tmbus::MeterType::ALARM_STATUS); ++i)
    {
        if(_registers[i].info == nullptr)
        {
            break;
        }

        if(_registers[i].info->regType == tmbus::RegisterType::IR)
        {
            float scaledValue = static_cast<float>(_registers[i].value)
                                / static_cast<float>(_registers[i].scaleFactor);

            values[i] = scaledValue;
        }
        else
        {
            values[i] = 0; // Default to 0 value and scale factor of 1
        }
    }

    pm = pzemCore::powerMeasure(values, _model);
}
void ModbusRegisters::printAllRegisters()
{
    printRegisters(RegisterType::IR);
    printRegisters(RegisterType::HR);
    printRegisters(RegisterType::PRG);
}
void ModbusRegisters::printRegisters(RegisterType regType)
{
    switch(regType)
    {
        case RegisterType::IR:
            Serial.println("\n---------- Input Registers ----------");
            break;
        case RegisterType::HR:
            Serial.println("\n---------- Holding Registers ----------");
            break;
        case RegisterType::PRG:
            Serial.println("\n---------- PRG Registers ----------");
            break;
        default:
            Serial.println("\n---------- Unknown Registers ----------");
            break;
    }

    auto printRegister = [](const Register& reg) {
        if(reg.info == nullptr)
        {
            return;
        }
        Serial.printf("\nMeter: %s", reg.info->name);
        Serial.printf("\n");
        Serial.print("Operation Type: ");
        switch(reg.info->opType)
        {
            case OperationType::READONLY:
                Serial.println("Read");
                break;
            case OperationType::WRITEONLY:
                Serial.println("Write");
                break;
            default:
                Serial.println("Read/Write");
                break;
        }
        uint32_t newValue = reg.value;
        uint16_t scaleFactor = reg.info->scaleFactor;
        float scaledValue = static_cast<float>(newValue) / static_cast<float>(scaleFactor);
        Serial.printf("Start Address: 0x%04X\n", reg.startAddress);
        Serial.printf("Length: %d\n", reg.length);
        Serial.printf("Value: %d\n", newValue);
        Serial.printf("Scale: %d\n", scaleFactor);

        Serial.printf("Scaled Value: %.2f\n", scaledValue);
        Serial.printf("Meter Unit:%s\n ", reg.info->unit);
        Serial.println("---------------------------------");
    };

    // Iterate through the registers and print only those matching the requested type
    for(size_t i = 0; i < _registers.size(); i++)
    {
        const Register& reg = _registers[i];
        if(reg.info == nullptr)
        {
            continue;
        }
        // Check if the register type matches the requested type
        if(reg.info->regType == regType)
        {
            printRegister(reg);
        }
    }
}

} // namespace tmbus
