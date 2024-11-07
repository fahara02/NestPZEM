#include "IModbusDevice.hpp"
#include "esp_log.h"
#include "NestUtility.hpp"
namespace pzemCore
{

void IModbusDevice::setRegisters(PZEMModel m)
{
    Utility::MeterMap& metermap = Utility::MeterMap::getInstance();
    meterMapPtr = metermap.sendMapReference(m);
    // ESP_LOGI("IModbusDevice", "Meter map pointer: %p", meterMapPtr);

    if(meterMapPtr && !meterMapPtr->empty())
    {
        // ESP_LOGI("IModbusDevice", "Loading Meter map ref for model: %s",
        //          Utility::ToString::model(m));
        _modbusRegisters->loadRegisters(*meterMapPtr);

        // Initialize registers based on the model
        if(m == PZEMModel::PZEM004T)
        {
            RIR_BEGIN = tmbus::PZ004T_RIR_DATA_BEGIN;
            RIR_LENGTH = tmbus::PZ004T_RIR_DATA_LEN;
            RHR_BEGIN = tmbus::PZ004T_RHR_DATA_BEGIN;
            RHR_LENGTH = tmbus::PZ004_RHR_DATA_LEN;
            RIR_MAX_RESP_LENGTH = tmbus::PZ004T_RIR_RESP_LEN;
            numRIR = tmbus::PZ004T_IR_NUMS;
            numRHR = tmbus::PZ004T_HR_NUMS;
            numPRG = tmbus::PZ004T_PRG_NUMS;
        }
        else if(m == PZEMModel::PZEM003)
        {
            RIR_BEGIN = tmbus::PZ003_RIR_DATA_BEGIN;
            RIR_LENGTH = tmbus::PZ003_RIR_DATA_LEN;
            RHR_BEGIN = tmbus::PZ003_RHR_DATA_BEGIN;
            RHR_LENGTH = tmbus::PZ003_RHR_DATA_LEN;
            RIR_MAX_RESP_LENGTH = tmbus::PZ003_RIR_RESP_LEN;
            numRIR = tmbus::PZ003_IR_NUMS;
            numRHR = tmbus::PZ003_HR_NUMS;
            numPRG = tmbus::PZ003_PRG_NUMS;
        }
    }
    else
    {
        ESP_LOGE("IModbusDevice", "Meter map for model %d not found or invalid",
                 static_cast<int>(m));
        return;
    }
}

const tmbus::ModbusRegisters::Register* IModbusDevice::getRegister(MeterType meterType) const
{
    const tmbus::ModbusRegisters::Register* reg = _modbusRegisters->getRegister(meterType);
    return reg;
}

const std::pair<uint16_t, uint16_t> IModbusDevice::getallIR() const
{
    return {RIR_BEGIN, RIR_LENGTH};
};
const std::pair<uint16_t, uint16_t> IModbusDevice::getallHR() const
{
    return {RHR_BEGIN, RHR_LENGTH};
};
std::optional<uint16_t> IModbusDevice::getMatchingStartAddress(uint8_t cmd) const
{
    for(size_t i = 0; i < MAX_COMMANDS; ++i)
    {
        size_t index = (_commandIndex + MAX_COMMANDS - 1 - i) % MAX_COMMANDS; // Wrap around
        uint8_t lastCmd = _lastSendCommands[index].first;

        // Match the command and return the corresponding start address
        if(lastCmd == cmd)
        {
            return _lastSendCommands[index].second; // Return the matching start address
        }
    }

    return std::nullopt; // No match found
}
void IModbusDevice::manageList(uint8_t cmd, uint16_t regAddr)
{
    // Check for duplicates
    for(size_t i = 0; i < MAX_COMMANDS; ++i)
    {
        if(_lastSendCommands[i].first == cmd && _lastSendCommands[i].second == regAddr)
        {
            return;
        }
    }

    // Add new command
    _lastSendCommands[_commandIndex] = std::make_pair(cmd, regAddr);
    _commandIndex = (_commandIndex + 1) % MAX_COMMANDS; // Wrap around if needed
}

} // namespace pzemCore