#include "PZEMDevice.hpp"
#include "UARTMsgQueue.hpp"

namespace pzemCore
{
PZEMDevice::PZEMDevice(PZEMModel model, uint8_t id, uint8_t addr, uint8_t lineNo, Phase phase,
                       const char* nametag) :
    PZEMClient(id),
    IModbusDevice(id, model),
    PowerMeter(getModbusRegisters(), model, id, addr, lineNo, phase, nametag),
    _model(model),
    _id(id),
    _addr(addr),
    _cfg(),
    _rtu(this, _addr),
    modbusRegisters(getModbusRegisters()),
    _updateTaskHandle(nullptr),
    _stopTask(false)
{
    PZEMDevice::init();
}

PZEMDevice::~PZEMDevice()
{
    if(_updateTaskHandle)
    {
        _stopTask.store(true);          // Signal the task to stop
        vTaskDelay(pdMS_TO_TICKS(100)); // Give the task time to exit
        _updateTaskHandle = nullptr;
        if(_updateSemaphore)
        {
            vSemaphoreDelete(_updateSemaphore); // Clean up the semaphore
        }
        ESP_LOGI(DEVICE_TAG, "updateMeters task stopped.");
    }
}
void PZEMDevice::init()
{
    if(!_initialised)
    {
        if(_model == PZEMModel::PZEM003 || _model == PZEMModel::PZEM004T)
        {
            isBigEndian = true;
        }
        ESP_LOGI(DEVICE_TAG, "Initializing PZEMDevice with ID: %d, Address: %d", _id, _addr);

        if(xTaskCreatePinnedToCore(
               [](void* param) { static_cast<PZEMDevice*>(param)->updateMetersTask(); },
               "PZEM_UpdateMeters", 4096, this, 1, &_updateTaskHandle, 1)
           == pdPASS)
        {
            ESP_LOGI(DEVICE_TAG, "updateMeters task created successfully.");
        }
        else
        {
            ESP_LOGW(DEVICE_TAG, "Failed to create updateMeters task.");
        }
        handleEvent(Event::NEW_DEVICE);
        _initialised = true;
    }
}

uint8_t PZEMDevice::getaddr() const { return _addr; }

void PZEMDevice::rx_sink(const tmbus::receive* msg)
{
    if(parse_RXMessage(msg))
    {
        if(rx_callback) rx_callback(id, msg);
    }
}
void PZEMDevice::updateMetersTask()
{
    ESP_LOGI(DEVICE_TAG, "Starting updateMeters task.");

    while(!_stopTask.load())
    {
        // Wait for the semaphore (i.e., a signal to update)
        if(xSemaphoreTake(_updateSemaphore, portMAX_DELAY) == pdTRUE)
        {
            ESP_LOGI(DEVICE_TAG, "Semaphore received, updating meters...");
            // Perform the update
            updateMeters();
            updateJobCard();
            vTaskDelay(pdMS_TO_TICKS(1000)); // Adjust delay as required
        }
    }
    ESP_LOGI(DEVICE_TAG, "Exiting updateMeters task.");
    vTaskDelete(nullptr); // Delete the task when finished
}
bool PZEMDevice::updateMeters()
{
    if(!txMsgQueue)
    {
        return false;
    }
    handleEvent(Event::READING_IR);
    transmit* command = readInputRegisters();
    resetPoll();
    txMsgQueue->txEnQueue(command);
    return true;
}

bool PZEMDevice::resetEnergyCounter()
{
    if(!txMsgQueue)
    {
        return false;
    }

    transmit* command = resetEnergy();
    txMsgQueue->txEnQueue(command);
    return true;
}

bool PZEMDevice::isValidAddress(uint8_t address)
{
    uint8_t minRange = static_cast<uint8_t>(SLAVE_ADDRESS::ADDR_MIN);
    uint8_t maxRange = static_cast<uint8_t>(SLAVE_ADDRESS::ADDR_MAX);
    if(address > minRange && address < maxRange)
    {
        return true;
    }
    return false;
}
bool PZEMDevice::isValidThreshHold(uint16_t value)
{
    if(value < _cfg.maxWatt)
    {
        return true;
    }
    return false;
}

transmit* PZEMDevice::getModbusAddress() { return readSingleRegister(MeterType::SLAVE_ADDRESS); }
transmit* PZEMDevice::getAlarmThreshold() { return readSingleRegister(MeterType::ALARM_THRESHOLD); }

transmit* PZEMDevice::setAlarmThreshold(uint16_t value)
{
    uint16_t newValue;
    if(isValidThreshHold(value))
    {
        newValue = value;
    }
    else
    {
        newValue = _cfg.alarmThresh;
    }
    return writeSingleRegister(MeterType::ALARM_THRESHOLD, newValue);
}

transmit* PZEMDevice::setModbusAddress(uint8_t newAddr)
{
    uint8_t changedAddress;
    if(isValidAddress(newAddr))
    {
        changedAddress = newAddr;
    }
    else
    {
        changedAddress = _addr;
    }
    return writeSingleRegister(MeterType::SLAVE_ADDRESS, changedAddress);
}

transmit* PZEMDevice::resetEnergy()
{
    const tmbus::ModbusRegisters::Register* REG = getRegister(MeterType::RESET);

    return _rtu.sendEnergyResetMsg(_addr, REG->startAddress);
}
transmit* PZEMDevice::setShunt(shunt_t shunt)
{
    if(_model == PZEMModel::PZEM003)
    {
        uint8_t value = static_cast<uint8_t>(shunt);
        return writeSingleRegister(MeterType::CURRENT_RANGE, value);
    }
    return nullptr;
}

transmit* PZEMDevice::readSingleRegister(tmbus::MeterType meterType, bool waitForReply)
{
    const tmbus::ModbusRegisters::Register* Reg = getRegister(meterType);
    FunctionCode code = FunctionCode::READ_INPUT_REG;
    if(Reg == nullptr)
    {
        return nullptr;
    }
    if(Reg->info->regType == RegisterType::HR)
    {
        code = FunctionCode::READ_HOLDING_REG;
    }

    return _rtu.sendMsg(code, Reg->startAddress, Reg->length, _addr, waitForReply);
}

transmit* PZEMDevice::writeSingleRegister(tmbus::MeterType meterType, uint16_t value,
                                          bool waitForReply)
{
    const tmbus::ModbusRegisters::Register* Reg = getRegister(meterType);
    FunctionCode code = FunctionCode::READ_INPUT_REG;
    if(Reg == nullptr)
    {
        return nullptr;
    }
    if(Reg->info->regType == RegisterType::HR)
    {
        code = FunctionCode::WRITE_SINGLE_REG;
    }
    else if(Reg->info->regType == RegisterType::PRG)
    {
        code = FunctionCode::WRITE_SINGLE_REG;
    }
    else
    {
        ESP_LOGE(DEVICE_TAG, " wrong meter Type executing read command ");
        code = FunctionCode::READ_INPUT_REG;
    }
    return _rtu.sendMsg(code, Reg->startAddress, value, _addr, waitForReply);
}

transmit* PZEMDevice::readInputRegisters()
{
    bool waitForReply = true;
    auto [startAddress, length] = getallIR();
    return _rtu.sendMsg(FunctionCode::READ_INPUT_REG, startAddress, length, _addr, waitForReply);
}
transmit* PZEMDevice::readHoldingRegisters()
{
    bool waitForReply = true;
    auto [startAddress, length] = getallHR();
    return _rtu.sendMsg(FunctionCode::READ_HOLDING_REG, startAddress, length, _addr, waitForReply);
}

bool PZEMDevice::parse_RXMessage(const receive* rxMessage)
{
    if(!rxMessage->isValid) return false;
    if(rxMessage->addr != _addr) return false;

    FunctionCode code = static_cast<FunctionCode>(rxMessage->cmd);
    const uint8_t* data = rxMessage->msg.getValidRawData();
    const uint8_t length = rxMessage->size;

    switch(code)
    {
        case FunctionCode::READ_INPUT_REG: {
            if(parseDataByRegister(data, length, RegisterType::IR))
                return true;
            else
                return false;
        }
        case FunctionCode::READ_HOLDING_REG:
            if(parseDataByRegister(data, length, RegisterType::HR))
                return true;
            else
                return false;

        case FunctionCode::WRITE_SINGLE_REG:
            if(modbusRegisters->updateSingleWrite(data, isBigEndian))
                return true;
            else
                return false;

        case FunctionCode::RESET:
            Serial.println("RESET not implemented");
            return true;

        case FunctionCode::CALIBRATE:
            Serial.println("CALIBRATE not implemented");
            return true;

        case FunctionCode::CMD_RERR:
        case FunctionCode::CMD_WERR:
        case FunctionCode::CMD_RSTERR:
        case FunctionCode::CMD_CALERR:
            Serial.println("Error handling not implemented");
            return true;

        default:
            Serial.println("Unknown function code");
            return false;
    }

    return false; // Fallback for safety, in case no case was matched
}

bool PZEMDevice::parseDataByRegister(const uint8_t* data, const uint8_t length,
                                     RegisterType regType)
{
    const uint8_t fullLength = (regType == RegisterType::IR) ? RIR_MAX_RESP_LENGTH : RHR_LENGTH;

    Update updateType = (length == fullLength)               ? Update::FULL :
                        (length < fullLength && length != 1) ? Update::PARTIAL :
                        (length == 1)                        ? Update::SINGLE :
                                                               Update::INVALID;

    if(updateType == Update::INVALID)
    {
        return false; // Invalid length
    }

    if(updateType == Update::FULL)
    {
        return updateRegisters(regType, Update::FULL, data, length, 0);
    }
    else
    {
        FunctionCode lastSendCommand =
            (regType == RegisterType::IR ? FunctionCode::READ_INPUT_REG :
                                           FunctionCode::READ_HOLDING_REG);
        auto startAddressOpt = getMatchingStartAddress(static_cast<uint8_t>(lastSendCommand));
        if(!startAddressOpt.has_value()) return false;

        return updateRegisters(regType, updateType, data, length, startAddressOpt.value());
    }
}

bool PZEMDevice::updateRegisters(RegisterType regType, Update updateType, const uint8_t* data,
                                 const uint8_t length, const uint16_t startAddress)
{
    uint16_t baseAddress = (regType == RegisterType::IR) ? RIR_BEGIN : RHR_BEGIN;
    uint16_t numRegs = (regType == RegisterType::IR) ? numRIR : numRHR;

    switch(updateType)
    {
        case Update::FULL:
            modbusRegisters->update(baseAddress, numRegs, data, isBigEndian);
            break;
        case Update::PARTIAL:
            modbusRegisters->update(startAddress, length, data, isBigEndian);
            break;
        case Update::SINGLE: {
            const ModbusRegisters::Register* reg = modbusRegisters->getRegister(startAddress);
            if(reg != nullptr)
            {
                modbusRegisters->updateSingle(reg->meterType, data, isBigEndian);
            }
            break;
        }
        default:
            return false;
    }
    return true;
}

} // namespace pzemCore