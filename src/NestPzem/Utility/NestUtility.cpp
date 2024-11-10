#include "NestUtility.hpp"
#include "UtilityConstants.hpp"
#include <cstdio>
#include <cstring>
#include <cstdint>
#include "esp_log.h"
using namespace tmbus;
using namespace pzemCore;





std::array<std::pair<tmbus::MeterType, tmbus::MeterInfo>, tmbus::METER_MAP_004T_SIZE>
    Utility::MeterMap::_meterMap_004T;
std::array<std::pair<tmbus::MeterType, tmbus::MeterInfo>, tmbus::METER_MAP_003_SIZE>
    Utility::MeterMap::_meterMap_003;

uint16_t Utility::CRC::calculate(const uint8_t* data, uint16_t len)
{
    uint8_t nTemp;
    uint16_t crc = 0xFFFF;
    while(len--)
    {
        nTemp = (*data++ ^ crc) & 0xFF;
        crc = (uint16_t)CRC16_REFLECTED_MODBUS_TABLE[nTemp] ^ (crc >> 8);
    }
    return crc;
}
bool Utility::CRC::check(const uint8_t* buf, uint16_t len)
{
    if(len < 5) return false;
    uint16_t receivedCRC = *(uint16_t*)&buf[len - 2];
    return receivedCRC == calculate(buf, len - 2);
}
void Utility::CRC::set(uint8_t* buf, uint16_t len)
{
    if(len <= 2) return;
    uint16_t crcVal = calculate(buf, len - 2);
    buf[6] = __builtin_bswap16(*(uint16_t*)&crcVal);
}

void Utility::MeterMap::loadMeterInfo()
{
    ESP_LOGI("Utility::MeterMap", "Creating Meter Map for PZEM004T...");
    Utility::MeterMap::_meterMap_004T = {
        {{tmbus::MeterType::VOLTAGE,
          {tmbus::RegisterType::IR, tmbus::OperationType::READONLY, 0x0000, 1, 10, "Voltage", "V"}},
         {tmbus::MeterType::CURRENT,
          {tmbus::RegisterType::IR, tmbus::OperationType::READONLY, 0x0001, 2, 1000, "Current",
           "A"}},
         {tmbus::MeterType::POWER,
          {tmbus::RegisterType::IR, tmbus::OperationType::READONLY, 0x0003, 2, 10, "Power", "W"}},
         {tmbus::MeterType::ENERGY,
          {tmbus::RegisterType::IR, tmbus::OperationType::READONLY, 0x0005, 2, 1, "Energy", "Wh"}},
         {tmbus::MeterType::FREQUENCY,
          {tmbus::RegisterType::IR, tmbus::OperationType::READONLY, 0x0007, 1, 10, "Frequency",
           "Hz"}},
         {tmbus::MeterType::PF,
          {tmbus::RegisterType::IR, tmbus::OperationType::READONLY, 0x0008, 1, 100, "Power Factor",
           ""}},
         {tmbus::MeterType::ALARM_STATUS,
          {tmbus::RegisterType::IR, tmbus::OperationType::READONLY, 0x0009, 1, 1, "Alarm Status",
           ""}},
         {tmbus::MeterType::ALARM_THRESHOLD,
          {tmbus::RegisterType::HR, tmbus::OperationType::READ_WRITE, 0x0001, 1, 1,
           "Alarm Threshold", ""}},
         {tmbus::MeterType::SLAVE_ADDRESS,
          {tmbus::RegisterType::HR, tmbus::OperationType::READ_WRITE, 0x0002, 1, 1, "Slave Address",
           ""}},
         {tmbus::MeterType::CALIBRATE,
          {tmbus::RegisterType::PRG, tmbus::OperationType::WRITEONLY, 0x3721, 1, 1, "Calibrate",
           ""}}}};
    meterMap_004T_loaded = true;
    ESP_LOGI("Utility::MeterMap", "Meter Map for PZEM004T created.");

    ESP_LOGI("Utility::MeterMap", "Creating Meter Map for PZEM003...");
    Utility::MeterMap::_meterMap_003 = {
        {{tmbus::MeterType::VOLTAGE,
          {tmbus::RegisterType::IR, tmbus::OperationType::READONLY, 0x0000, 1, 10, "Voltage", "V"}},
         {tmbus::MeterType::CURRENT,
          {tmbus::RegisterType::IR, tmbus::OperationType::READONLY, 0x0001, 1, 100, "Current",
           "A"}},
         {tmbus::MeterType::POWER,
          {tmbus::RegisterType::IR, tmbus::OperationType::READONLY, 0x0002, 2, 10, "Power", "W"}},
         {tmbus::MeterType::ENERGY,
          {tmbus::RegisterType::IR, tmbus::OperationType::READONLY, 0x0004, 2, 1, "Energy", "Wh"}},
         {tmbus::MeterType::ALARM_STATUS,
          {tmbus::RegisterType::IR, tmbus::OperationType::READONLY, 0x0006, 2, 1, "Alarm Status",
           ""}},
         {tmbus::MeterType::HV_ALARM_THRESHOLD,
          {tmbus::RegisterType::HR, tmbus::OperationType::READ_WRITE, 0x0000, 1, 10,
           "HV Alarm Threshold", ""}},
         {tmbus::MeterType::LV_ALARM_THRESHOLD,
          {tmbus::RegisterType::HR, tmbus::OperationType::READ_WRITE, 0x0001, 1, 10,
           "LV Alarm Threshold", ""}},
         {tmbus::MeterType::SLAVE_ADDRESS,
          {tmbus::RegisterType::HR, tmbus::OperationType::READ_WRITE, 0x0002, 1, 1, "Slave Address",
           ""}},
         {tmbus::MeterType::CURRENT_RANGE,
          {tmbus::RegisterType::PRG, tmbus::OperationType::WRITEONLY, 0x0003, 1, 1, "Range", ""}},
         {tmbus::MeterType::CALIBRATE,
          {tmbus::RegisterType::PRG, tmbus::OperationType::WRITEONLY, 0x3721, 1, 1, "Calibrate",
           ""}}}};
    meterMap_003_loaded = true;
    ESP_LOGI("Utility::MeterMap", "Meter Map for PZEM003 created.");
}

std::array<std::pair<tmbus::MeterType, tmbus::MeterInfo>, tmbus::METER_MAP_004T_SIZE>*
Utility::MeterMap::getMapReference(pzemCore::PZEMModel model) const
{
    switch(model)
    {
        case pzemCore::PZEMModel::PZEM004T:
            if(meterMap_004T_loaded)
            {
                return &_meterMap_004T;
            }
            break;
        case pzemCore::PZEMModel::PZEM003:
            if(meterMap_003_loaded)
            {
                return &_meterMap_003;
            }
            break;
        default:
            ESP_LOGE("Utility::MeterMap", "Unsupported model");
            return nullptr;
    }
    if(_meterMap_004T.empty())
    {
        ESP_LOGE("Utility::MeterMap", "Failed to load meter map for PZEM004T");
    }
    if(_meterMap_003.empty())
    {
        ESP_LOGE("Utility::MeterMap", "Failed to load meter map for PZEM003");
    }

    ESP_LOGW("Utility::MeterMap", "Map for the requested model is not loaded");
    return nullptr;
}

std::array<std::pair<tmbus::MeterType, tmbus::MeterInfo>, tmbus::METER_MAP_004T_SIZE>*
Utility::MeterMap::sendMapReference(pzemCore::PZEMModel model)
{
    switch(model)
    {
        case pzemCore::PZEMModel::PZEM004T:
            if(!meterMap_004T_loaded)
            {
                ESP_LOGW("Utility::MeterMap",
                         "Meter map for PZEM004T is not loaded. Loading it now...");
                loadMeterInfo(); // Ensure the map is loaded
            }
            return &_meterMap_004T;
        case pzemCore::PZEMModel::PZEM003:
            if(!meterMap_003_loaded)
            {
                ESP_LOGW("Utility::MeterMap",
                         "Meter map for PZEM003 is not loaded. Loading it now...");
                loadMeterInfo(); // Ensure the map is loaded
            }
            return &_meterMap_003;
        default:
            ESP_LOGE("Utility::MeterMap", "Unsupported model");
            return nullptr;
    }
}

void Utility::MeterMap::printMeterInfo()
{
    Utility::PRINT print;

    Serial.println("Meter Map 004T:");
    for(const auto& entry: _meterMap_004T)
    {
        print.map(entry.first, entry.second);
    }

    // Print the content of meterMap_003
    Serial.println("Meter Map 003:");
    for(const auto& entry: _meterMap_003)
    {
        print.map(entry.first, entry.second);
    }
}

const char* registerTypeToString(RegisterType regType)
{
    switch(regType)
    {
        case RegisterType::IR:
            return "IR";
        case RegisterType::HR:
            return "HR";
        case RegisterType::PRG:
            return "PRG";
        default:
            return "NONE";
    }
}

// Convert OperationType to string
const char* operationTypeToString(OperationType opType)
{
    switch(opType)
    {
        case OperationType::READONLY:
            return "READONLY";
        case OperationType::WRITEONLY:
            return "WRITEONLY";
        case OperationType::READ_WRITE:
            return "READ_WRITE";
        default:
            return "NONE";
    }
}

const char* Utility::ToString::model(pzemCore::PZEMModel model)
{
    const char* modelStr;
    switch(model)
    {
        case PZEMModel::PZEM004T:
            modelStr = "PZEM004T";
            break;
        case PZEMModel::PZEM003:
            modelStr = "PZEM003";
            break;
        case PZEMModel::NOT_SET:
            modelStr = "NOT_SET";
            break;
        default:
            modelStr = "ERROR";
            break;
    }
    return modelStr;
}

const char* Utility::ToString::meter(tmbus::MeterType meter)
{
    const char* meterStr;
    switch(meter)
    {
        case tmbus::MeterType::VOLTAGE:
            meterStr = "Voltage";
            break;
        case tmbus::MeterType::CURRENT:
            meterStr = "Current";

            break;
        case tmbus::MeterType::POWER:
            meterStr = "Power";
            break;
        case tmbus::MeterType::ENERGY:
            meterStr = "Energy";
            break;
        case tmbus::MeterType::FREQUENCY:
            meterStr = "Frequency";
            break;
        case tmbus::MeterType::PF:
            meterStr = "Power Factor";
            break;
        case tmbus::MeterType::ALARM_STATUS:
            meterStr = "Alarm Status";
            break;
        case tmbus::MeterType::ALARM_THRESHOLD:
            meterStr = "Alarm Threshold";
            break;
        case tmbus::MeterType::SLAVE_ADDRESS:
            meterStr = "Slave Address";
            break;
        case tmbus::MeterType::CALIBRATE:
            meterStr = "Calibrate";
            break;
        default:
            meterStr = "Unknown Meter Type";
            break;
    }
    return meterStr;
}
const char* Utility::ToString::regType(tmbus::RegisterType regType)
{
    const char* regStr;
    switch(regType)
    {
        case tmbus::RegisterType::IR:
            regStr = "Input Register";
            break;
        case tmbus::RegisterType::HR:
            regStr = "Holding Register";
            break;

        case tmbus::RegisterType::PRG:
            regStr = "Programming Register";
            break;
        default:
            regStr = "Unknown Register";
            break;
    }
    return regStr;
}
const char* Utility::ToString::opType(tmbus::OperationType opType)
{
    const char* opStr;
    switch(opType)
    {
        case tmbus::OperationType::READONLY:
            opStr = "Read-Only";
            break;
        case tmbus::OperationType::WRITEONLY:
            opStr = "Write-Only";
            break;
        case tmbus::OperationType::READ_WRITE:
            opStr = "Read/Write";
            break;
        default:
            opStr = "Unknown Operation";
            break;
    }

    return opStr;
}
const char* Utility::ToString::State(pzemCore::State state)
{
    const char* stateStr;
    switch(state)
    {
        case pzemCore::State::INIT:
            stateStr = "INIT";
            break;
        case pzemCore::State::ACTIVE:
            stateStr = "ACTIVE";
            break;
        case pzemCore::State::IDLE:
            stateStr = "IDLE";
            break;
        case pzemCore::State::BUSY:
            stateStr = "BUSY";
            break;
        case pzemCore::State::POLL:
            stateStr = "POLLING";
            break;
        case pzemCore::State::UPDATE_FAIL:
            stateStr = "UPDATE FAIL";
            break;
        case pzemCore::State::NOLOAD:
            stateStr = "NO LOAD";
            break;
        case pzemCore::State::NO_VOLTAGE:
            stateStr = "NO VOLTAGE";
            break;
        default:
            stateStr = "UNKNOWN";
            break;
    }

    return stateStr;
}
const char* Utility::ToString::Phase(pzemCore::Phase phase)
{
    const char* phaseStr;
    switch(phase)
    {
        case pzemCore::Phase::SINGLE_PHASE:
            phaseStr = "SINGLE PHASE";
            break;
        case pzemCore::Phase::RED_PHASE:
            phaseStr = "RED PHASE";
            break;
        case pzemCore::Phase::YELLOW_PHASE:
            phaseStr = "YELLOW PHASE";
            break;
        case pzemCore::Phase::BLUE_PHASE:
            phaseStr = "BLUE PHASE";
            break;
        default:
            phaseStr = "UNKNOWN PHASE";
            break;
    }

    return phaseStr;
}

void Utility::PRINT::map(const tmbus::MeterType meterType, const tmbus::MeterInfo& map)
{
    Serial.printf("Meter: %s\n", Utility::ToString::meter(meterType));
    Serial.printf("Register Type:%s \n", Utility::ToString::regType(map.regType));
    Serial.printf("Operation Type:%s \n ", Utility::ToString::opType(map.opType));

    // Print Start Address
    Serial.print("Start Address: 0x");
    Serial.printf("%04X\n", map.startAddress);
    // Print Length
    Serial.print("Length: ");
    Serial.println(map.length);
    // Print Scale Factor
    Serial.print("Scale Factor: ");
    Serial.println(map.scaleFactor);
    // Print Meter Name
    Serial.print("Meter Name: ");
    Serial.println(map.name);
    // Print Unit
    Serial.print("Unit: ");
    Serial.println(map.unit);
    Serial.println("---------------------------------");
};

void Utility::PRINT::memory()
{
    uint32_t cpu_freq = esp_clk_cpu_freq() / 1000000; // CPU frequency in MHz
    uint32_t DRam = heap_caps_get_free_size(MALLOC_CAP_8BIT);
    uint32_t IRam =
        heap_caps_get_free_size(MALLOC_CAP_32BIT) - heap_caps_get_free_size(MALLOC_CAP_8BIT);

    uint32_t free_heap_size = esp_get_free_heap_size();
    uint32_t minimum_free_heap_size = esp_get_minimum_free_heap_size();
    uint32_t largest_free_block = heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);
    uint32_t stackmem = uxTaskGetStackHighWaterMark(NULL);

    ESP_LOGI(MEMORY_TAG, "CPU Frequency: %d MHz", cpu_freq);
    ESP_LOGI(MEMORY_TAG, "DRAM \t\t %d", DRam);
    ESP_LOGI(MEMORY_TAG, "IRam \t\t %d", IRam);
    ESP_LOGI(MEMORY_TAG, "free heap size = %d", free_heap_size);
    ESP_LOGI(MEMORY_TAG, "min free heap size = %d", minimum_free_heap_size);
    ESP_LOGI(MEMORY_TAG, "largest free block = %d", largest_free_block);
    // Heap Fragmentation
    float fragmentation = 100.0f - (largest_free_block * 100.0f / free_heap_size);
    ESP_LOGI(MEMORY_TAG, "Heap Fragmentation: %.2f%%", fragmentation);
    ESP_LOGI(MEMORY_TAG, "stack space = %d", stackmem);
    UBaseType_t numTasks = uxTaskGetNumberOfTasks();
    ESP_LOGI(MEMORY_TAG, "Number of Tasks: %d", numTasks);

    // Additional metrics

// PSRAM Info (if supported)
#if CONFIG_SPIRAM_SUPPORT
    uint32_t psram_size = esp_spiram_get_size();
    uint32_t free_psram_size = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
    ESP_LOGI(MEMORY_TAG, "PSRAM Size: %d bytes", psram_size);
    ESP_LOGI(MEMORY_TAG, "Free PSRAM: %d bytes", free_psram_size);
#endif
}
