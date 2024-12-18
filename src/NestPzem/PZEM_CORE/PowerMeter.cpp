#include "PowerMeter.hpp"
#include "ModbusDefaults.h"
using namespace tmbus;
namespace pzemCore
{
const JobCard& PowerMeter::getJobCard() const { return _jobcard; }
const std::array<uint16_t, PowerMeter::maxJobCardRegisters>& PowerMeter::getOutBox() const
{
    return _outBox;
}
const pzemCore::powerMeasure& PowerMeter::getMeasures() const { return _jobcard.pm; }
State PowerMeter::getState() const { return _state.load(); }
uint8_t PowerMeter::getaddr() const { return static_cast<uint8_t>(tmbus::SLAVE_ADDRESS::ADDR_ANY); }

void PowerMeter::onRegisterUpdated(bool registerUpdated, tmbus::ModbusRegisters::Register* reg)
{
    if(registerUpdated)
    {
        getMeter(reg);
    }
}
bool PowerMeter::getMeter(const tmbus::ModbusRegisters::Register* reg)
{
    bool success = false;
    if(reg->info == nullptr)
    {
        return false;
    }
    if(reg && reg->info->regType == tmbus::RegisterType::IR)
    {
        float scaledValue = static_cast<float>(reg->value) / static_cast<float>(reg->scaleFactor);

        success = _jobcard.pm.setField(reg->meterType, scaledValue);
    }
    return success;
}

void PowerMeter::updateMetrics()
{
    if(_updateSemaphore)
    {
        xSemaphoreGive(_updateSemaphore);
    }
    updateOutBox();
}

bool PowerMeter::updateOutBox()
{
    const JobCard& jobCard = getJobCard();
    uint16_t index = 0;

    // Map the NamePlate information
    _outBox[index++] = static_cast<uint16_t>(jobCard.info.model);
    _outBox[index++] = jobCard.info.id;
    _outBox[index++] = jobCard.info.slaveAddress;
    _outBox[index++] = jobCard.info.lineNo;
    _outBox[index++] = static_cast<uint16_t>(jobCard.info.phase);

    // Map meterName array (9 chars, packed as 16-bit registers)
    for(size_t i = 0; i < jobCard.info.meterName.size() && index < maxJobCardRegisters; i += 2)
    {
        uint16_t namePart = static_cast<uint16_t>(jobCard.info.meterName[i]) << 8;
        if(i + 1 < jobCard.info.meterName.size())
        {
            namePart |= jobCard.info.meterName[i + 1];
        }
        _outBox[index++] = namePart;
    }

    // Lambda to pack float as two 16-bit registers
    auto addFloatToRegisters = [&](float value) {
        uint32_t bits = *reinterpret_cast<const uint32_t*>(&value); // Standard reinterpret
        if(index + 2 <= maxJobCardRegisters)
        {
            _outBox[index++] = static_cast<uint16_t>((bits >> 16) & 0xFFFF); // High 16 bits
            _outBox[index++] = static_cast<uint16_t>(bits & 0xFFFF);         // Low 16 bits
        }
    };

    // Voltage, Current, Power, Energy, Frequency, Power Factor, Alarm Status
    addFloatToRegisters(jobCard.pm.getField(MeterType::VOLTAGE));
    addFloatToRegisters(jobCard.pm.getField(MeterType::CURRENT));
    addFloatToRegisters(jobCard.pm.getField(MeterType::POWER));
    addFloatToRegisters(jobCard.pm.getField(MeterType::ENERGY));
    addFloatToRegisters(jobCard.pm.getField(MeterType::FREQUENCY));
    addFloatToRegisters(jobCard.pm.getField(MeterType::PF));
    addFloatToRegisters(jobCard.pm.getField(MeterType::ALARM_STATUS));

    // Safely map a signed 64-bit value to four 16-bit registers (MSB first)
    auto addInt64ToRegisters = [&](int64_t value) {
        uint64_t safeValue =
            static_cast<uint64_t>(value < 0 ? 0 : value); // Default negative values to 0
        if(index + 4 <= maxJobCardRegisters)
        { // Ensure space for 4 registers
            _outBox[index++] =
                static_cast<uint16_t>((safeValue >> 48) & 0xFFFF); // Top 16 bits (MSB)
            _outBox[index++] = static_cast<uint16_t>((safeValue >> 32) & 0xFFFF); // Next 16 bits
            _outBox[index++] = static_cast<uint16_t>((safeValue >> 16) & 0xFFFF); // Next 16 bits
            _outBox[index++] = static_cast<uint16_t>(safeValue & 0xFFFF); // Bottom 16 bits (LSB)
        }
    };
    // Map the 64-bit fields
    addInt64ToRegisters(jobCard.poll_us);
    addInt64ToRegisters(jobCard.lastUpdate_us);
    addInt64ToRegisters(jobCard.dataAge_ms);
    // Additional fields with bounds checks

    _outBox[index++] = jobCard.dataStale ? 1 : 0;
    _outBox[index++] = static_cast<uint16_t>(jobCard.deviceState.load());

    // Ensure the entire process succeeded
    return index <= maxJobCardRegisters;
}

void PowerMeter::updateLatestMeasures() { _registers->getPowerMeasures(_jobcard.pm); }

bool PowerMeter::updateJobCard()
{
    updateLatestMeasures();
    const powerMeasure& pm = _jobcard.pm;

    if(pm.isValid())
    {
        _jobcard.lastUpdate_us = _jobcard.pm.lastMeasured();

        _jobcard.dataAge_ms = dataAge();
        _jobcard.dataStale = dataStale();
        // handleEvent(Event::UPDATE);
        return true;
    }
    return false;
}

void PowerMeter::handleEvent(Event e) {}

int64_t PowerMeter::dataAge() const
{
    return (esp_timer_get_time() - _jobcard.lastUpdate_us) / 1000;
}
void PowerMeter::resetPoll() const { _jobcard.poll_us = esp_timer_get_time(); }
bool PowerMeter::dataStale() const
{
    return (esp_timer_get_time() - _jobcard.lastUpdate_us > 3 * PZEM_REFRESH_PERIOD * 1000);
}

void PowerMeter::print() { _jobcard.print(); };

void JobCard::print()
{
    // Print the JobCard details
    ESP_LOGI(PM_TAG, "+---------------------------------------------------+");
    ESP_LOGI(PM_TAG, "|                  JobCard Details                  |");
    ESP_LOGI(PM_TAG, "+----------------+----------------------------------+");
    ESP_LOGI(PM_TAG, "| Field           | Value                           |");
    ESP_LOGI(PM_TAG, "+----------------+----------------------------------+");
    ESP_LOGI(PM_TAG, "| Model           | %-31s |", Utility::ToString::model(info.model));
    ESP_LOGI(PM_TAG, "| ID              | %-31d |", info.id);
    ESP_LOGI(PM_TAG, "| Slave Address   | %-31d |", info.slaveAddress);
    ESP_LOGI(PM_TAG, "| Line Number     | %-31d |", info.lineNo);
    ESP_LOGI(PM_TAG, "| Phase           | %-31s |", Utility::ToString::Phase(info.phase));
    ESP_LOGI(PM_TAG, "| Meter Name      | %-31s |", info.meterName.data());
    ESP_LOGI(PM_TAG, "+----------------+----------------------------------+");

    // Print the PowerMeasure details
    pm.print();

    // Print the state and timing information
    ESP_LOGI(PM_TAG, "+----------------+----------------------------------+");
    ESP_LOGI(PM_TAG, "| Poll Time (us)  | %-31lld |", poll_us);
    ESP_LOGI(PM_TAG, "| Last Update (us)| %-31lld |", lastUpdate_us);
    ESP_LOGI(PM_TAG, "| Data Age (ms)   | %-31lld |", dataAge_ms);
    ESP_LOGI(PM_TAG, "| Data Stale      | %-31s |", dataStale ? "Yes" : "No");
    ESP_LOGI(PM_TAG, "| Device State    | %-31s |", Utility::ToString::State(deviceState.load()));
    ESP_LOGI(PM_TAG, "+---------------------------------------------------+");
}

} // namespace pzemCore