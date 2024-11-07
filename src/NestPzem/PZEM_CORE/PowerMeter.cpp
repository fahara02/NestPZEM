#include "PowerMeter.hpp"
#include "ModbusDefaults.h"
using namespace tmbus;
namespace pzemCore
{
const JobCard& PowerMeter::getJobCard() const { return _jobcard; }
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
    // updateMeters();
    updateJobCard();
}
const pzemCore::powerMeasure& PowerMeter::getMeasures() const
{
    _registers->getPowerMeasures(_jobcard.pm);
    return _jobcard.pm;
}

bool PowerMeter::updateJobCard() const
{
    const powerMeasure& pm = getMeasures();

    if(pm.isValid())
    {
        _jobcard.lastUpdate_us = esp_timer_get_time();

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

// void JobCard::print()
// {
//     // Print the JobCard details
//     ESP_LOGI(PM_TAG, "+---------------------------------------------------+");
//     ESP_LOGI(PM_TAG, "|                  JobCard Details                  |");
//     ESP_LOGI(PM_TAG, "+----------------+----------------------------------+");
//     ESP_LOGI(PM_TAG, "| Field           | Value                           |");
//     ESP_LOGI(PM_TAG, "+----------------+----------------------------------+");
//     ESP_LOGI(PM_TAG, "| Model           | %-31s |", Utility::ToString::model(info.model));
//     ESP_LOGI(PM_TAG, "| ID              | %-31d |", info.id);
//     ESP_LOGI(PM_TAG, "| Slave Address   | %-31d |", info.slaveAddress);
//     ESP_LOGI(PM_TAG, "| Line Number     | %-31d |", info.lineNo);
//     ESP_LOGI(PM_TAG, "| Phase           | %-31s |", Utility::ToString::Phase(info.phase));
//     ESP_LOGI(PM_TAG, "| Meter Name      | %-31s |",
//              info.meterName ? info.meterName.get() : "Unnamed");
//     ESP_LOGI(PM_TAG, "+----------------+----------------------------------+");

//     // Print the PowerMeasure details
//     pm.print();

//     // Print the state and timing information
//     ESP_LOGI(PM_TAG, "+----------------+----------------------------------+");
//     ESP_LOGI(PM_TAG, "| Poll Time (us)  | %-31lld |", poll_us);
//     ESP_LOGI(PM_TAG, "| Last Update (us)| %-31lld |", lastUpdate_us);
//     ESP_LOGI(PM_TAG, "| Data Age (ms)   | %-31lld |", dataAge_ms);
//     ESP_LOGI(PM_TAG, "| Data Stale      | %-31s |", dataStale ? "Yes" : "No");
//     ESP_LOGI(PM_TAG, "| Device State    | %-31s |",
//     Utility::ToString::State(deviceState.load())); ESP_LOGI(PM_TAG,
//     "+---------------------------------------------------+");
// }
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