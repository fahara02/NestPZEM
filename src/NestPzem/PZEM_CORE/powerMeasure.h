
#ifndef POWER_MEASURE_H
#define POWER_MEASURE_H

#include <stdint.h>
#include <limits>
#include <array>
#include <utility>
#include <tuple>
#include <esp_timer.h>
#include "PZEM_Defaults.h"
#include "Arduino.h"
#include "esp_log.h"
#include "NestUtility.hpp"

namespace pzemCore
{
static const char* PM_TAG __attribute__((unused)) = "pMeasure";

enum class powerType { AC, DC, NONE };

template <typename T>
struct Range
{
    T min;
    T max;

    Range(T min_v = std::numeric_limits<T>::min(), T max_v = std::numeric_limits<T>::max()) :
        min(min_v), max(max_v)
    {
    }

    bool isInRange(const T& value) const { return value >= min && value <= max; }
};

struct powerMeasure
{
   private:
    float voltage, current, power, energy, frequency, pf, alarms;
    PZEMModel _model;
    powerType type;
    bool _isValid;
    int64_t last_measured_us;
    std::array<Range<uint32_t>, 7> ranges;

   public:
    powerMeasure(PZEMModel model);
    powerMeasure(const std::array<float, 7>& data, PZEMModel model);

    bool isValid() const { return _isValid; }
    unsigned long lastMeasured() const { return last_measured_us; }
    std::tuple<uint32_t, uint32_t, uint32_t, uint32_t, uint32_t, uint32_t, uint32_t> getAllValues()
        const;
    bool setField(tmbus::MeterType field, float scaledValue);
    float getField(tmbus::MeterType field) const;
    void print();

   private:
    void initializeRanges();
};

inline powerMeasure::powerMeasure(PZEMModel model) :

    voltage(0),
    current(0),
    power(0),
    energy(0),
    frequency(0),
    pf(0),
    alarms(0),
    _model(model),
    _isValid(false),
    last_measured_us(0)
{
    initializeRanges();
}

inline powerMeasure::powerMeasure(const std::array<float, 7>& data, PZEMModel model) :
    powerMeasure(model)
{
    bool success = true;
    for(size_t i = 0; i < data.size(); ++i)
    {
        uint32_t scaledValue = data[i];

        success &= setField(static_cast<tmbus::MeterType>(i), scaledValue);
    }
    if(success)
    {
        _isValid = true;
        last_measured_us = esp_timer_get_time();
    }
}

inline void powerMeasure::initializeRanges()
{
    if(_model == PZEMModel::PZEM004T)
    {
        ranges = {
            Range<uint32_t>(80, 260),  // Voltage
            Range<uint32_t>(0, 100),   // Current
            Range<uint32_t>(0, 23000), // Power
            Range<uint32_t>(0, 9999),  // Energy
            Range<uint32_t>(45, 65),   // Frequency
            Range<uint32_t>(0, 1),     // Power Factor
            Range<uint32_t>(0, 1)      // Alarms
        };
        type = powerType::AC;
    }
    else if(_model == PZEMModel::PZEM003)
    {
        ranges = {
            Range<uint32_t>(0, 300),   // Voltage
            Range<uint32_t>(0, 300),   // Current
            Range<uint32_t>(0, 90000), // Power
            Range<uint32_t>(0, 9999),  // Energy
            Range<uint32_t>(0, 0),     // Frequency (not applicable for DC)
            Range<uint32_t>(0, 0),     // Power Factor
            Range<uint32_t>(0, 1)      // Alarms
        };
        type = powerType::DC;
    }
    else
    {
        ranges.fill(Range<uint32_t>(0, 0)); // Default to zero ranges
        type = powerType::NONE;
    }
}
inline bool powerMeasure::setField(tmbus::MeterType field, float scaledValue)
{
    auto fieldIndex = static_cast<size_t>(field);
    bool success = false;

    if(ranges[fieldIndex].isInRange(scaledValue))
    {
        switch(field)
        {
            case tmbus::MeterType::VOLTAGE:
                voltage = scaledValue;
                success = true;
                break;
            case tmbus::MeterType::CURRENT:
                current = scaledValue;
                success = true;
                break;
            case tmbus::MeterType::POWER:
                power = scaledValue;
                success = true;
                break;
            case tmbus::MeterType::ENERGY:
                energy = scaledValue;
                success = true;
                break;
            case tmbus::MeterType::FREQUENCY:
                frequency = scaledValue;
                success = true;
                break;
            case tmbus::MeterType::PF:
                pf = scaledValue;
                success = true;
                break;
            case tmbus::MeterType::ALARM_STATUS:
                alarms = scaledValue;
                success = true;
                break;
            default:
                ESP_LOGE(PM_TAG, "Unknown Field");
                return success;
        }
        if(success)
        {
            _isValid = true;
            last_measured_us = esp_timer_get_time();
        }
    }
    else
    {
        ESP_LOGE(PM_TAG, "Value %0.2f is out of range! for Field %s ", scaledValue,
                 Utility::ToString::meter(field));
    }
    return success;
}

inline float powerMeasure::getField(tmbus::MeterType field) const
{
    switch(field)
    {
        case tmbus::MeterType::VOLTAGE:
            return voltage;
        case tmbus::MeterType::CURRENT:
            return current;
        case tmbus::MeterType::POWER:
            return power;
        case tmbus::MeterType::ENERGY:
            return energy;
        case tmbus::MeterType::FREQUENCY:
            return frequency;
        case tmbus::MeterType::PF:
            return pf;
        case tmbus::MeterType::ALARM_STATUS:
            return alarms;
        default:
            return 0; // Handle unknown field
    }
}

inline std::tuple<uint32_t, uint32_t, uint32_t, uint32_t, uint32_t, uint32_t, uint32_t>
powerMeasure::getAllValues() const
{
    return std::make_tuple(voltage, current, power, energy, frequency, pf, alarms);
}

inline void powerMeasure::print()
{
    // Power measurements box matching JobCard's width
    ESP_LOGI(PM_TAG, "+---------------------------------------------------+");
    ESP_LOGI(PM_TAG, "|               Power Measurements                  |");
    ESP_LOGI(PM_TAG, "+----------------+----------------------------------+");
    ESP_LOGI(PM_TAG, "| Parameter       | Value                           |");
    ESP_LOGI(PM_TAG, "+----------------+----------------------------------+");
    ESP_LOGI(PM_TAG, "| Voltage         | %-6.2f V                        |", voltage);
    ESP_LOGI(PM_TAG, "| Current         | %-6.2f A                        |", current);
    ESP_LOGI(PM_TAG, "| Power           | %-6.2f W                        |", power);
    ESP_LOGI(PM_TAG, "| Energy          | %-6.2f Wh                       |", energy);
    ESP_LOGI(PM_TAG, "| Frequency       | %-6.2f Hz                       |", frequency);
    ESP_LOGI(PM_TAG, "| Power Factor    | %-6.2f                          |", pf);
    ESP_LOGI(PM_TAG, "| Alarms          | %-6.2f                          |", alarms);
    ESP_LOGI(PM_TAG, "+----------------+----------------------------------+");

    // Validity and last measured timestamp
    ESP_LOGI(PM_TAG, "| Valid:          | %-4s                            |",
             isValid() ? "Yes" : "No");
    ESP_LOGI(PM_TAG, "| Last Measured   | %-20lu us         |", lastMeasured());
    ESP_LOGI(PM_TAG, "+---------------------------------------------------+");
}

} // namespace pzemCore

#endif
