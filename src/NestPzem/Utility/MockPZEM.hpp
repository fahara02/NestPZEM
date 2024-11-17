#ifndef MOCK_PZEM_HPP
#define MOCK_PZEM_HPP
#include <array>
#include <cstdlib>
#include <cmath>
#include <ctime>
#include <cstdint>
#include <vector>
#include <algorithm>
#include <esp_timer.h>
#include <memory>
#include "PZEMDevice.hpp"
#include "ModbusDefaults.h"
#include "esp_log.h"
using namespace pzemCore;
using namespace tmbus;

namespace Utility
{

class FakeMeter
{
   public:
    enum class FakeMeterType : uint8_t {
        VOLTAGE,
        CURRENT_16, // 16-bit current
        CURRENT_32, // 32-bit current (for AC)
        POWER,
        ENERGY,
        FREQUENCY,    // Optional (skipped for DC)
        POWER_FACTOR, // Optional (skipped for DC)
        ALARM_STATUS,
        COUNT // Helper to track the number of types
    };
    enum class DeviceState { IDLE, NORMAL, OVERLOAD };

   private:
    uint8_t slaveAddress;
    bool isDCModel;
    DeviceState currentState;
    int64_t lastUpdate; // Use esp_timer_get_time() for timing in microseconds

    struct Range
    {
        uint32_t min;
        uint32_t max;
    };

    std::array<Range, static_cast<size_t>(FakeMeterType::COUNT)> _ranges;
    std::array<uint32_t, static_cast<size_t>(FakeMeterType::COUNT)> _currentValues;
    std::array<uint32_t, static_cast<size_t>(FakeMeterType::COUNT)> _deviation;
    std::array<float, static_cast<size_t>(FakeMeterType::COUNT)> _randomizationProbability;

   public:
    FakeMeter(uint8_t slaveAddr, bool isDC = false);

    std::vector<uint8_t> generateMockData();
    void setInitialValues(
        const std::array<uint32_t, static_cast<size_t>(FakeMeterType::COUNT)>& initialValues);
    void setDeviation(
        const std::array<uint32_t, static_cast<size_t>(FakeMeterType::COUNT)>& devValues);
    void setRandomizationProbability(
        const std::array<float, static_cast<size_t>(FakeMeterType::COUNT)>& probabilities);
    void setDeviceState(DeviceState state);
    uint32_t getCurrentValues(FakeMeterType type) const;

   private:
    void updateValues();
    uint32_t applyDeviation(uint32_t value, uint32_t deviation, float probability);
    float applySinusoidalDeviation(uint32_t baseValue, uint32_t deviation, float timeStep);
    uint32_t addNoise(uint32_t value, uint32_t noiseLevel);
    void updateRangesBasedOnState();
    uint16_t calculateCRC(const std::vector<uint8_t>& data);
    void simulateFailure();
};

FakeMeter::FakeMeter(uint8_t slaveAddr, bool isDC) : slaveAddress(slaveAddr), isDCModel(isDC)
{
    _ranges = {
        Range{isDC ? 0 : 800u, isDC ? 3000u : 2600u}, // Voltage
        Range{0u, isDC ? 30000u : 100000u},           // 16-bit current for DC, 32-bit for AC
        Range{0u, isDC ? 30000u : 100000u},           // 32-bit current for AC
        Range{0u, isDC ? 900000u : 230000u},          // Power
        Range{0u, isDC ? 9999u : 9999u},              // Energy
        Range{isDC ? 0 : 450u, isDC ? 0 : 650u},      // Frequency (skipped for DC)
        Range{0u, 100u},                              // Power Factor (skipped for DC)
        Range{0u, 1u}                                 // Alarm Status
    };

    _currentValues = {2300, 1500, 3450, 3450, 0, 600, 90, 0};
    _deviation = {10, 20, 20, 5, 10, 10, 10, 1};
    _randomizationProbability = {0.2, 0.3, 0.3, 0.1, 0.05, 0.1, 0.1, 0.05};

    std::srand(static_cast<unsigned>(esp_timer_get_time())); // Seed the random number generator
}

void FakeMeter::setInitialValues(
    const std::array<uint32_t, static_cast<size_t>(FakeMeterType::COUNT)>& initialValues)
{
    _currentValues = initialValues;
}

void FakeMeter::setDeviation(
    const std::array<uint32_t, static_cast<size_t>(FakeMeterType::COUNT)>& devValues)
{
    _deviation = devValues;
}
void FakeMeter::setDeviceState(DeviceState state)
{
    currentState = state;
    updateRangesBasedOnState();
}

void FakeMeter::setRandomizationProbability(
    const std::array<float, static_cast<size_t>(FakeMeterType::COUNT)>& probabilities)
{
    _randomizationProbability = probabilities;
}

uint32_t FakeMeter::applyDeviation(uint32_t value, uint32_t deviation, float probability)
{
    if((std::rand() % 100) / 100.0f < probability)
    {
        int32_t dev = std::rand() % (deviation * 2) - deviation; // +/- deviation
        return value + dev;
    }
    return value;
}
float FakeMeter::applySinusoidalDeviation(uint32_t baseValue, uint32_t deviation, float timeStep)
{
    // Simulate sinusoidal variation
    float angle = fmod(timeStep * 2.0f * M_PI, 2.0f * M_PI); // Normalize angle to [0, 2π]
    float sinValue = sin(angle);                             // Generate sinusoidal wave
    return baseValue + deviation * sinValue;                 // Add sinusoidal deviation
}
uint32_t FakeMeter::addNoise(uint32_t value, uint32_t noiseLevel)
{
    return value + (std::rand() % noiseLevel);
}

void FakeMeter::updateValues()
{
    uint64_t currentTime = esp_timer_get_time(); // ESP32-specific time function in microseconds
    float timeStep =
        (currentTime % 1000000) / 1000000.0f; // Normalize to seconds within 1-second period

    for(size_t i = 0; i < static_cast<size_t>(FakeMeterType::COUNT); ++i)
    {
        // Skip DC-only or AC-only parameters if necessary
        if(isDCModel
           && (i == static_cast<size_t>(FakeMeterType::FREQUENCY)
               || i == static_cast<size_t>(FakeMeterType::POWER_FACTOR)))
        {
            continue;
        }

        // Apply sinusoidal and random deviations
        _currentValues[i] = applySinusoidalDeviation(_currentValues[i], _deviation[i], timeStep);
        _currentValues[i] =
            applyDeviation(_currentValues[i], _deviation[i], _randomizationProbability[i]);

        // Clamp the values within the defined range
        _currentValues[i] = std::max(_ranges[i].min, std::min(_ranges[i].max, _currentValues[i]));
    }
}

// Generate mock data in the serial communication format
std::vector<uint8_t> FakeMeter::generateMockData()
{
    updateValues();

    std::vector<uint8_t> mockData = {slaveAddress, 0x04, 0x14};

    // Voltage
    mockData.push_back(static_cast<uint8_t>(_currentValues[0] >> 8));
    mockData.push_back(static_cast<uint8_t>(_currentValues[0] & 0xFF));

    // Current (16-bit for DC, 32-bit for AC)
    if(isDCModel)
    {
        mockData.push_back(static_cast<uint8_t>(_currentValues[1] >> 8)); // 16-bit current
        mockData.push_back(static_cast<uint8_t>(_currentValues[1] & 0xFF));
    }
    else
    {
        mockData.push_back(static_cast<uint8_t>(_currentValues[2] & 0xFF));
        mockData.push_back(static_cast<uint8_t>(_currentValues[2] >> 8));
        mockData.push_back(static_cast<uint8_t>(_currentValues[2] >> 24)); // 32-bit current for AC
        mockData.push_back(static_cast<uint8_t>(_currentValues[2] >> 16));
    }

    // Power and Energy
    // Power (32-bit)
    mockData.push_back(static_cast<uint8_t>(_currentValues[3] & 0xFF)); // Least significant byte
    mockData.push_back(static_cast<uint8_t>(_currentValues[3] >> 8));
    mockData.push_back(static_cast<uint8_t>(_currentValues[3] >> 24)); // Most significant byte
    mockData.push_back(static_cast<uint8_t>(_currentValues[3] >> 16));

    // Energy (32-bit)
    mockData.push_back(static_cast<uint8_t>(_currentValues[4] & 0xFF)); // Least significant byte
    mockData.push_back(static_cast<uint8_t>(_currentValues[4] >> 8));
    mockData.push_back(static_cast<uint8_t>(_currentValues[4] >> 24)); // Most significant byte
    mockData.push_back(static_cast<uint8_t>(_currentValues[4] >> 16));

    if(!isDCModel)
    {
        // Frequency (AC only)
        mockData.push_back(static_cast<uint8_t>(_currentValues[5] >> 8));
        mockData.push_back(static_cast<uint8_t>(_currentValues[5] & 0xFF));

        // Power Factor (AC only)
        mockData.push_back(static_cast<uint8_t>(_currentValues[6] >> 8));
        mockData.push_back(static_cast<uint8_t>(_currentValues[6] & 0xFF));
    }

    // Alarm Status
    mockData.push_back(static_cast<uint8_t>(_currentValues[7] >> 8));
    mockData.push_back(static_cast<uint8_t>(_currentValues[7] & 0xFF));

    uint16_t crc = calculateCRC(mockData);
    mockData.push_back(static_cast<uint8_t>(crc >> 8));
    mockData.push_back(static_cast<uint8_t>(crc & 0xFF));

    return mockData;
}
uint32_t FakeMeter::getCurrentValues(FakeMeterType type) const
{
    return _currentValues[static_cast<size_t>(type)];
}
void FakeMeter::updateRangesBasedOnState()
{
    switch(currentState)
    {
        case DeviceState::IDLE:
            // Example: Lower voltage and current ranges during idle
            _ranges[static_cast<size_t>(FakeMeterType::VOLTAGE)] = {1000, 1200};
            _ranges[static_cast<size_t>(FakeMeterType::CURRENT_32)] = {0, 500};
            break;
        case DeviceState::NORMAL:
            // Restore normal ranges
            _ranges[static_cast<size_t>(FakeMeterType::VOLTAGE)] = {800, 2600};
            _ranges[static_cast<size_t>(FakeMeterType::CURRENT_32)] = {0, 30000};
            break;
        case DeviceState::OVERLOAD:
            // Example: Increase power and current ranges
            _ranges[static_cast<size_t>(FakeMeterType::POWER)] = {500000, 900000};
            _ranges[static_cast<size_t>(FakeMeterType::CURRENT_32)] = {20000, 30000};
            break;
    }
}
uint16_t FakeMeter::calculateCRC(const std::vector<uint8_t>& data)
{
    uint16_t crc = 0xFFFF;
    for(uint8_t byte: data)
    {
        crc ^= byte;
    }
    return crc;
}

void FakeMeter::simulateFailure()
{
    // Simulates a failure by setting out-of-range or invalid values
    _currentValues.fill(0xFFFFFFFF); // Simulate device communication error or reset
}

void debugPrintMockData(const std::vector<uint8_t>& data)
{
    printf("Mock Data: ");
    for(uint8_t byte: data)
    {
        printf("%02X ", byte);
    }
    printf("\n");
}

class MockPZEM : public PZEMDevice
{
   public:
    MockPZEM(PZEMModel model, uint8_t id, uint8_t addr, uint8_t lineNo = 0,
             Phase phase = Phase::SINGLE_PHASE, const char* nametag = nullptr) :
        PZEMDevice(model, id, addr, lineNo, phase, nametag),
        _meter(addr, model != PZEMModel::PZEM004T)
    {
    }
    std::vector<uint8_t> sendRandomData() { return _meter.generateMockData(); }
    // Override updateMeters to update job card with mock data
    bool updateMeters() override
    {
        const uint16_t address = 0x0000; // Starting address
        const uint8_t numInputRegisters = 7;
        std::vector<uint8_t> mockData = _meter.generateMockData();
        uint8_t SerializeData[mockData.size()];
        std::copy(mockData.begin(), mockData.end(), SerializeData);

        TTLModbusMessage tmsg;
        tmsg.decodeMessage(SerializeData, sizeof(SerializeData), true);
        const uint8_t* data = tmsg.getValidRawData();
        std::shared_ptr<tmbus::ModbusRegisters> mReg1 = this->getModbusRegisters();
        if(mReg1.get()->update(address, numInputRegisters, data, true))
        {
            return true;
        }
        else
        {
            return false;
        }
    }

   private:
    FakeMeter _meter;
};

} // namespace Utility

#endif