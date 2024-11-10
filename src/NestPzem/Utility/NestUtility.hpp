#ifndef NEST_UTILITY_HPP
#define NEST_UTILITY_HPP

#include <LittleFS.h>
#include <esp_log.h>
#include <esp_heap_caps.h>
#include <esp32/clk.h>
#include <cstdint>
#include <string.h>
#include <map>
#include <utility>
#include "PZEM_Defaults.h"
#include "ModbusDefaults.h"

static const char* MEMORY_TAG __attribute__((unused)) = "MEMORY";
namespace Utility
{
struct PairHash
{
    template <class T1, class T2>
    std::size_t operator()(const std::pair<T1, T2>& pair) const
    {
        return std::hash<T1>()(pair.first) ^ std::hash<T2>()(pair.second);
    }
};

class CRC
{
   public:
    static uint16_t calculate(const uint8_t* data, uint16_t len);
    static bool check(const uint8_t* buf, uint16_t len);
    static void set(uint8_t* buf, uint16_t len);
};

class LOG
{
   public:
    void log();
};
class ToString
{
   public:
    static const char* model(pzemCore::PZEMModel model);
    static const char* meter(tmbus::MeterType meter);
    static const char* regType(tmbus::RegisterType regType);
    static const char* opType(tmbus::OperationType opType);
    static const char* State(pzemCore::State state);
    static const char* Phase(pzemCore::Phase phase);
};
class PRINT
{
   public:
    static void map(const tmbus::MeterType meterType, const tmbus::MeterInfo& map);
    static void memory();
};

class MeterMap
{
   private:
    static std::array<std::pair<tmbus::MeterType, tmbus::MeterInfo>, tmbus::METER_MAP_004T_SIZE>
        _meterMap_004T;
    static std::array<std::pair<tmbus::MeterType, tmbus::MeterInfo>, tmbus::METER_MAP_003_SIZE>
        _meterMap_003;

   public:
    static MeterMap& getInstance()
    {
        static MeterMap instance; // Singleton instance
        return instance;
    }
    // Converts string representations into enum types

    // Loads the meter info from the file system
    void loadMeterInfo();
    std::array<std::pair<tmbus::MeterType, tmbus::MeterInfo>, tmbus::METER_MAP_004T_SIZE>*
    sendMapReference(pzemCore::PZEMModel model);

    // Checks if the maps are loaded and available
    bool isMapAvailable(pzemCore::PZEMModel model) const;
    void printMeterInfo();
    bool isMapLoaded(pzemCore::PZEMModel model) const;

   private:
    MeterMap() {} // Private constructor for singleton
    MeterMap(const MeterMap&) = delete;
    MeterMap& operator=(const MeterMap&) = delete;

    // Flags to indicate if the meter maps are loaded
    bool meterMap_004T_loaded = false;
    bool meterMap_003_loaded = false;
    std::array<std::pair<tmbus::MeterType, tmbus::MeterInfo>, tmbus::METER_MAP_004T_SIZE>*
    getMapReference(pzemCore::PZEMModel model) const;
    // Internal helper to safely access maps
};
}; // namespace Utility

#endif