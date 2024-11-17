#ifndef PZEM_MODBUS_HPP
#define PZEM_MODBUS_HPP

#include "ModbusServerTCPasync.h"
#include "PZEMDevice.hpp"
#include <array>
#include <memory>
#include <utility>
#include "Logging.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

class PZEMModbus
{
   private:
    static constexpr uint8_t maxDevices = 5;
    static constexpr uint8_t maxRegisters = 40;

    // Register Offsets and Sizes
    struct RegisterLayout
    {
        static constexpr uint8_t NamePlateOffset = 0;
        static constexpr uint8_t NamePlateSize = 10;
        static constexpr uint8_t PowerMeasureOffset = NamePlateOffset + NamePlateSize;
        static constexpr uint8_t PowerMeasureSize = 14;
        static constexpr uint8_t TimersOffset = PowerMeasureOffset + PowerMeasureSize;
        static constexpr uint8_t TimersSize = 8;
        static constexpr uint8_t StateOffset = TimersOffset + TimersSize;
        static constexpr uint8_t StateSize = 2;
    };

    // Server and devices array
    std::unique_ptr<ModbusServerTCPasync> MBServer;
    std::array<std::shared_ptr<pzemCore::PZEMDevice>, maxDevices> devices{};
    std::array<std::array<uint16_t, maxRegisters>, maxDevices> jobCardRegisters{};

    // Semaphore for thread-safe access
    SemaphoreHandle_t registerMutex;

    // Variadic constructor for multiple devices
    template <typename... Devices>
    PZEMModbus(std::unique_ptr<ModbusServerTCPasync> server, Devices&&... devicePtrs) :
        MBServer(std::move(server))
    {
        static_assert(sizeof...(Devices) <= maxDevices, "Too many devices provided.");
        initializeDevices(0, std::forward<Devices>(devicePtrs)...);
        registerMutex = xSemaphoreCreateMutex();
        configASSERT(registerMutex != nullptr); // Ensure mutex is created successfully
    }

    // Base case for recursion
    void initializeDevices(size_t) {}

    // Recursive variadic function to populate the devices array
    template <typename FirstDevice, typename... RemainingDevices>
    void initializeDevices(size_t index, FirstDevice&& first, RemainingDevices&&... remaining)
    {
        if(index < maxDevices)
        {
            devices[index] = std::forward<FirstDevice>(first);
            initializeDevices(index + 1, std::forward<RemainingDevices>(remaining)...);
        }
    }

    // Update the Modbus register map from all devices
    void updateRegisters()
    {
        if(xSemaphoreTake(registerMutex, portMAX_DELAY) == pdTRUE)
        {
            for(uint8_t deviceIndex = 0; deviceIndex < maxDevices; ++deviceIndex)
            {
                if(devices[deviceIndex])
                {
                    jobCardRegisters[deviceIndex] = devices[deviceIndex]->getOutBox();
                }
            }
            xSemaphoreGive(registerMutex);
        }
    }

   public:
    template <typename... Devices>
    static PZEMModbus& getInstance(std::unique_ptr<ModbusServerTCPasync> server,
                                   Devices&&... devicePtrs)
    {
        static PZEMModbus instance(std::move(server), std::forward<Devices>(devicePtrs)...);
        return instance;
    }

    // Handle Modbus Function Code 03 (Read Holding Registers)
    ModbusMessage FC03(const ModbusMessage& request)
    {
        updateRegisters();
        ModbusMessage response;

        // Parse the request to extract the address and number of words
        uint16_t address, words;
        request.get(2, address);
        request.get(4, words);

        // Validate the server ID
        uint8_t serverID = request.getServerID();
        uint8_t functionCode = request.getFunctionCode();
        if(serverID == 0 || serverID > maxDevices)
        {
            response.setError(serverID, functionCode, INVALID_SERVER);
            return response;
        }

        // Validate the address and word count
        if(address + words > maxRegisters)
        {
            response.setError(serverID, functionCode, ILLEGAL_DATA_ADDRESS);
            return response;
        }

        // Ensure that words is greater than 0
        if(words == 0)
        {
            response.setError(serverID, functionCode, ILLEGAL_DATA_VALUE);
            return response;
        }

        // Prepare the response header

        response.add(serverID, functionCode, (uint8_t)(words * 2));

        // Add the requested registers to the response
        if(xSemaphoreTake(registerMutex, portMAX_DELAY) == pdTRUE)
        {
            const auto& deviceRegisters = jobCardRegisters[serverID - 1];
            for(uint16_t i = address; i < address + words; ++i)
            {
                response.add(deviceRegisters[i]);
            }
            xSemaphoreGive(registerMutex);
        }
        else
        {
            response.setError(serverID, functionCode, SERVER_DEVICE_BUSY);
        }

        return response;
    }

    // Start Modbus Server
    void startServer()
    {
        for(uint8_t i = 0; i < maxDevices; ++i)
        {
            if(devices[i])
            {
                MBServer->registerWorker(
                    i + 1, READ_HOLD_REGISTER,
                    [this](const ModbusMessage& request) { return this->FC03(request); });
            }
        }
        if(MBServer->start(502, 4, 20000))
        {
            Serial.printf("Modbus server started on port 502\n");
        }
        else
        {
            Serial.printf("Failed to start Modbus server! restarting\n");
            ESP.restart();
        };
    }
};

#endif // PZEM_MODBUS_HPP
