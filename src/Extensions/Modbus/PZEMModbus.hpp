#ifndef PZEM_MODBUS_HPP
#define PZEM_MODBUS_HPP
#include "RTUNode.hpp"

#include "ModbusServerTCPasync.h"
#include "CoilData.h"
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
    static constexpr uint8_t maxClients = 4;
    // Coil configuration
    static constexpr uint16_t MAX_COILS = 5;
    static constexpr uint16_t UPS_IN_COIL_ADDR = 0;
    static constexpr uint16_t LOAD_BANK_1_COIL_ADDR = 1;
    static constexpr uint16_t LOAD_BANK_2_COIL_ADDR = 2;
    static constexpr uint16_t LOAD_BANK_3_COIL_ADDR = 3;
    static constexpr uint16_t TEST_UPDATE_ADDR = 4;
    const std::array<uint8_t, MAX_COILS> coilGPIOs = {23, 22, 21, 19, 18};
    CoilData coils{MAX_COILS};
    static constexpr uint32_t serverTimeout = 20000;
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
    SemaphoreHandle_t coilMutex;

    // Variadic constructor for multiple devices
    template <typename... Devices>
    PZEMModbus(std::unique_ptr<ModbusServerTCPasync> server, Devices&&... devicePtrs) :
        MBServer(std::move(server))
    {
        static_assert(sizeof...(Devices) <= maxDevices, "Too many devices provided.");
        initializeDevices(0, std::forward<Devices>(devicePtrs)...);
        initGPIOs();
        registerMutex = xSemaphoreCreateMutex();
        configASSERT(registerMutex != nullptr); // Ensure mutex is created successfully
        coilMutex = xSemaphoreCreateMutex();
        configASSERT(coilMutex != nullptr); // Ensure mutex is created successfully
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
    void initGPIOs()
    {
        gpio_config_t io_conf = {};
        io_conf.mode = GPIO_MODE_OUTPUT_OD;
        io_conf.pin_bit_mask =
            (1ULL << 23) | (1ULL << 22) | (1ULL << 21) | (1ULL << 19) | (1ULL << 18);
        io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
        io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
        io_conf.intr_type = GPIO_INTR_DISABLE;
        gpio_config(&io_conf);
        // Set GPIOs to HIGH to turn relays off
        gpio_set_level(GPIO_NUM_23, 1);
        gpio_set_level(GPIO_NUM_22, 1);
        gpio_set_level(GPIO_NUM_21, 1);
        gpio_set_level(GPIO_NUM_19, 1);
        gpio_set_level(GPIO_NUM_18, 1);
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
    void updateGPIO(uint16_t address, bool state)
    {
        int index = address - UPS_IN_COIL_ADDR;
        if(index >= 0 && index < MAX_COILS)
        {
            gpio_set_level(static_cast<gpio_num_t>(coilGPIOs[index]), state);
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
    ModbusMessage FC01(const ModbusMessage& request)
    {
        ModbusMessage response;

        // Extract the starting address and number of coils to read
        uint16_t startAddress = 0;
        uint16_t numCoils = 0;
        request.get(2, startAddress, numCoils);

        // Validate the server ID
        uint8_t serverID = request.getServerID();
        uint8_t functionCode = request.getFunctionCode();

        if(serverID == 0 || serverID > maxDevices + 1) // +1 for the GPIO server
        {
            response.setError(serverID, functionCode, INVALID_SERVER);
            return response;
        }

        // Validate address range
        if(startAddress < UPS_IN_COIL_ADDR
           || startAddress + numCoils > UPS_IN_COIL_ADDR + MAX_COILS)
        {
            response.setError(serverID, functionCode, ILLEGAL_DATA_ADDRESS);
            return response;
        }

        // Fetch the real-time coil states from GPIO
        std::vector<uint8_t> coilData;
        coilData.resize((numCoils + 7) / 8); // Each byte contains 8 coils

        for(uint16_t i = 0; i < numCoils; ++i)
        {
            uint16_t coilIndex = startAddress - UPS_IN_COIL_ADDR + i;
            if(coilIndex < MAX_COILS)
            {
                uint8_t gpioState = gpio_get_level(static_cast<gpio_num_t>(coilGPIOs[coilIndex]));
                if(gpioState)
                {
                    coilData[i / 8] |= (1 << (i % 8)); // Set the corresponding bit in the response
                }
            }
        }

        // Prepare the response
        response.add(serverID, functionCode, static_cast<uint8_t>(coilData.size()));
        response.add(coilData);

        return response;
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
    ModbusMessage FC05(const ModbusMessage& request)
    {
        ModbusMessage response;

        uint16_t address, value;
        request.get(2, address);
        request.get(4, value);

        uint8_t serverID = request.getServerID();
        uint8_t functionCode = request.getFunctionCode();

        if(serverID == 0 || serverID > maxDevices)
        {
            response.setError(serverID, functionCode, INVALID_SERVER);
            return response;
        }

        if(address < UPS_IN_COIL_ADDR || address >= UPS_IN_COIL_ADDR + MAX_COILS)
        {
            response.setError(serverID, functionCode, ILLEGAL_DATA_ADDRESS);
            return response;
        }

        bool state = (value == 0xFF00);
        if(xSemaphoreTake(coilMutex, portMAX_DELAY) == pdTRUE)
        {
            bool coil_state = !state;
            if(coils.set(address - UPS_IN_COIL_ADDR, state))
            {
                updateGPIO(address, coil_state);
                response.add(serverID, functionCode, address, value);
            }
            else
            {
                response.setError(serverID, functionCode, SERVER_DEVICE_FAILURE);
            }
            xSemaphoreGive(coilMutex);
        }
        else
        {
            response.setError(serverID, functionCode, SERVER_DEVICE_BUSY);
        }

        return response;
    }

    // FC0F: Write Multiple Coils
    ModbusMessage FC0F(const ModbusMessage& request)
    {
        ModbusMessage response;

        uint16_t startAddress, coilCount;
        uint8_t byteCount;
        uint16_t offset = request.get(2, startAddress, coilCount, byteCount);

        uint8_t serverID = request.getServerID();
        uint8_t functionCode = request.getFunctionCode();

        if(serverID == 0 || serverID > maxDevices)
        {
            response.setError(serverID, functionCode, INVALID_SERVER);
            return response;
        }

        if(startAddress < UPS_IN_COIL_ADDR
           || startAddress + coilCount > UPS_IN_COIL_ADDR + MAX_COILS)
        {
            response.setError(serverID, functionCode, ILLEGAL_DATA_ADDRESS);
            return response;
        }

        if(byteCount != ((coilCount - 1) >> 3) + 1)
        {
            response.setError(serverID, functionCode, ILLEGAL_DATA_VALUE);
            return response;
        }

        std::vector<uint8_t> coilData(byteCount);
        request.get(offset, coilData, byteCount);

        bool operationFailed = false;
        for(uint16_t i = 0; i < coilCount; ++i)
        {
            bool state = coilData[i / 8] & (1 << (i % 8));
            if(xSemaphoreTake(coilMutex, portMAX_DELAY) == pdTRUE)
            {
                if(!coils.set(startAddress - UPS_IN_COIL_ADDR + i, state))
                {
                    operationFailed = true; // Track failure
                }
                else if(startAddress + i >= UPS_IN_COIL_ADDR
                        && startAddress + i < UPS_IN_COIL_ADDR + MAX_COILS)
                {
                    updateGPIO(startAddress + i, state);
                }
                xSemaphoreGive(coilMutex);
            }
            else
            {
                response.setError(serverID, functionCode, SERVER_DEVICE_BUSY);
                return response;
            }
        }

        if(operationFailed)
        {
            response.setError(serverID, functionCode, SERVER_DEVICE_FAILURE);
        }
        else
        {
            response.add(serverID, functionCode, startAddress, coilCount);
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
                MBServer->registerWorker(i + 1, WRITE_COIL, [this](const ModbusMessage& request) {
                    return this->FC05(request);
                });
            }
        }
        uint8_t gpioServerID = 11;
        MBServer->registerWorker(gpioServerID, READ_COIL, [this](const ModbusMessage& request) {
            return this->FC01(request);
        });
        // Register workers for GPIO-related coil operations
        MBServer->registerWorker(gpioServerID, WRITE_COIL, [this](const ModbusMessage& request) {
            return this->FC05(request);
        });

        MBServer->registerWorker(
            gpioServerID, WRITE_MULT_COILS,
            [this](const ModbusMessage& request) { return this->FC0F(request); });
        if(MBServer->start(502, maxClients, serverTimeout))
        {
            Serial.printf("\nModbus server started on port 502\n");
        }
        else
        {
            Serial.printf("\nFailed to start Modbus server! restarting\n");
            ESP.restart();
        };
    }
};

#endif // PZEM_MODBUS_HPP
