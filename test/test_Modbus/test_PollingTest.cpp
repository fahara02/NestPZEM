#include "PollingTest.hpp"
#include <unity.h>
#include <vector>
#include "Arduino.h"
#include "esp_timer.h"
#include "ModbusMessage.h"
#include "ModbusRegisters.hpp"
#include "IModbusDevice.hpp"
#include "PZEM_Defaults.h"
#include "PowerMeter.hpp"
#include "NestUtility.hpp"
#include "CommonTestData.hpp"
#include "MockPZEM.hpp"

using namespace tmbus;
using namespace pzemCore;
class MockModbusDevice : public IModbusDevice, public PowerMeter
{
   public:
    PZEMModel _model;
    uint8_t _id;
    uint8_t _addr;
    NamePlate _namePlate;
    Utility::MockPZEM _pzem;

    MockModbusDevice(PZEMModel model, uint8_t id, uint8_t addr, uint8_t lineNo = 0,
                     Phase phase = Phase::SINGLE_PHASE, const char* nametag = nullptr) :
        IModbusDevice(id, model),
        PowerMeter(getModbusRegisters(), model, id, addr, lineNo, phase, nametag),
        _model(model),
        _namePlate(model, id, addr, lineNo, phase, nametag),
        _pzem(model, id, addr)

    {
    }

    ~MockModbusDevice() override {}

    void init() override
    {
        // initMeter();
        Serial.println("ModckModbus Device created!!");
    }

    bool updateRegisters(RegisterType regType, Update updateType, const uint8_t* data,
                         const uint8_t length, const uint16_t startAddress = 0x0000) override
    {
        return true;
    }
    std::vector<uint8_t> randomData() { return _pzem.sendRandomData(); }
    bool updateMeters() override { return true; }
    bool resetEnergyCounter() override { return true; }
    uint8_t getaddr() const override { return _addr; }
};

void PollingTest::runAllTests()
{
    RUN_TEST(periodicPolling);
    //  RUN_TEST(dataAgeDetection);
}

void PollingTest::dataAgeDetection()
{ // please implement
}

void PollingTest::periodicPolling()
{
    Serial.println("Starting polling test...");

    // Setup Mock Devices
    pzemCore::PZEMModel model = pzemCore::PZEMModel::PZEM004T;
    uint8_t addr1 = 0x01, addr2 = 0x02;
    MockModbusDevice device1(model, 11, addr1);
    MockModbusDevice device2(model, 22, addr2);
    Serial.println("Created two devices...");

    // Setup Modbus registers
    std::shared_ptr<tmbus::ModbusRegisters> mReg1 = device1.getModbusRegisters();
    std::shared_ptr<tmbus::ModbusRegisters> mReg2 = device2.getModbusRegisters();
    const uint16_t address = 0x0000; // Starting address
    const uint8_t numInputRegisters = 7;

    TEST_ASSERT_TRUE(device1.autopoll(true));
    TEST_ASSERT_TRUE(device2.autopoll(true));

    const uint32_t testDurationMs = 10000; // 10 seconds
    uint32_t currentTime = millis();
    const uint32_t pollingIntervalMs = 1000;
    uint32_t lastPollTime = currentTime;

    bool pollDevice1 = true; // Start with device 1

    while(millis() - currentTime < testDurationMs)
    {
        // Ensure correct polling interval
        if(millis() - lastPollTime >= pollingIntervalMs)
        {
            lastPollTime = millis();

            if(pollDevice1)
            {
                ModbusMessage modbusMessage;
                Serial.println("decoding message device 1...");
                std::vector<uint8_t> randomDataVector = device1._pzem.sendRandomData();
                uint8_t randomData1[randomDataVector.size()]; // Size based on vector length
                std::copy(randomDataVector.begin(), randomDataVector.end(), randomData1);
                TEST_ASSERT_TRUE(
                    modbusMessage.decodeMessage(randomData1, sizeof(randomData1), true));
                Serial.println("getting raw data for device 1 from modbus message...");
                const uint8_t* data = modbusMessage.getValidRawData();

                ModbusMessage modbusMessage2;
                Serial.println("decoding message device 2...");
                TEST_ASSERT_TRUE(modbusMessage2.decodeMessage(testData2, sizeof(testData2), true));
                Serial.println("getting raw data for device 2 from modbus message...");
                const uint8_t* data2 = modbusMessage2.getValidRawData();
                if(data == nullptr || data2 == nullptr)
                {
                    Serial.println("Error: Null data received for device 1");
                }
                else
                {
                    Serial.print("data pointer is valid ");
                }
                Serial.println("checking update device 1...");
                ESP_LOGI("Polling test", "device1 received register ptr: %p", mReg1.get());
                TEST_ASSERT_TRUE(mReg1.get()->update(address, numInputRegisters, data, true));

                Serial.println("checking update device 2...");
                ESP_LOGI("Polling test", "device 2 received register ptr: %p", mReg2.get());
                TEST_ASSERT_TRUE(mReg2.get()->update(address, numInputRegisters, data2, true));
            }
        }

        vTaskDelay(
            pdMS_TO_TICKS(100)); // Slightly faster delay to allow more control over time increments
    }

    TEST_ASSERT_TRUE(device1.autopoll(false));
    TEST_ASSERT_TRUE(device2.autopoll(false));

    Serial.println("Polling test finished.");
}
