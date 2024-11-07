#include <unity.h>
#include "RTU_Modbus.hpp"
#include "TTL_ModbusTest.hpp"
#include "ModbusDefaults.h"
#include "IModbusDevice.hpp"
#include <cstring> // for std::memcmp

using namespace tmbus;
using namespace pzemCore;
class MockRTUModbusDevice : public IModbusDevice
{
   public:
    MockRTUModbusDevice(uint8_t id, pzemCore::PZEMModel model) : IModbusDevice(id, model){};

    void init() override { Serial.println("ModckModbus Device created!!"); }
    bool updateRegisters(RegisterType regType, Update updateType, const uint8_t* data,
                         const uint8_t length, const uint16_t startAddress = 0x0000) override
    {
        return true;
    }
};

// Define expected values for testing
const uint8_t expectedBuffer[TTL_MODBUS_NORMAL_FRAME_SIZE] = {/* Fill with expected byte values */};

void TTL_ModbusTest::runAllTests()
{
    RUN_TEST(test_sendMsg);
    // RUN_TEST(test_sendEnergyResetMsg);
}

void TTL_ModbusTest::test_sendMsg()
{
    Serial.println("Starting test_sendMsg");
    FunctionCode testCmd = FunctionCode::WRITE_SINGLE_REG;
    PZEMModel model = pzemCore::PZEMModel::PZEM004T;
    uint16_t testRegAddr = 0x0001;
    uint16_t testValueOrNumRegs = 0x0001;
    uint8_t testSlaveAddr = 0x01;
    bool testWaitForReply = true;
    int id = 21;

    Serial.println("Preparing to create MockRTUModbusDevice");
    Serial.printf("ID: %d, Model: %d\n", id, static_cast<uint8_t>(model));
    MockRTUModbusDevice device(id, pzemCore::PZEMModel::PZEM004T);
    Serial.println("getting modbus register get function ");
    // tmbus::ModbusRegisters& modbusRegisters = device.getModbusRegisters();

    Serial.println("creating RTU modbus device");
    RTUModbus modbus(&device, testSlaveAddr);
    Serial.println("transmitting result");
    // Act: Call the method under test
    transmit* result =
        modbus.sendMsg(testCmd, testRegAddr, testValueOrNumRegs, testSlaveAddr, testWaitForReply);

    // Assert: Verify the result is not null and check values if needed
    TEST_ASSERT_NOT_NULL(result);

    delete result;
}

void TTL_ModbusTest::test_sendEnergyResetMsg()
{
    uint8_t testSlaveAddr = 0x01;
    uint16_t testRegAddr = 0x0001;
    MockRTUModbusDevice device(1, PZEMModel::PZEM004T);

    RTUModbus modbus(&device, testSlaveAddr);

    // Act: Call the method under test
    transmit* result = modbus.sendEnergyResetMsg(testSlaveAddr, testRegAddr);

    // Assert: Verify the result is not null
    TEST_ASSERT_NOT_NULL(result);

    // Clean up

    delete result;
}