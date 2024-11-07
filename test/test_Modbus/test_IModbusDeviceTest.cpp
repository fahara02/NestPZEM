// #include <unity.h>
// #include "IModbusDevice.h"     // Include the header for IModbusDevice
// #include "ModbusRegisters.hpp" // Include your ModbusRegisters definition
// #include "PZEM_Defaults.h"
// #include "IModbusDeviceTest.hpp"

// using namespace tmbus;
// using namespace pzemCore;

// // Mock class for IModbusDevice
// class MockModbusDevice : public IModbusDevice
// {
//    public:
//     void init() override
//     {
//         // Mock init function
//     }

//     // Other methods as needed
// };

// // Implementation of the test methods in IModbusDeviceTest

// void IModbusDeviceTest::runAllTests()
// {
//     RUN_TEST(testSetRegistersForPZEM004T);
//     RUN_TEST(testSetRegistersForPZEM003);
//     RUN_TEST(testInvalidModel);
//     RUN_TEST(testGetRIRByName);
//     RUN_TEST(testGetRHRByName);
//     RUN_TEST(testGetAllRIR);
//     RUN_TEST(testGetAllRHR);
//     RUN_TEST(testScalingFactor);
//     RUN_TEST(testRegisterNotAllowedFunctionCode);
// }

// void IModbusDeviceTest::testSetRegistersForPZEM004T()
// {
//     MockModbusDevice device;
//     device.setRegisters(PZEMModel::PZEM004T);

//     // Use `TEST_ASSERT_EQUAL` with actual register values
//     TEST_ASSERT_EQUAL(PZEM004T_RIR[0].address, device.getRIR(0).address);
//     TEST_ASSERT_EQUAL(PZEM004T_RHR[0].address, device.getRHR(0).address);
//     TEST_ASSERT_EQUAL(PZ004T_RIR_DATA_BEGIN, device.getallRIR().first);
//     TEST_ASSERT_EQUAL(PZ004T_RIR_DATA_LEN, device.getallRIR().second);
//     TEST_ASSERT_EQUAL(PZ004T_RHR_DATA_BEGIN, device.getallRHR().first);
//     TEST_ASSERT_EQUAL(PZ004_RHR_DATA_LEN, device.getallRHR().second);
// }

// void IModbusDeviceTest::testSetRegistersForPZEM003()
// {
//     MockModbusDevice device;
//     device.setRegisters(PZEMModel::PZEM003);

//     // Use `TEST_ASSERT_EQUAL` with actual register values
//     TEST_ASSERT_EQUAL(PZEM003_RIR[0].address, device.getRIR(0).address);
//     TEST_ASSERT_EQUAL(PZEM003_RHR[0].address, device.getRHR(0).address);
//     TEST_ASSERT_EQUAL(PZ003_RIR_DATA_BEGIN, device.getallRIR().first);
//     TEST_ASSERT_EQUAL(PZ003_RIR_DATA_LEN, device.getallRIR().second);
//     TEST_ASSERT_EQUAL(PZ003_RHR_DATA_BEGIN, device.getallRHR().first);
//     TEST_ASSERT_EQUAL(PZ003_RHR_DATA_LEN, device.getallRHR().second);
// }

// void IModbusDeviceTest::testInvalidModel()
// {
//     MockModbusDevice device;
//     device.setRegisters(static_cast<PZEMModel>(-1));

//     TEST_ASSERT_EQUAL(0, device.getallRIR().second);
//     TEST_ASSERT_EQUAL(0, device.getallRHR().second);
// }

// void IModbusDeviceTest::testGetRIRByName()
// {
//     MockModbusDevice device;
//     device.setRegisters(PZEMModel::PZEM004T);

//     const ModbusRegister& reg = device.getRIRByName("Voltage");
//     TEST_ASSERT_EQUAL_STRING("Voltage", reg.name);
//     TEST_ASSERT_EQUAL(0x0000, reg.address);
// }

// void IModbusDeviceTest::testGetRHRByName()
// {
//     MockModbusDevice device;
//     device.setRegisters(PZEMModel::PZEM004T);

//     const ModbusRegister& reg = device.getRHRByName("Modbus_Address");
//     TEST_ASSERT_EQUAL_STRING("Modbus_Address", reg.name);
//     TEST_ASSERT_EQUAL(0x0002, reg.address);
// }

// void IModbusDeviceTest::testGetAllRIR()
// {
//     MockModbusDevice device;
//     device.setRegisters(PZEMModel::PZEM004T);

//     auto pair = device.getallRIR();
//     TEST_ASSERT_EQUAL(PZ004T_RIR_DATA_BEGIN, pair.first);
//     TEST_ASSERT_EQUAL(PZ004T_RIR_DATA_LEN, pair.second);
// }

// void IModbusDeviceTest::testGetAllRHR()
// {
//     MockModbusDevice device;
//     device.setRegisters(PZEMModel::PZEM004T);

//     auto pair = device.getallRHR();
//     TEST_ASSERT_EQUAL(PZ004T_RHR_DATA_BEGIN, pair.first);
//     TEST_ASSERT_EQUAL(PZ004_RHR_DATA_LEN, pair.second);
// }

// void IModbusDeviceTest::testScalingFactor()
// {
//     ModbusRegister voltageReg = PZEM004T_VOLTAGE;
//     uint32_t rawValue = 230; // Raw value for voltage
//     voltageReg.updateValue(rawValue);

//     float scaledValue = voltageReg.getScaledValue();
//     TEST_ASSERT_EQUAL_FLOAT(230.0f, scaledValue);
// }

// void IModbusDeviceTest::testRegisterNotAllowedFunctionCode()
// {
//     const ModbusRegister& voltageReg = PZEM004T_VOLTAGE;

//     TEST_ASSERT_FALSE(voltageReg.isFunctionCodeAllowed(FunctionCode::WRITE_SINGLE_REG));
//     TEST_ASSERT_TRUE(voltageReg.isFunctionCodeAllowed(FunctionCode::READ_INPUT_REG));
// }
