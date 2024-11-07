#include <unity.h>
#include "ModbusMessage.h"
#include "ModbusRegisters.hpp"
#include "ModbusRegistersTest.hpp"
#include "IModbusDevice.hpp"
#include "PZEM_Defaults.h"
#include "Arduino.h"
#include "esp_timer.h"
#include "PowerMeter.hpp"
#include "NestUtility.hpp"
#include "CommonTestData.hpp"

using namespace tmbus;
using namespace pzemCore;

class MockModbusDevice : public IModbusDevice, public PowerMeter
{
   public:
    PZEMModel _model;
    uint8_t _id;
    uint8_t _addr;
    NamePlate _namePlate;

    MockModbusDevice(PZEMModel model, uint8_t id, uint8_t addr, uint8_t lineNo = 0,
                     Phase phase = Phase::SINGLE_PHASE, const char* descr = nullptr) :
        IModbusDevice(id, model),
        PowerMeter(getModbusRegisters(), model, id, addr, lineNo, phase, descr),
        _model(model),
        _namePlate(_model, _id, _addr, lineNo, phase, descr)

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

    bool updateMeters() override { return true; }
    bool resetEnergyCounter() override { return true; }
    uint8_t getaddr() const override { return 0; }
};

void ModbusRegistersTest::runAllTests() { RUN_TEST(test_UpdateRegisters); }

void ModbusRegistersTest::test_UpdateRegisters()
{
    Serial.println("first memory usage---- -1");
    Utility::PRINT::memory();

    uint16_t address = 0x0000; // Starting address of IR
    uint8_t numInputRegisters = 7;

    ModbusMessage modbusMessage;
    TEST_ASSERT_TRUE(modbusMessage.decodeMessage(testData, sizeof(testData), true));

    ModbusMessage modbusMessage2;
    TEST_ASSERT_TRUE(modbusMessage2.decodeMessage(testData2, sizeof(testData2), true));

    Serial.println(" memory usage After ModbusMessage  decode --2");
    Utility::PRINT::memory();

    const uint8_t* data = modbusMessage.getValidRawData();
    uint8_t byteCount = modbusMessage.getByteCount();
    TEST_ASSERT_EQUAL(byteCount, 20);

    const uint8_t* data2 = modbusMessage2.getValidRawData();
    uint8_t byteCount2 = modbusMessage2.getByteCount();
    TEST_ASSERT_EQUAL(byteCount2, 20);
    pzemCore::PZEMModel model1 = pzemCore::PZEMModel::PZEM004T;
    uint8_t id_1 = 11;
    uint8_t addr = 0x01;
    MockModbusDevice device1(model1, id_1, addr);

    // device1.setRegisters(model1);
    const JobCard* job1;
    job1 = &device1.getJobCard();

    Serial.println(" memory usage After first device--3");
    Utility::PRINT::memory();

    std::shared_ptr<tmbus::ModbusRegisters> mReg1 = device1.getModbusRegisters();
    int64_t startTime = esp_timer_get_time();
    TEST_ASSERT_TRUE(mReg1->update(address, numInputRegisters, data, true));
    int64_t endTime = esp_timer_get_time();
    int64_t elapsedTime = endTime - startTime; // Time in microseconds
    Serial.printf("Single update time (us): %lld\n", elapsedTime);

    Serial.println(" memory usage After first update--4");
    Utility::PRINT::memory();

    pzemCore::PZEMModel model2 = pzemCore::PZEMModel::PZEM004T;
    uint8_t id2 = 22;
    uint8_t addr2 = 0x02;
    MockModbusDevice device2(model2, id2, addr2);

    Serial.println(" memory usage After second device--5");
    Utility::PRINT::memory();
    // Serial.printf("\nDevice 2 Model: %s", toString.model(device2.getModel()));
    // device2.setRegisters(model2);
    const JobCard* job2;
    job2 = &device2.getJobCard();

    std::shared_ptr<tmbus::ModbusRegisters> mReg2 = device2.getModbusRegisters();
    int64_t startTime2 = esp_timer_get_time();
    TEST_ASSERT_TRUE(mReg2->update(address, numInputRegisters, data2, true));
    int64_t endTime2 = esp_timer_get_time();
    int64_t elapsedTime2 = endTime2 - startTime2; // Time in microseconds
    Serial.printf("Single update time (us): %lld\n", elapsedTime2);

    Serial.println(" memory usage After 2nd update--6");
    Utility::PRINT::memory();

    Serial.println("\n");
    Serial.println("\n ---------SIZES-------------");
    Serial.printf("\n MODBUS MESSAGE : %d", sizeof(modbusMessage));
    Serial.printf("\n MOCK MODBUS DEVICE : %d", sizeof(device1));
    Serial.printf("\n MODBUS REGISTER1 SIZE: %d", sizeof(mReg1));
    Serial.printf("\n MOCK MODBUS DEVICE2 : %d", sizeof(device2));
    Serial.printf("\n MODBUS REGISTER2 SIZE: %d", sizeof(mReg2));
    Serial.println("\n ---------SIZES-------------");
    Serial.println("\n");

    Serial.println("\n ---------INSTANCE 1 powerMeasure-------------");
    Serial.printf("\n printing pmeasure voltage %0.2f", job1->pm.getField(MeterType::VOLTAGE));
    Serial.printf("\n printing pmeasure current %0.2f", job1->pm.getField(MeterType::CURRENT));
    Serial.printf("\n printing pmeasure power %0.2f", job1->pm.getField(MeterType::POWER));
    Serial.printf("\n printing pmeasure power %0.2f", job1->pm.getField(MeterType::ENERGY));
    Serial.printf("\n printing pmeasure power %0.2f", job1->pm.getField(MeterType::FREQUENCY));
    Serial.printf("\n printing pmeasure power %0.2f", job1->pm.getField(MeterType::PF));
    Serial.println("\n");
    Serial.println("\n ---------INSTANCE 2 powerMeasure-------------");
    Serial.printf("\n printing pmeasure voltage %0.2f", job2->pm.getField(MeterType::VOLTAGE));
    Serial.printf("\n printing pmeasure current %0.2f", job2->pm.getField(MeterType::CURRENT));
    Serial.printf("\n printing pmeasure power %0.2f", job2->pm.getField(MeterType::POWER));
    Serial.printf("\n printing pmeasure power %0.2f", job2->pm.getField(MeterType::ENERGY));
    Serial.printf("\n printing pmeasure power %0.2f", job2->pm.getField(MeterType::FREQUENCY));
    Serial.printf("\n printing pmeasure power %0.2f", job2->pm.getField(MeterType::PF));
    Serial.println("\n");

    TEST_ASSERT_FLOAT_WITHIN(0.1f, 220.0f,
                             mReg1->readScaledValue(MeterType::VOLTAGE)); // Voltage
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 1.000f,
                             mReg1->readScaledValue(MeterType::CURRENT)); // Current
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 220.0f,
                             mReg1->readScaledValue(MeterType::POWER)); // Power
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f,
                             mReg1->readScaledValue(MeterType::ENERGY)); // Energy
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 50.0f,
                             mReg1->readScaledValue(MeterType::FREQUENCY)); //
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 1.00f,
                             mReg1->readScaledValue(MeterType::PF));              // Power Factor
    TEST_ASSERT_EQUAL_UINT32(0, mReg1->readScaledValue(MeterType::ALARM_STATUS)); // Alarm Status
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 230.0f,
                             mReg2->readScaledValue(MeterType::VOLTAGE)); // Voltage
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 1.500f,
                             mReg2->readScaledValue(MeterType::CURRENT)); // Current
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 345.0f,
                             mReg2->readScaledValue(MeterType::POWER)); // Power
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f,
                             mReg2->readScaledValue(MeterType::ENERGY)); // Energy
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 60.0f,
                             mReg2->readScaledValue(MeterType::FREQUENCY)); // Frequency
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.9f,
                             mReg2->readScaledValue(MeterType::PF));              // Power Factor
    TEST_ASSERT_EQUAL_UINT32(0, mReg2->readScaledValue(MeterType::ALARM_STATUS)); // Alarm Status
}
