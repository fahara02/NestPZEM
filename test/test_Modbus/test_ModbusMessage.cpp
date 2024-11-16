#include <unity.h>
#include <cstring>
#include "Arduino.h"
#include "TTL_Modbus_Message.h"
#include "ModbusMessageTest.hpp"
#include "NestUtility.hpp"

using namespace tmbus;
using namespace pzemCore;

#include "ModbusMessageTest.hpp"

void ModbusMessageTest::runAllTests()
{
    RUN_TEST(test_read_voltage_from_RIR);
    RUN_TEST(test_read_alarm_threshold_from_RHR);
    RUN_TEST(test_write_modbus_address_to_register);
    RUN_TEST(test_read_min_register_value);
}

void ModbusMessageTest::test_read_voltage_from_RIR()
{
    // Example usage for a read input register command

    TTLModbusMessage msg(0x01, 0x04, 0x0002, 0x0001); // Reading 1 register starting from 0x0000
    size_t READ_MESSAGE_SIZE = TTL_MODBUS_NORMAL_FRAME_SIZE;
    uint8_t messageBuffer[READ_MESSAGE_SIZE] = {0};
    msg.serializeMessage(messageBuffer, READ_MESSAGE_SIZE, true);
    // Validate the parsed data
    TEST_ASSERT_EQUAL(0x01, messageBuffer[0]);

    TEST_ASSERT_EQUAL(0x04, messageBuffer[1]);

    TEST_ASSERT_EQUAL(0x0002, (uint16_t)(messageBuffer[2] << 8 | messageBuffer[3]));

    TEST_ASSERT_EQUAL(0x0001, (uint16_t)(messageBuffer[4] << 8 | messageBuffer[5]));

    uint8_t messageDataBuffer[READ_MESSAGE_SIZE - 2];
    TEST_ASSERT_EQUAL(true, msg.serializeData(messageDataBuffer, READ_MESSAGE_SIZE - 2, true));

    uint16_t calculatedCRC = calculateCRC(reinterpret_cast<const uint8_t*>(&messageDataBuffer),
                                          sizeof(messageDataBuffer));

    uint16_t storedCRC = (uint16_t)(messageBuffer[7] << 8 | messageBuffer[6]);
    Serial.println("Printing Modbus Message size");
    Serial.println(sizeof(ModbusMessage));
    TEST_ASSERT_EQUAL(calculatedCRC, storedCRC);
}

void ModbusMessageTest::test_read_alarm_threshold_from_RHR()
{
    // Example usage for a read holding register command
    TTLModbusMessage msg(0x01, 0x03, 0x0005, 0x0002); // Reading 2 registers starting from 0x0005
    size_t READ_MESSAGE_SIZE = TTL_MODBUS_NORMAL_FRAME_SIZE;
    uint8_t messageBuffer[READ_MESSAGE_SIZE] = {0};

    TEST_ASSERT_EQUAL(true, msg.serializeMessage(messageBuffer, READ_MESSAGE_SIZE, true));

    TEST_ASSERT_EQUAL(0x01, messageBuffer[0]);
    TEST_ASSERT_EQUAL(0x03, messageBuffer[1]);
    TEST_ASSERT_EQUAL(0x0005, (uint16_t)(messageBuffer[2] << 8 | messageBuffer[3]));
    TEST_ASSERT_EQUAL(0x0002, (uint16_t)(messageBuffer[4] << 8 | messageBuffer[5]));
    uint8_t messageDataBuffer[READ_MESSAGE_SIZE - 2];
    TEST_ASSERT_EQUAL(true, msg.serializeData(messageDataBuffer, READ_MESSAGE_SIZE - 2, true));

    // Validate the CRC
    uint16_t calculatedCRC = calculateCRC(reinterpret_cast<const uint8_t*>(&messageDataBuffer),
                                          sizeof(messageDataBuffer));
    TEST_ASSERT_EQUAL(0x0AD4, calculatedCRC);

    uint16_t storedCRC = (uint16_t)(messageBuffer[7] << 8 | messageBuffer[6]);
    TEST_ASSERT_EQUAL(calculatedCRC, storedCRC);
}

void ModbusMessageTest::test_write_modbus_address_to_register()
{
    // Example usage for a write single register command
    TTLModbusMessage msg(0x01, 0x06, 0x000A, 0x0001); // Writing value 0x0001 to register 0x000A
    size_t READ_MESSAGE_SIZE = TTL_MODBUS_NORMAL_FRAME_SIZE;
    uint8_t messageBuffer[READ_MESSAGE_SIZE] = {0};

    TEST_ASSERT_EQUAL(true, msg.serializeMessage(messageBuffer, READ_MESSAGE_SIZE, true));
    // Validate the parsed data
    TEST_ASSERT_EQUAL(0x01, messageBuffer[0]);

    TEST_ASSERT_EQUAL(0x06, messageBuffer[1]);

    TEST_ASSERT_EQUAL(0x000A, (uint16_t)(messageBuffer[2] << 8 | messageBuffer[3]));

    TEST_ASSERT_EQUAL(0x0001, (uint16_t)(messageBuffer[4] << 8 | messageBuffer[5]));
    uint8_t messageDataBuffer[READ_MESSAGE_SIZE - 2];
    TEST_ASSERT_EQUAL(true, msg.serializeData(messageDataBuffer, READ_MESSAGE_SIZE - 2, true));

    // Validate the CRC
    uint16_t calculatedCRC = calculateCRC(reinterpret_cast<const uint8_t*>(&messageDataBuffer),
                                          sizeof(messageDataBuffer));

    uint16_t storedCRC = (uint16_t)(messageBuffer[7] << 8 | messageBuffer[6]);
    TEST_ASSERT_EQUAL(calculatedCRC, storedCRC);
}
void ModbusMessageTest::test_read_min_register_value()
{
    // Test case for reading from the minimum register value (0x0000)
    Serial.println("Starting test_read_min_register_value");

    TTLModbusMessage msg(0x01, 0x04, 0x0000, 0x0001); // Reading 1 register from 0x0000
    size_t READ_MESSAGE_SIZE = TTL_MODBUS_NORMAL_FRAME_SIZE;
    uint8_t messageBuffer[READ_MESSAGE_SIZE] = {0};
    msg.serializeMessage(messageBuffer, READ_MESSAGE_SIZE, true);

    TEST_ASSERT_EQUAL(0x01, messageBuffer[0]); // Check slave address
    TEST_ASSERT_EQUAL(0x04, messageBuffer[1]); // Check function code
    TEST_ASSERT_EQUAL(
        0x0000, (uint16_t)(messageBuffer[2] << 8 | messageBuffer[3])); // Check register address
    TEST_ASSERT_EQUAL(0x0001,
                      (uint16_t)(messageBuffer[4] << 8 | messageBuffer[5])); // Check register count

    uint8_t messageDataBuffer[READ_MESSAGE_SIZE - 2];
    TEST_ASSERT_EQUAL(true, msg.serializeData(messageDataBuffer, READ_MESSAGE_SIZE - 2, true));

    // Check CRC calculation
    uint16_t calculatedCRC = calculateCRC(reinterpret_cast<const uint8_t*>(&messageDataBuffer),
                                          sizeof(messageDataBuffer));
    uint16_t storedCRC = (uint16_t)(messageBuffer[7] << 8 | messageBuffer[6]);
    TEST_ASSERT_EQUAL(calculatedCRC, storedCRC);

    Serial.println("CRC checked");
}

uint16_t ModbusMessageTest::calculateCRC(const uint8_t* data, uint16_t length)
{
    // This function will calculate CRC for validation
    uint16_t crc = 0xFFFF;
    for(uint16_t pos = 0; pos < length; pos++)
    {
        crc ^= (uint16_t)data[pos]; // XOR byte into least significant byte of crc

        for(uint8_t i = 8; i != 0; i--)
        {
            if((crc & 0x0001) != 0)
            {
                crc >>= 1; // Shift right and XOR 0xA001
                crc ^= 0xA001;
            }
            else
            {
                crc >>= 1; // Just shift right
            }
        }
    }
    return crc;
}
