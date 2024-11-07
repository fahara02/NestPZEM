#ifndef MODBUS_MESSAGE_TEST_HPP
#define MODBUS_MESSAGE_TEST_HPP

#include <unity.h>
#include "ModbusMessage.h"

class ModbusMessageTest
{
   public:
    static void runAllTests();

   private:
    static void test_read_voltage_from_RIR();
    static void test_read_alarm_threshold_from_RHR();
    static void test_write_modbus_address_to_register();
    static void test_read_min_register_value();
    static void test_big_endian_serialization();
    static uint16_t calculateCRC(const uint8_t* data, uint16_t length);
};

#endif // IMODBUS_DEVICE_TEST_HPP
