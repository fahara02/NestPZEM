#ifndef MODBUS_REGISTERS_TEST_HPP
#define MODBUS_REGISTERS_TEST_HPP

#include <unity.h>
#include "ModbusRegisters.hpp"

class ModbusRegistersTest
{
   public:
    static void runAllTests();

   private:
    static void test_load_IR();
    static void test_UpdateRegisters();
    static void printMemoryUsage();
};

#endif // IMODBUS_DEVICE_TEST_HPP
