#ifndef TTL_MODBUS_TEST_HPP
#define TTL_MODBUS_TEST_HPP

#include <unity.h>
#include "RTU_Modbus.hpp"

class TTL_ModbusTest
{
   public:
    static void runAllTests();

   private:
    static void test_sendMsg();
    static void test_sendEnergyResetMsg();
};

#endif // IMODBUS_DEVICE_TEST_HPP
