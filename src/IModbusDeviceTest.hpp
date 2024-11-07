#ifndef IMODBUS_DEVICE_TEST_HPP
#define IMODBUS_DEVICE_TEST_HPP

#include <unity.h>
#include "IModbusDevice.hpp"

class IModbusDeviceTest
{
   public:
    static void runAllTests();

   private:
    static void testSetRegistersForPZEM004T();
    static void testSetRegistersForPZEM003();
    static void testInvalidModel();
    static void testGetRIRByName();
    static void testGetRHRByName();
    static void testGetAllRIR();
    static void testGetAllRHR();
    static void testScalingFactor();
    static void testRegisterNotAllowedFunctionCode();
};

#endif // IMODBUS_DEVICE_TEST_HPP
