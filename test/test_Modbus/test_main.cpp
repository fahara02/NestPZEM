// #include <Arduino.h>
// #include <unity.h>
// #include "IModbusDeviceTest.hpp"
// #include "ModbusMessageTest.hpp"
// #include "ModbusRegistersTest.hpp"
// #include "TTL_ModbusTest.hpp"
// #include "PollingTest.hpp"

// void setup()
// {
//     Serial.begin(115200);
//     delay(1000);
//     Serial.println("Starting PZEM test");
//     UNITY_BEGIN();
//     // IModbusDeviceTest::runAllTests();
//     // Serial.println("Starting Modbus Message test");
//     // ModbusMessageTest::runAllTests();
//     // Serial.println("Starting Modbus Registers test");
//     // ModbusRegistersTest::runAllTests();
//     //  Serial.println("Starting Modbus RTU test");
//     //  TTL_ModbusTest::runAllTests();
//     Serial.println("Starting ModbusPowerMeter Polling test");
//     PollingTest::runAllTests();
//     UNITY_END();
//     Serial.println("Ending Modbus Class test");
// }

// void loop() {}