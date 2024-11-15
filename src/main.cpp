#include <Arduino.h>
#include <WiFi.h>
#include "FS.h"
#include <LittleFS.h>
#include <time.h>
#include "NestUtility.hpp"
#include "PZEMDevice.hpp"
#include "PZEM_constants.h"
#include "powerMeasure.h"
#include "UARTMsgQueue.hpp"
#include "UARTManager.hpp"
#include "ModbusRegisters.hpp"
#include "esp_log.h"
#include "PZEMClient.hpp"
#include "IModbusDevice.hpp"
#include "RTU_Modbus.hpp"
#include "PowerMeter.hpp"

#include "pb_encode.h"
#include "pb_decode.h"
#include "pData.pb.h"
#include "PZEMModbus.hpp"

#define FORMAT_LITTLEFS_IF_FAILED true
#define PZEM_UART_PORT UART_NUM_1
#define RX_PIN 16
#define TX_PIN 17
constexpr uint8_t PZEM_ID = 39;
constexpr uint8_t PZEM_ADDRESS = 1;

#ifndef MY_SSID
    #define MY_SSID "Torongo"
#endif
#ifndef MY_PASS
    #define MY_PASS "torongo@2021"
#endif

char ssid[] = MY_SSID; // SSID and ...
char pass[] = MY_PASS; // password for the WiFi network used

pzemCore::PZEMModel model = pzemCore::PZEMModel::PZEM004T;
uart_config_t uartConfig = {.baud_rate = 9600,
                            .data_bits = UART_DATA_8_BITS,
                            .parity = UART_PARITY_DISABLE,
                            .stop_bits = UART_STOP_BITS_1,
                            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE};
tmbus::UARTManager uartManager(PZEM_UART_PORT, uartConfig, RX_PIN, TX_PIN);

std::shared_ptr<pzemCore::PZEMDevice> pzemDevice = nullptr;

void setup()
{
    Serial.begin(115200);

    WiFi.begin(ssid, pass);
    delay(200);
    while(WiFi.status() != WL_CONNECTED)
    {
        Serial.print(". ");
        delay(500);
    }
    IPAddress wIP = WiFi.localIP();
    Serial.printf("WIFi IP address: %u.%u.%u.%u\n", wIP[0], wIP[1], wIP[2], wIP[3]);
    Serial.printf("creating the PZEM Device with id %d and address %d ", PZEM_ID, PZEM_ADDRESS);
    pzemDevice = std::make_shared<pzemCore::PZEMDevice>(model, PZEM_ID, PZEM_ADDRESS);
    Serial.printf("id of pzem device is %d", pzemDevice->getId());
    pzemDevice->autopoll(true);
    auto modbus_server = std::make_unique<ModbusServerTCPasync>();

    PZEMModbus::getInstance(std::move(modbus_server), pzemDevice).startServer();
}

void loop()
{
    if(pzemDevice)
    {
        auto mReg1 = pzemDevice->getModbusRegisters();
        mReg1->printAllRegisters();
    }
    delay(5000);
}

//   unsigned char buffer[128];
//     size_t message_length;
//     bool status = false;

//     _pz_PowerMeasure pData = pz_PowerMeasure_init_zero;

//     pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));

//     pData.voltage = 230;
//     pData.current = 1.1;
//     pData.power = 245;
//     pData.energy = 0;
//     pData.frequency = 50;
//     pData.pf = 0.8;
//     pData.alarms = 1;
//     status = pb_encode(&stream, pz_PowerMeasure_fields, &pData);
//     message_length = stream.bytes_written;
//     Serial.println(message_length);
//     Serial.println("encoding");
//     delay(5000);