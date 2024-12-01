#ifndef RTU_NODE
#define RTU_NODE
#include "stdint.h"
#include "Arduino.h"
#include "ModbusClientRTU.h"
#include "ModbusBridgeWiFi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <memory>
#include <utility>

constexpr uint16_t RTU_RX_PIN = 25;
constexpr uint16_t RTU_TX_PIN = 26;
HardwareSerial RelayBoardSerial(2);

class RTUNode

{
   private:
    uint16_t _timeout;
    std::unique_ptr<ModbusClientRTU> _mbRTU;
    std::unique_ptr<ModbusBridgeWiFi> _mbBridge;
    SemaphoreHandle_t _rtuMutex;

    RTUNode(std::unique_ptr<ModbusClientRTU> rtu, std::unique_ptr<ModbusBridgeWiFi> bridge,
            uint16_t timeout = 2000) :
        _mbRTU(std::move(rtu)), _mbBridge(std::move(bridge)), _timeout(timeout)
    {
        _rtuMutex = xSemaphoreCreateMutex();
        init();
    }
    void init(){};

   public:
};

#endif