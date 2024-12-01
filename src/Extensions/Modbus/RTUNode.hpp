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
    std::unique_ptr<ModbusClientRTU> _mbRTU;
    std::unique_ptr<ModbusBridgeWiFi> _mbBridge;
    uint16_t _timeout;
    SemaphoreHandle_t _rtuMutex;

    // Private constructor for Meyers singleton
    RTUNode(uint16_t timeout = 2000) :
        _mbRTU(std::make_unique<ModbusClientRTU>()),
        _mbBridge(std::make_unique<ModbusBridgeWiFi>()),
        _timeout(timeout),
        _rtuMutex(xSemaphoreCreateMutex())
    {
        init();
    }

    void init()
    {
        // Prepare Serial for RTU communication
        RTUutils::prepareHardwareSerial(RelayBoardSerial);
        RelayBoardSerial.begin(9600, SERIAL_8N1, RTU_RX_PIN, RTU_TX_PIN);

        // Initialize ModbusRTU client
        _mbRTU->setTimeout(_timeout);
        _mbRTU->begin(RelayBoardSerial, 1);

        // Initialize Modbus bridge
        _mbBridge->attachServer(4, 1, ANY_FUNCTION_CODE, _mbRTU.get());
        _mbBridge->denyFunctionCode(4, 6); // Example denial
        _mbBridge->attachServer(5, 4, READ_HOLD_REGISTER, _mbRTU.get());
        _mbBridge->addFunctionCode(5, READ_INPUT_REGISTER);
    }

   public:
    // Delete copy constructor and assignment operator
    RTUNode(const RTUNode&) = delete;
    RTUNode& operator=(const RTUNode&) = delete;

    // Meyers singleton instance accessor
    static RTUNode& getInstance(uint16_t timeout = 2000)
    {
        static RTUNode instance(timeout);
        return instance;
    }

    void startBridge(uint16_t port, uint16_t maxClients, uint16_t inactivityTimeout)
    {
        if(_mbBridge)
        {
            _mbBridge->start(port, maxClients, inactivityTimeout);
        }
    }

    void listServers()
    {
        if(_mbBridge)
        {
            _mbBridge->listServer();
        }
    }

    SemaphoreHandle_t getMutex() const { return _rtuMutex; }
};

#endif // RTU_NODE
