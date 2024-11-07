#ifndef UART_MANAGER_HPP
#define UART_MANAGER_HPP
#include "UARTConfig.h"
#include "UARTMsgQueue.hpp"

namespace tmbus
{

class UARTManager : public UARTMsgQueue
{
   public:
    UARTManager(const uart_port_t p, const uart_config_t cfg, int gpio_rx = UART_PIN_NO_CHANGE,
                int gpio_tx = UART_PIN_NO_CHANGE) :
        port(p)
    {
        init(cfg, gpio_rx, gpio_tx);
    }

    UARTManager(const uart_port_t p, int gpio_rx = UART_PIN_NO_CHANGE,
                int gpio_tx = UART_PIN_NO_CHANGE) :
        port(p)
    {
        uart_config_t ucfg = uartDataFrame().configure();
        init(ucfg, gpio_rx, gpio_tx);
    }

    virtual ~UARTManager();

    const uart_port_t port;

    bool startQueues() override;
    void stopQueues() override;
    bool txEnQueue(transmit* msg) override;
    void attachRXhandler(rxHandler f) override;
    void dettachRXhandler() override;
    void init(const uart_config_t& uartcfg, int gpio_rx, int gpio_tx);
    UARTManager(const UARTManager&) = delete;
    UARTManager& operator=(const UARTManager&) = delete;

   private:
    TaskHandle_t rxTaskHandle = nullptr;
    TaskHandle_t txTaskHandle = nullptr;
    SemaphoreHandle_t sendNextMutex;

    QueueHandle_t rxMsgQueue = nullptr;
    QueueHandle_t txMsgQueue = nullptr;

    bool start_RX_MessageQueue();
    void stop_RX_MessageQueue();
    bool start_TX_MessageQueue();
    void stop_TX_MessageQueue();

    void rxQueueHandler();
    void txQueueHandler();

    static void rxTask(void* pvParams)
    {
        (reinterpret_cast<UARTManager*>(pvParams))->rxQueueHandler();
    }

    static void txTask(void* pvParams)
    {
        (reinterpret_cast<UARTManager*>(pvParams))->txQueueHandler();
    }

    // helper functions
    void handleRXEvent(const uart_event_t& event);
    void handleTxPacket(transmit* packet);
    void handleUartData();

    bool createTask(TaskFunction_t taskFunc, const char* taskName, TaskHandle_t& taskHandle);
    void stopTask(TaskHandle_t& taskHandle);
    void clearAndDeleteQueue(QueueHandle_t& queue, bool isTxQueue);
    void flushRxBuffer();
    void resetRxQueue();
    size_t getBufferedDataLength();
    uint8_t* allocateBuffer(size_t datalen);
    size_t readUartData(uint8_t* buff, size_t datalen);
    void processUartData(uint8_t* buff, size_t datalen);
};
} // namespace tmbus
#endif