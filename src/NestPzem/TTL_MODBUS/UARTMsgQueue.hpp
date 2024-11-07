#ifndef UART_MSG_HPP
#define UART_MSG_HPP
#include <cstddef> // For size_t
#include <cstdint> // For uint8_t
#include <cassert> // For assert
#include <memory>
#include "driver/uart.h"
#include <functional>
#include "NestUtility.hpp"
#include "ModbusMessage.h"
#include "esp_log.h"

namespace tmbus
{
// ESP32 log tag
static const char* UART_TAG __attribute__((unused)) = "UartMsg";
struct UARTMsg
{
    const uint8_t* data;
    size_t length;

    UARTMsg(const uint8_t* data = nullptr, size_t length = 0) : data(data), length(length) {}
};

struct transmit
{
    UARTMsg msg;
    bool waitForReciever;
    explicit transmit(const uint8_t* data = nullptr, size_t length = 0,
                      bool receiverRequest = true) :
        msg(data, length), waitForReciever(receiverRequest)
    {
        // msg.data now points to the same data buffer passed in, no copying
        // The transmit object owns the data, so no need to allocate it again
    }
    ~transmit()
    {
        delete[] msg.data;
        msg.data = nullptr;
    }
};

struct receive
{
    // UARTMsg msg;

    const bool isValid;
    const uint8_t addr;
    const uint8_t cmd;
    const uint8_t size;
    ModbusMessage msg;

    receive(const uint8_t* data, const uint8_t size) :
        isValid(Utility::CRC::check(data, size)),
        addr(data != nullptr && size > 0 ? data[0] : 0),
        cmd(data != nullptr && size > 1 ? data[1] : 0),
        size(size),
        msg(addr, cmd, isValid)

    {
        assert(data != nullptr && size > 0 && "Invalid data or size");
        if(isValid)
        {
            if(msg.decodeMessage(data, size, true))
            {
                ESP_LOGI(UART_TAG, " Successfully decoded to modbus message");
            }
            else
            {
                ESP_LOGE(UART_TAG, "Failed to decode Modbus message.");
            }
        }
        else
        {
            ESP_LOGE(UART_TAG, "Failed in CRC check");
        }
    }

    ~receive() = default; // No manual cleanup needed
};
class UARTMsgQueue
{
   public:
    typedef std::function<void(transmit*)> txHandler;
    typedef std::function<void(receive*)> rxHandler;

    UARTMsgQueue(){};
    virtual ~UARTMsgQueue(){};

    UARTMsgQueue(const UARTMsgQueue&) = delete;
    UARTMsgQueue& operator=(const UARTMsgQueue&) = delete;

    virtual bool txEnQueue(transmit* msg) = 0;
    virtual void attachRXhandler(rxHandler func);
    virtual void dettachRXhandler();

    virtual void txMsgDebug(const transmit* m);
    virtual void rxMsgDebug(const receive* m);
    virtual bool startQueues() { return true; };
    virtual void stopQueues(){};

   protected:
    rxHandler rx_callback = nullptr;
};

} // namespace tmbus
#endif