#ifndef PZEM_PORT_H
#define PZEM_PORT_H
#include "UARTMsgQueue.hpp"
#include "UARTConfig.h"
#include "UARTManager.hpp"
namespace pzemCore
{

class PZEMPort
{
    bool isMsgUnderProcess;
    std::unique_ptr<char[]> Descripton;

    void setDescription(const char* _name);

   public:
    const uint8_t id;
    const char* getDescription() const;
    bool active() const { return isMsgUnderProcess; }
    bool active(bool newstate);
    std::unique_ptr<tmbus::UARTMsgQueue> msgHandlerPtr = nullptr;

    PZEMPort(uint8_t _id, tmbus::UARTMsgQueue* mq, const char* _name = nullptr) : id(_id)
    {
        // std::move(mq);
        msgHandlerPtr.reset(mq);
        setDescription(_name);
        isMsgUnderProcess = msgHandlerPtr->startQueues();
    }

    // Construct a new UART port
    PZEMPort(uint8_t _id, tmbus::UART_Config& cfg, const char* _name = nullptr) : id(_id)
    {
        // Use std::make_unique for automatic memory management
        auto _pzemUART =
            std::make_unique<tmbus::UARTManager>(cfg.p, cfg.uartcfg, cfg.gpio_rx, cfg.gpio_tx);
        msgHandlerPtr = std::move(_pzemUART); // Transfer ownership
        setDescription(_name);
        isMsgUnderProcess = msgHandlerPtr->startQueues();
    }
};
} // namespace pzemCore

#endif