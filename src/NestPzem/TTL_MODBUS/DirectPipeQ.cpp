#include "DirectPipeQ.hpp"
#include "esp_log.h"
namespace tmbus
{

DirectPipeQ::~DirectPipeQ()
{
    tx_callback = nullptr;
    rx_callback = nullptr;
}

// NullQ implementation methods
void DirectPipeQ::attachTXhandler(txHandler func)
{
    if(func) tx_callback = std::move(func);
}

void DirectPipeQ::dettachTXhandler() { tx_callback = nullptr; }

bool DirectPipeQ::txEnQueue(transmit* msg)
{
    bool status = false;
    if(tx_callback)
    {
        tx_callback(msg);
        status = true;
    }

    delete msg;
    return status;
}

bool DirectPipeQ::receiveQueue(receive* msg)
{
    if(rx_callback)
    {
        rx_callback(msg);
        return true;
    }

    delete msg;
    return false;
}

uartBridge::uartBridge()
{
    portA.attachTXhandler(std::bind(&uartBridge::tx_rx, this, std::placeholders::_1, true));
    portB.attachTXhandler(std::bind(&uartBridge::tx_rx, this, std::placeholders::_1, false));
}
void uartBridge::tx_rx(transmit* tm, bool atob)
{
    auto* rmsg = new receive(tm->msg.data, tm->msg.length);
    atob ? portB.receiveQueue(rmsg) : portA.receiveQueue(rmsg);
}
} // namespace tmbus