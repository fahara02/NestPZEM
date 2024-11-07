#ifndef DIRECT_PIPE_Q_HPP
#define DIRECT_PIPE_Q_HPP
#include "UARTMSGQueue.hpp"
namespace tmbus
{
class DirectPipeQ : public UARTMsgQueue
{
   public:
    DirectPipeQ(){};

    virtual ~DirectPipeQ();

    DirectPipeQ(const DirectPipeQ&) = delete;
    DirectPipeQ& operator=(const DirectPipeQ&) = delete;

    bool txEnQueue(transmit* msg) override;
    bool receiveQueue(receive* msg);
    void attachTXhandler(txHandler func);
    void dettachTXhandler();

   protected:
    txHandler tx_callback = nullptr;
};
class uartBridge
{
   private:
    void tx_rx(transmit* tm, bool atob);

   public:
    uartBridge();

    DirectPipeQ portA;
    DirectPipeQ portB;
};

} // namespace tmbus

#endif