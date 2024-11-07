#ifndef PZEM_CLIENT_HPP
#define PZEM_CLIENT_HPP

#include <cmath>
#include <cstring>
#include "UARTMsgQueue.hpp"
#include "ModbusDefaults.h"
#include "PZEM_Defaults.h"
#include "PZEM_constants.h"

typedef std::function<void(uint8_t id, const tmbus::receive*)> rx_callback_t;

namespace pzemCore
{
static const char* PZEM_TAG __attribute__((unused)) = "PZEM";

class PZEMClient
{
    bool sink_lock = false; // flag marking rx_sink as an active call-back when attached
   protected:
    tmbus::UARTMsgQueue* txMsgQueue = nullptr; // UartQ sink for TX messages
    rx_callback_t rx_callback = nullptr;       // external callback to trigger on RX data

   public:
    const uint8_t id;
    bool active = true;
    explicit PZEMClient(uint8_t _id) : id(_id) {}
    virtual ~PZEMClient();

    void attachMsgHandler(tmbus::UARTMsgQueue* mq, bool tx_only = false);
    void detachMsgHandler();

    inline void detach_rx_callback() { rx_callback = nullptr; };
    void attach_rx_callback(rx_callback_t f);

    virtual void rx_sink(const tmbus::receive* msg) = 0;

   private:
};
} // namespace pzemCore
#endif