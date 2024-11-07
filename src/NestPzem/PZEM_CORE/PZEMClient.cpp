#include "PZEMClient.hpp"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"

namespace pzemCore
{
PZEMClient::~PZEMClient()
{
    ESP_LOGD(PZEM_TAG, "PZEM deconstruct, id: %d", id);

    if(sink_lock) detachMsgHandler();
}
void PZEMClient::attachMsgHandler(tmbus::UARTMsgQueue* mq, bool tx_only)
{
    if(!mq
       || txMsgQueue) // check if either new port is empty or there is already exist an attachment
        return;

    txMsgQueue = mq;

    if(tx_only) return;

    txMsgQueue->attachRXhandler([this](tmbus::receive* msg) {
        rx_sink(msg);
        delete msg; // must delete the message once processed, otherwise it will leak mem
    });
    sink_lock = true;
}

void PZEMClient::detachMsgHandler()
{
    if(!txMsgQueue) return;
    if(sink_lock) txMsgQueue->dettachRXhandler();

    txMsgQueue = nullptr;
    sink_lock = false;
}
void PZEMClient::attach_rx_callback(rx_callback_t f)
{
    if(!f) return;
    rx_callback = std::move(f);
}

} // namespace pzemCore