#include "UARTMsgQueue.hpp"
#include "esp_log.h"
namespace tmbus
{
void UARTMsgQueue::attachRXhandler(rxHandler func)
{
    if(!func) return;
    rx_callback = std::move(func);
}

void UARTMsgQueue::dettachRXhandler() { rx_callback = nullptr; }

void UARTMsgQueue::rxMsgDebug(const receive* m)
{
    if(m == nullptr || !m->msg.frame.rxFrame.byteCount)
    {
        ESP_LOGE(UART_TAG, "Invalid or zero length RX packet");
        return;
    }

    // Allocate a buffer for the formatted string
    char buff[m->msg.frame.rxFrame.byteCount * 4]; // Buffer on stack
    char* ptr = buff;

    // Format the data as a hex string
    for(uint8_t i = 0; i < m->msg.frame.rxFrame.byteCount; ++i)
    {
        ptr += sprintf(ptr, "%.2x ", m->msg.frame.rxFrame.data[i]);
    }

    ESP_LOGE(UART_TAG, "RX packet, len:%d, CRC: %s, HEX: %s", m->msg.frame.rxFrame.byteCount,
             m->isValid ? "OK" : "BAD", buff);
}

void UARTMsgQueue::txMsgDebug(const transmit* m)
{
    if(!m->msg.length)
    {
        ESP_LOGE(UART_TAG, "Zero len TX packet");
        return;
    }
    char* buff = new char[m->msg.length * 4];
    char* ptr = buff;
    for(uint8_t i = 0; i < m->msg.length; ++i)
    {
        ptr += sprintf(ptr, "%.2x ", m->msg.data[i]);
    }

    // print with ERROR severity, so no need to redefine CORE_DEBUG_LEVEL
    ESP_LOGE(UART_TAG, "TX packet, len:%d, HEX: %s", m->msg.length, buff);
    delete[] buff;
}

} // namespace tmbus