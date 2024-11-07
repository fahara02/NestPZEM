#include "UARTManager.hpp"
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include "ModbusDefaults.h"
namespace tmbus
{

void UARTManager::init(const uart_config_t& uartcfg, int gpio_rx, int gpio_tx)
{
    // TODO: catch port init errors
    uart_param_config(port, &uartcfg);
    uart_set_pin(port, gpio_tx, gpio_rx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(port, RX_BUFFER_SIZE, TX_BUFFER_SIZE, rx_msg_q_DEPTH, &rxMsgQueue, 0);
    sendNextMutex = xSemaphoreCreateBinary(); // Ready-To-Send-next semaphore
}
bool UARTManager::startQueues() { return start_RX_MessageQueue() && start_TX_MessageQueue(); };
void UARTManager::stopQueues()
{
    stop_RX_MessageQueue();
    stop_TX_MessageQueue();
}

bool UARTManager::createTask(TaskFunction_t taskFunc, const char* taskName,
                             TaskHandle_t& taskHandle)
{
    return xTaskCreate(taskFunc, taskName, EVT_TASK_STACK, reinterpret_cast<void*>(this),
                       EVT_TASK_PRIO, &taskHandle)
           == pdPASS;
}
void UARTManager::stopTask(TaskHandle_t& taskHandle)
{
    if(taskHandle)
    {
        vTaskDelete(taskHandle);
        taskHandle = nullptr;
    }
}

bool UARTManager::start_RX_MessageQueue()
{
    if(!rxMsgQueue) return false;

    if(!rxTaskHandle)
    {
        return createTask(UARTManager::rxTask, EVT_TASK_NAME, rxTaskHandle);
    }

    return true;
}

bool UARTManager::start_TX_MessageQueue()
{
    if(txMsgQueue) return true; // Check if queue is allocated

    txMsgQueue = xQueueCreate(tx_msg_q_DEPTH, sizeof(transmit*));
    if(!txMsgQueue) return false;

    if(!txTaskHandle)
    {
        return createTask(UARTManager::txTask, EVT_TASK_NAME, txTaskHandle);
    }

    return true;
}

void UARTManager::stop_TX_MessageQueue()
{
    if(txTaskHandle)
    {
        vTaskDelete(txTaskHandle);
        txTaskHandle = nullptr;
    }

    if(!txMsgQueue) return;

    QueueHandle_t _t = txMsgQueue;
    txMsgQueue = nullptr;

    transmit* msg = nullptr;
    while(xQueueReceive(_t, &(msg), (TickType_t)0) == pdPASS)
    {
        delete msg;
    }

    vQueueDelete(_t);
}

void UARTManager::stop_RX_MessageQueue()
{
    if(rxTaskHandle)
    {
        vTaskDelete(rxTaskHandle);
        rxTaskHandle = nullptr;
    }
}

size_t UARTManager::getBufferedDataLength()
{
    size_t datalen = 0;
    ESP_ERROR_CHECK(uart_get_buffered_data_len(port, &datalen));
    return datalen;
}

uint8_t* UARTManager::allocateBuffer(size_t datalen)
{
    uint8_t* buff = static_cast<uint8_t*>(malloc(datalen));
    return buff;
}

size_t UARTManager::readUartData(uint8_t* buff, size_t datalen)
{
    size_t bytesRead = uart_read_bytes(port, buff, datalen, pzemCore::PZEM_UART_RX_READ_TICKS);
    if(bytesRead == 0)
    {
        ESP_LOGD(UART_TAG, "Unable to read data from RX buffer");
        delete[] buff;
    }
    return bytesRead;
}

void UARTManager::processUartData(uint8_t* buff, size_t datalen)
{
    if(datalen > 0)
    {
        receive* msg = new receive(buff, datalen);
        // ESP_LOGD(TAG, "Got RX data packet from buffer, len: %d, t: %ld", datalen,
        // 		 esp_timer_get_time() / 1000);
        rxMsgDebug(msg);
        rx_callback(msg);
    }
}

bool UARTManager::txEnQueue(transmit* msg)
{
    if(!msg) return false;

    if(!txMsgQueue)
    {
        delete msg;
        return false;
    }

    //	ESP_LOGD(TAG, "TX packet enque, t: %ld", esp_timer_get_time() / 1000);

    if(xQueueSendToBack(txMsgQueue, (void*)&msg, (TickType_t)0) == pdTRUE)
        return true;
    else
    {
        delete msg;
        return false;
    }
}

void UARTManager::attachRXhandler(rxHandler func)
{
    if(!func) return;
    rx_callback = std::move(func);
    start_RX_MessageQueue();
}

void UARTManager::dettachRXhandler()
{
    rx_callback = nullptr;
    stop_RX_MessageQueue();
}

void UARTManager::handleRXEvent(const uart_event_t& event)
{
    switch(event.type)
    {
        case UART_DATA:
            handleUartData();
            break;
        case UART_FIFO_OVF:
            ESP_LOGW(UART_TAG, "UART RX FIFO overflow!");
            resetRxQueue();
            break;
        case UART_BUFFER_FULL:
            ESP_LOGW(UART_TAG, "UART RX ring buffer full");
            flushRxBuffer();
            resetRxQueue();
            break;
        case UART_BREAK:
        case UART_FRAME_ERR:
            ESP_LOGW(UART_TAG, "UART RX error");
            break;
        default:
            break;
    }
}

void UARTManager::handleUartData()
{
    if(!rx_callback)
    {
        flushRxBuffer();
        resetRxQueue();
        return;
    }

    size_t datalen = getBufferedDataLength();
    if(datalen == 0)
    {
        flushRxBuffer();
        resetRxQueue();
        return;
    }

    uint8_t* buff = allocateBuffer(datalen);
    if(buff)
    {
        datalen = readUartData(buff, datalen);
        processUartData(buff, datalen);
    }
    else
    {
        flushRxBuffer();
    }
}
void UARTManager::rxQueueHandler()
{
    for(;;)
    {
        xSemaphoreGive(sendNextMutex);
        uart_event_t event;

        if(xQueueReceive(rxMsgQueue, &event, portMAX_DELAY))
        {
            handleRXEvent(event);
        }
    }

    vTaskDelete(NULL);
}

void UARTManager::handleTxPacket(transmit* packet)
{
    if(packet->waitForReciever)
    {
        ESP_LOGD(UART_TAG, "Waiting for TX semaphore, t: %lld", esp_timer_get_time() / 1000);
        xSemaphoreTake(sendNextMutex, pdMS_TO_TICKS(pzemCore::PZEM_UART_TIMEOUT));
    }

    uart_write_bytes(port, reinterpret_cast<const char*>(packet->msg.data), packet->msg.length);
    // ESP_LOGD(TAG, "TX packet sent to UART FIFO, t: %ld", esp_timer_get_time() / 1000);
    txMsgDebug(packet);

    delete packet; // Clean up the packet after transmission
}

void UARTManager::txQueueHandler()
{
    for(;;)
    {
        transmit* packet = nullptr;

        if(xQueueReceive(txMsgQueue, &packet, portMAX_DELAY))
        {
            handleTxPacket(packet);
        }
    }

    vTaskDelete(NULL);
}
//  the helpers

void UARTManager::clearAndDeleteQueue(QueueHandle_t& queue, bool isTxQueue)
{
    if(!queue) return;

    if(isTxQueue)
    {
        transmit* msg = nullptr;
        while(xQueueReceive(queue, &msg, (TickType_t)0) == pdPASS)
        {
            delete msg;
        }
    }
    vQueueDelete(queue);
    queue = nullptr;
}

void UARTManager::flushRxBuffer() { uart_flush_input(port); }

void UARTManager::resetRxQueue() { xQueueReset(rxMsgQueue); }

// void UartBase::txQueueHandler()
// {
// 	transmit* packet = nullptr;

// 	for(;;)
// 	{
// 		// 'xQueueReceive' will "sleep" untill some message arrives from the msg
// 		// queue
// 		if(xQueueReceive(txMsgQueue, &(packet), portMAX_DELAY))
// 		{
// 			// if smg would expect a reply than I need to grab a semaphore from the
// 			// RX queue task
// 			if(packet->waitForReciever)
// 			{
// 				ESP_LOGD(TAG, "Wait for tx semaphore, t: %lld", esp_timer_get_time() / 1000);
// 				xSemaphoreTake(sendNextMutex, pdMS_TO_TICKS(pzemCore::PZEM_UART_TIMEOUT));
// 				// an old reply migh be still in the rx queue while I'm handling this
// 				// one
// 				// uart_flush_input(port);     // input should be cleared from any
// 				// leftovers if I expect a reply (in case of a timeout only)
// 				// xQueueReset(rxMsgQueue);
// 			}

// 			// Send message data to the UART TX FIFO
// 			uart_write_bytes(port, (const char*)packet->msg.data, packet->msg.length);

// 			ESP_LOGD(TAG, "TX - packet sent to uart FIFO, t: %ld", esp_timer_get_time() / 1000);
// 			txMsgDebug(packet);

// 			// destroy message
// 			delete packet;
// 			packet = nullptr;
// 		}
// 	}

// 	vTaskDelete(NULL);
// }

// void UartBase::rxQueueHandler()
// {
// 	uart_event_t event;

// 	// Task runs inside Infinite loop
// 	for(;;)
// 	{
// 		xSemaphoreGive(sendNextMutex);

// 		// 'xQueueReceive' will "sleep" untill an event messages arrives from the
// 		// RX event queue
// 		if(xQueueReceive(rxMsgQueue, reinterpret_cast<void*>(&event), portMAX_DELAY))
// 		{
// 			// Handle received event
// 			switch(event.type)
// 			{
// 				case UART_DATA:
// 				{
// 					if(!rx_callback)
// 					{ // if there is no RX handler, than discard all RX
// 						uart_flush_input(port);
// 						xQueueReset(rxMsgQueue);
// 						break;
// 					}

// 					size_t datalen = 0;
// 					ESP_ERROR_CHECK(uart_get_buffered_data_len(port, &datalen));
// 					if(0 == datalen)
// 					{
// 						ESP_LOGD(TAG, "can't retreive RX data from buffer, t: %lld",
// 								 esp_timer_get_time() / 1000);
// 						uart_flush_input(port);
// 						xQueueReset(rxMsgQueue);
// 						break;
// 					}

// 					ESP_LOGD(TAG, "RX buff has %u bytes data msg, t: %lld", datalen,
// 							 esp_timer_get_time() / 1000);

// 					uint8_t* buff = (uint8_t*)malloc(datalen);
// 					if(buff)
// 					{
// 						datalen =
// 							uart_read_bytes(port, buff, datalen, pzemCore::PZEM_UART_RX_READ_TICKS);
// 						if(!datalen)
// 						{
// 							ESP_LOGD(TAG, "unable to read data from RX buff");
// 							delete[] buff;
// 							uart_flush_input(port);
// 							xQueueReset(rxMsgQueue);
// 							break;
// 						}

// 						receive* msg = new receive(buff, datalen);

// 						ESP_LOGD(TAG, "got RX data packet from buff, len: %d, t: %ld", datalen,
// 								 esp_timer_get_time() / 1000);
// 						rxMsgDebug(msg);

// 						rx_callback(msg); // call external function to process PZEM message
// 					}
// 					else
// 						uart_flush_input(port);

// 					break;
// 				}
// 				case UART_FIFO_OVF:
// 					ESP_LOGW(TAG, "UART RX fifo overflow!");
// 					xQueueReset(rxMsgQueue);
// 					break;
// 				case UART_BUFFER_FULL:
// 					ESP_LOGW(TAG, "UART RX ringbuff full");
// 					uart_flush_input(port);
// 					xQueueReset(rxMsgQueue);
// 					break;
// 				case UART_BREAK:
// 				case UART_FRAME_ERR:
// 					ESP_LOGW(TAG, "UART RX err");
// 					break;
// 				default:
// 					break;
// 			}
// 		}
// 	}

// 	vTaskDelete(NULL);
// }

} // namespace tmbus