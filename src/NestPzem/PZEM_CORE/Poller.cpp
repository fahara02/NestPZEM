#include "Poller.hpp"
#include "PowerMeter.hpp"
#include "NestPzem.h"
#ifndef pdTICKS_TO_MS
    #define pdTICKS_TO_MS(xTicks) (((TickType_t)(xTicks)*1000u) / configTICK_RATE_HZ)
#endif

using namespace pzemCore;
using namespace nestPzem;
template class Poller<PowerMeter>;
template class Poller<meterPool>;

template <typename T>
Poller<T>::~Poller()
{
    if(pollTimerhandler)
    {
        if(xTimerIsTimerActive(pollTimerhandler) == pdTRUE)
        {
            xTimerStop(pollTimerhandler, TIMER_CMD_TIMEOUT);
        }
        if(xTimerDelete(pollTimerhandler, TIMER_CMD_TIMEOUT) != pdPASS)
        {
            ESP_LOGE(POLLER_TAG, "Error: Timer deletion failed");
        }
        pollTimerhandler = nullptr; // Prevents double-free
    }
}

template <typename T>
bool Poller<T>::isPolling() const
{
    if(pollTimerhandler && xTimerIsTimerActive(pollTimerhandler) != pdFALSE) return true;

    return false;
}
template <typename T>
bool Poller<T>::autopoll(bool newstate)
{
    if(newstate)
    {
        // Check if the timer handler is null and create the timer if needed
        if(!pollTimerhandler)
        {
            pollTimerhandler =
                xTimerCreate(POLLER_NAME,                     // Name of the poller
                             pdMS_TO_TICKS(poll_period),      // Period of the timer in ticks
                             pdTRUE,                          // Auto-reload
                             reinterpret_cast<void*>(Device), // Timer ID
                             &pzemCore::Poller<T>::timeKeeper // Callback function
                );

            if(!pollTimerhandler)
            {
                ESP_LOGE(POLLER_TAG, "Error: Timer creation failed");
                return false;
            }
        }

        // Start the timer if it is not already active
        if(xTimerIsTimerActive(pollTimerhandler) == pdFALSE)
        {
            if(xTimerStart(pollTimerhandler, TIMER_CMD_TIMEOUT) != pdPASS)
            {
                ESP_LOGE(POLLER_TAG, "Error: Timer start failed");
                return false;
            }
        }

        return true;
    }
    else
    {
        // If the timer exists, stop and delete it
        if(pollTimerhandler)
        {
            if(xTimerIsTimerActive(pollTimerhandler) == pdTRUE)
            {
                xTimerStop(pollTimerhandler, TIMER_CMD_TIMEOUT);
            }

            if(xTimerDelete(pollTimerhandler, TIMER_CMD_TIMEOUT) != pdPASS)
            {
                ESP_LOGE(POLLER_TAG, "Error: Timer deletion failed");
                return false;
            }
            else
            {
                pollTimerhandler = nullptr; // Prevent re-deletion
            }
        }

        return true;
    }
}

template <typename T>
size_t Poller<T>::getPollrate() const
{
    if(pollTimerhandler) return pdTICKS_TO_MS(xTimerGetPeriod(pollTimerhandler));
    return 0;
}
template <typename T>
bool Poller<T>::setPollrate(size_t pollTime)
{
    if(pollTime < POLLER_MIN_PERIOD) return false;

    if(xTimerChangePeriod(pollTimerhandler, pollTime / portTICK_PERIOD_MS, TIMER_CMD_TIMEOUT)
       == pdPASS)
        return true;

    return false;
}
