#ifndef POLLER_HPP
#define POLLER_HPP
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "PZEM_Defaults.h"
#include "Arduino.h"
namespace pzemCore
{
static const char* POLLER_TAG __attribute__((unused)) = "POLLER";
#define POLLER_NAME "PZEM_POLLER"
template <typename T>
class Poller
{
   public:
    Poller(T* device, size_t pollPeriod = POLLER_PERIOD) :
        Device(device), poll_period(pollPeriod), pollTimerhandler(nullptr)
    {
    }

    virtual ~Poller();

    virtual void resetPoll() const = 0;
    virtual void print() = 0;

    bool isPolling() const;
    bool autopoll(bool newstate);
    size_t getPollrate() const;
    bool setPollrate(size_t pollTime);

   private:
    T* Device;
    size_t poll_period;
    TimerHandle_t pollTimerhandler;

    static void timeKeeper(TimerHandle_t xTimer)
    {
        if(!xTimer) return;

        T* device = static_cast<T*>(pvTimerGetTimerID(xTimer));
        if(device)
        {
            

            device->updateMetrics();
           // device->print();
        }
    };
};

} // namespace pzemCore
#endif