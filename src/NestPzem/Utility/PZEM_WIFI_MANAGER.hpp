#ifndef PZEM_WIFI_MANAGER_HPP
#define PZEM_WIFI_MANAGER_HPP

#include <WiFi.h>
#include <WiFiClient.h>
#include <functional>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
namespace Utility
{
class WiFiManager
{
   private:
    WiFiManager(const char* ssid, const char* password, uint8_t maxRetries = 5,
                uint32_t timeoutMs = 10000, bool enableDebug = false);

   public:
    enum class State { IDLE, CONNECTING, CONNECTED, FAILED };

    static WiFiManager& getInstance(const char* ssid, const char* password, uint8_t maxRetries = 5,
                                    uint32_t timeoutMs = 10000, bool enableDebug = false)
    {
        static WiFiManager instance(ssid, password, maxRetries, timeoutMs, enableDebug);
        return instance;
    }

    // Delete copy constructor and assignment operator to enforce singleton
    WiFiManager(const WiFiManager&) = delete;
    WiFiManager& operator=(const WiFiManager&) = delete;

    ~WiFiManager();

    using WiFiEventHandler = std::function<void()>;
    void scanNetworks() const;
    void begin();
    void setOnConnectedHandler(WiFiEventHandler handler);
    void setOnDisconnectedHandler(WiFiEventHandler handler);
    void setOnFailureHandler(WiFiEventHandler handler);

    bool isConnected() const;
    const char* getIPAddress() const;
    State getState() const;

   private:
    void connect();
    void handleWiFiEvent(WiFiEvent_t event, WiFiEventInfo_t info);
    void log(const char* message) const;

    char _ssid[32];
    char _password[64];
    uint8_t _maxRetries;
    uint8_t _currentRetries;
    uint32_t _timeoutMs;
    State _state;
    bool _debugEnabled;
    TaskHandle_t _pollingTaskHandle;

    WiFiEventHandler _onConnected;
    WiFiEventHandler _onDisconnected;
    WiFiEventHandler _onFailure;

    void pollWiFiStatus();
};
} // namespace Utility
#endif // WIFI_MANAGER_HPP
