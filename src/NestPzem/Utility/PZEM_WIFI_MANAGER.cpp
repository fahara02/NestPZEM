#include "PZEM_WIFI_MANAGER.hpp"
#include <WiFi.h>
namespace Utility
{

WiFiManager::WiFiManager(const char* ssid, const char* password, uint8_t maxRetries,
                         uint32_t timeoutMs, bool enableDebug) :
    _maxRetries(maxRetries),
    _currentRetries(0),
    _timeoutMs(timeoutMs),
    _state(State::IDLE),
    _debugEnabled(enableDebug),
    _pollingTaskHandle(nullptr)
{
    strncpy(_ssid, ssid, sizeof(_ssid) - 1);
    strncpy(_password, password, sizeof(_password) - 1);
    _ssid[sizeof(_ssid) - 1] = '\0';
    _password[sizeof(_password) - 1] = '\0';

    WiFi.onEvent([this](WiFiEvent_t event, WiFiEventInfo_t info) { handleWiFiEvent(event, info); });
}

WiFiManager::~WiFiManager()
{
    if(_pollingTaskHandle)
    {
        vTaskDelete(_pollingTaskHandle);
    }
    WiFi.disconnect(true);
}
void WiFiManager::scanNetworks() const
{
    int networkCount = WiFi.scanNetworks();

    log("Scanning WiFi Networks...");

    for(int i = 0; i < networkCount; ++i)
    {
        char buffer[128];
        snprintf(buffer, sizeof(buffer), "SSID: %s, RSSI: %d dBm", WiFi.SSID(i).c_str(),
                 WiFi.RSSI(i));
        log(buffer);
    }
}
void WiFiManager::begin()
{
    connect();
    if(_pollingTaskHandle == nullptr)
    {
        xTaskCreatePinnedToCore(
            [](void* pvParameters) {
                WiFiManager* instance = static_cast<WiFiManager*>(pvParameters);
                instance->pollWiFiStatus();
            },
            "WiFiPollingTask",
            4096, // Stack size
            this, // Task parameter
            1,    // Priority
            &_pollingTaskHandle,
            1 // Run on core 1
        );
    }
}
void WiFiManager::connect()
{
    _state = State::CONNECTING;
    WiFi.begin(_ssid, _password);
    log("Attempting to connect...");
}
void WiFiManager::pollWiFiStatus()
{
    unsigned long startMillis = millis();
    while(true)
    {
        if(_state == State::CONNECTING)
        {
            if(WiFi.isConnected() && WiFi.localIP().toString() != "0.0.0.0")
            {
                _state = State::CONNECTED;
                log("Connected to WiFi.");
                log(WiFi.localIP().toString().c_str());
                if(_onConnected)
                {
                    _onConnected();
                }
            }
            else if(_currentRetries >= _maxRetries || (millis() - startMillis > _timeoutMs))
            {
                _state = State::FAILED;
                log("Failed to connect to WiFi after retries or timeout.");
                ESP.restart(); // Restart on failure
            }
            else
            {
                ++_currentRetries;
                log("Retrying connection...");
                connect();
            }
        }
        else if(!WiFi.isConnected() && _state == State::CONNECTED)
        {
            _state = State::IDLE;
            log("WiFi disconnected. Reconnecting...");
            if(_onDisconnected)
            {
                _onDisconnected();
            }
            connect();
        }

        vTaskDelay(pdMS_TO_TICKS(2000)); // Check status every 2 seconds
    }
}

void WiFiManager::handleWiFiEvent(WiFiEvent_t event, WiFiEventInfo_t info)
{
    switch(event)
    {
        case SYSTEM_EVENT_STA_DISCONNECTED:
            log("WiFi Disconnected.");
            _state = State::IDLE;
            if(_onDisconnected)
            {
                _onDisconnected();
            }
            break;
        case SYSTEM_EVENT_STA_GOT_IP:
            log("WiFi Got IP Address.");
            break;
        default:
            break;
    }
}
void WiFiManager::setOnConnectedHandler(WiFiEventHandler handler) { _onConnected = handler; }

void WiFiManager::setOnDisconnectedHandler(WiFiEventHandler handler) { _onDisconnected = handler; }

void WiFiManager::setOnFailureHandler(WiFiEventHandler handler) { _onFailure = handler; }

bool WiFiManager::isConnected() const { return WiFi.isConnected(); }

const char* WiFiManager::getIPAddress() const
{
    static char ipAddress[16];
    if(isConnected())
    {
        snprintf(ipAddress, sizeof(ipAddress), "%s", WiFi.localIP().toString().c_str());
        return ipAddress;
    }
    return "0.0.0.0";
}

WiFiManager::State WiFiManager::getState() const { return _state; }

void WiFiManager::log(const char* message) const
{
    if(_debugEnabled)
    {
        Serial.printf("%s\n", message);
    }
}

} // namespace Utility
