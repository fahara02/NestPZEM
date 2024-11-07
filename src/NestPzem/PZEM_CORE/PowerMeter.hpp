#ifndef POWER_METER_HPP
#define POWER_METER_HPP
#include "PZEM_Defaults.h"
#include "powerMeasure.h"
#include "ModbusRegisters.hpp"
#include "SystemEvents.hpp"
#include <atomic>
#include <functional>
#include <memory>
#include <cmath>
#include <array>
#include <cstring>
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "NestUtility.hpp"
#include "Poller.hpp"
#include "esp_log.h"
namespace pzemCore
{
static const char* POWER_METER_TAG __attribute__((unused)) = "POWER_METER";

// struct NamePlate
// {
//     PZEMModel model;
//     uint8_t id;
//     uint8_t slaveAddress;
//     uint8_t lineNo;
//     Phase phase;
//     std::unique_ptr<char[]> meterName;
//     NamePlate() :
//         model(PZEMModel::NOT_SET),
//         id(255),
//         slaveAddress(static_cast<uint8_t>(tmbus::SLAVE_ADDRESS::ADDR_ANY)),
//         lineNo(0),
//         phase(Phase::SINGLE_PHASE),
//         meterName(nullptr)
//     {
//     }
//     NamePlate(PZEMModel m, uint8_t id, uint8_t addr, uint8_t ln = 0, Phase p =
//     Phase::SINGLE_PHASE,
//               const char* name = nullptr) :
//         model(m), id(id), slaveAddress(addr), lineNo(ln), phase(p)
//     {
//         if(!name || !*name)
//         {
//             meterName.reset(new char[9]); // i.e. PZEM-123
//             sprintf(meterName.get(), "PZEM-%d", id);
//         }
//         else
//             meterName.reset(strcpy(new char[strlen(name) + 1], name));
//     }
//     ~NamePlate() = default;
// };
struct NamePlate
{
    PZEMModel model;
    uint8_t id;
    uint8_t slaveAddress;
    uint8_t lineNo;
    Phase phase;
    std::array<char, 9> meterName;

    NamePlate() :
        model(PZEMModel::NOT_SET),
        id(255),
        slaveAddress(static_cast<uint8_t>(tmbus::SLAVE_ADDRESS::ADDR_ANY)),
        lineNo(0),
        phase(Phase::SINGLE_PHASE),
        meterName{}
    {
        snprintf(meterName.data(), meterName.size(), "PZEM-123");
    }

    NamePlate(PZEMModel m, uint8_t id, uint8_t addr, uint8_t ln = 0, Phase p = Phase::SINGLE_PHASE,
              const char* name = nullptr) :
        model(m), id(id), slaveAddress(addr), lineNo(ln), phase(p)
    {
        if(!name || !*name)
        {
            snprintf(meterName.data(), meterName.size(), "PZEM-%d", id);
        }
        else
        {
            strncpy(meterName.data(), name, meterName.size() - 1);
            meterName.back() = '\0'; // Ensure null termination
        }
    }

    ~NamePlate() = default;
};
struct JobCard

{
    NamePlate& info;
    powerMeasure pm;
    mutable int64_t poll_us = 0;
    int64_t lastUpdate_us = 0;
    int64_t dataAge_ms = 0;
    bool dataStale = false;
    std::atomic<State> deviceState{State::INIT};

    JobCard(PZEMModel model, NamePlate& info) :
        info(info),
        pm(model),
        poll_us(0),
        lastUpdate_us(0),
        dataAge_ms(0),
        dataStale(false),
        deviceState{State::INIT}
    {
    }

    void print();
};
class PowerMeter : public Poller<pzemCore::PowerMeter>
{
   private:
    std::atomic<State> _state{State::INIT};
    // tmbus::ModbusRegisters& _registers;
    std::shared_ptr<tmbus::ModbusRegisters> _registers;

   public:
    explicit PowerMeter(std::shared_ptr<tmbus::ModbusRegisters> mr, PZEMModel m, uint8_t id,
                        uint8_t addr, uint8_t ln = 0, Phase p = Phase::SINGLE_PHASE,
                        const char* nt = nullptr) :
        Poller<PowerMeter>(this, POLLER_PERIOD),
        _state(State::INIT),
        _registers(mr),
        _nameTag(nt),
        _namePlate(m, id, addr, ln, p, nt),
        _jobcard(m, _namePlate)
    {
        // ESP_LOGI(POWER_METER_TAG, "Power meter provided register ptr set to %p",
        // _registers.get());
        _registers->registerUpdateCallback(
            [this](bool registerUpdated, tmbus::ModbusRegisters::Register* reg) {
                this->onRegisterUpdated(registerUpdated, reg);
            });
    }
    virtual ~PowerMeter() override {}
    virtual uint8_t getaddr() const;
    uint8_t getId() const { return _namePlate.id; }
    virtual bool updateMeters() = 0;
    void updateMetrics();
    virtual bool resetEnergyCounter() = 0;

    bool updateJobCard() const;
    void handleEvent(Event e);

    void resetPoll() const override;
    void print() override;

    void onRegisterUpdated(bool registerUpdated, tmbus::ModbusRegisters::Register* reg);

    const JobCard& getJobCard() const;
    const pzemCore::powerMeasure& getMeasures() const;
    bool getMeter(const tmbus::ModbusRegisters::Register* reg);
    State getState() const;

   protected:
    const char* _nameTag;
    NamePlate _namePlate;
    mutable JobCard _jobcard;
    int64_t dataAge() const;
    bool dataStale() const;

   private:
    void setState(State new_state);
};

} // namespace pzemCore
#endif