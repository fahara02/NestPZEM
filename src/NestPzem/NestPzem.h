#ifndef NEST_PZEM_H
#define NEST_PZEM_H
#include <list>
#include "PZEM_Defaults.h"
#include "ModbusDefaults.h"
#include "PZEMDevice.hpp"
#include "UARTConfig.h"
#include "PzemPort.h"
#include "Poller.hpp"

static const char* NEST_PZEM __attribute__((unused)) = "NEST_PZEM";
namespace nestPzem
{
using namespace pzemCore;
using namespace tmbus;

class meterPool : public Poller<meterPool>
{
    struct nestNode
    {
        std::shared_ptr<PZEMPort> port;
        std::unique_ptr<PZEMDevice> pzem;
    };

   protected:
    std::list<std::shared_ptr<PZEMPort> > ports;  // list of registered ports
    std::list<std::shared_ptr<nestNode> > meters; // list of registered PZEM nodes

    std::shared_ptr<PZEMPort> port_by_id(uint8_t id);
    const PZEMDevice* pzem_by_id(uint8_t id) const;
    pzemCore::PZEMDevice* getPZEMForUpdate(uint8_t id);

   public:
    meterPool() :
        Poller(this, POLLER_PERIOD),
        _defaultNamePlate(),
        _defaultMeasure(PZEMModel::NOT_SET),
        _defaultJobCard(PZEMModel::NOT_SET, _defaultNamePlate)
    {
    }
    virtual ~meterPool() override
    {
        // Base class destructor will handle timer cleanup
    }
    void resetPoll() const override;
    void print() override;

    meterPool(const meterPool&) = delete;
    meterPool& operator=(const meterPool&) = delete;

    bool addPort(uint8_t _id, UART_Config& portcfg, const char* descr = nullptr);
    bool addPort(std::shared_ptr<PZEMPort> port);
    bool addPZEM(const uint8_t port_id, PZEMModel model, uint8_t id, uint8_t addr,
                 uint8_t lineNo = 0, Phase phase = Phase::SINGLE_PHASE,
                 const char* descr = nullptr);
    bool addPZEM(const uint8_t port_id, PZEMDevice* pzem);
    bool existPort(uint8_t id) { return port_by_id(id) == nullptr; }
    bool existPZEM(uint8_t id) { return pzem_by_id(id) == nullptr; }
    bool removePZEM(const uint8_t pzem_id);
    void attach_rx_callback(rx_callback_t f);
    inline void detach_rx_callback() { rx_callback = nullptr; }

    void updateMetrics();
    void resetEnergyCounter(uint8_t pzem_id);
    const JobCard& getJobCard(uint8_t id) const;
    const powerMeasure& getMeasures(uint8_t id) const;
    const char* getNameTag(uint8_t id) const;

   private:
    mutable NamePlate _defaultNamePlate;
    mutable powerMeasure _defaultMeasure;
    mutable JobCard _defaultJobCard;
    rx_callback_t rx_callback = nullptr; // external callback to trigger on RX dat

    void rx_dispatcher(const receive* msg, const uint8_t port_id);
};
} // namespace nestPzem
#endif