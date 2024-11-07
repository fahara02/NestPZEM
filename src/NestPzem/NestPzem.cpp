#include "NestPzem.h"
#include "esp_log.h"
namespace nestPzem
{

bool meterPool::addPort(uint8_t _id, UART_Config& portcfg, const char* descr)
{
    if(port_by_id(_id)) return false; // port with such id already exist

    auto p = std::make_shared<PZEMPort>(_id, portcfg, descr);
    return addPort(p);
}
bool meterPool::addPort(std::shared_ptr<PZEMPort> port)
{
    if(port_by_id(port->id)) return false; // port with such id already exist

    uint8_t portid = port->id;
    ports.emplace_back(port);

    // RX handler lambda catches port-id here and suppies this id to the handler function
    port->msgHandlerPtr->attachRXhandler([this, portid](receive* msg) {
        if(!msg) return;
        rx_dispatcher(msg, portid);
        delete msg;
    });

    return true;
}
bool meterPool::addPZEM(const uint8_t port_id, PZEMDevice* pzem)
{
    if(pzem->isValidAddress(pzem->getaddr())) return false;

    auto port = port_by_id(port_id);
    if(!port) return false;

    auto node = std::make_shared<nestNode>();
    node->port = port;
    pzem->detach_rx_callback();
    pzem->detachMsgHandler();
    pzem->attachMsgHandler(node->port.get()->msgHandlerPtr.get(), true);
    node->pzem.reset(std::move(pzem));
    meters.emplace_back(std::move(node));
    return true;
}
bool meterPool::addPZEM(const uint8_t port_id, PZEMModel model, uint8_t id, uint8_t addr,
                        uint8_t lineNo, Phase phase, const char* descr)
{
    if(addr < static_cast<uint8_t>(SLAVE_ADDRESS::ADDR_MIN)
       || addr > static_cast<uint8_t>(SLAVE_ADDRESS::ADDR_MAX)) // we do not want any broadcasters
                                                                // or wrong addresses in our pool
        return false;

    if(!port_by_id(port_id) || pzem_by_id(id))
        return false; // either port is missing or pzem with this id already exist

    PZEMDevice* pzem;
    pzem = new PZEMDevice(model, id, addr, lineNo, phase, descr);

    if(addPZEM(port_id, pzem))
        return true;
    else
    {
        delete pzem;
        return false;
    }
};

bool meterPool::removePZEM(const uint8_t pzem_id)
{
    for(auto i = meters.begin(); i != meters.end(); ++i)
    {
        if((*i)->pzem->id == pzem_id)
        {
            meters.erase(i);
            return true;
        }
    }
    return false;
}
void meterPool::rx_dispatcher(const receive* msg, const uint8_t port_id)
{
    if(!msg->isValid)
    {
        ESP_LOGW(NEST_PZEM, "RX packet CRC err");

        return;
    }

    for(const auto& i: meters)
    {
        if(i->pzem->getaddr() == msg->addr && i->port->id == port_id)
        {
            ESP_LOGD(NEST_PZEM, "Got match PZEM Node for port:%d , addr:%d\n", port_id, msg->addr);

            i->pzem->rx_sink(msg);

            if(rx_callback)
                rx_callback(i->pzem->id, msg); // run external call-back function (if set)
            return;
        }
    }

    ESP_LOGD(NEST_PZEM, "Stray packet, no matching PZEM found");
}

std::shared_ptr<PZEMPort> meterPool::port_by_id(uint8_t id)
{
    for(auto i: ports)
    {
        if(i->id == id) return i;
    }

    return nullptr;
}
void meterPool::attach_rx_callback(rx_callback_t f)
{
    if(!f) return;
    rx_callback = std::move(f);
}

const PZEMDevice* meterPool::pzem_by_id(uint8_t id) const
{
    for(auto i = meters.cbegin(); i != meters.cend(); ++i)
    {
        if(i->get()->pzem->id == id) return i->get()->pzem.get();
    }

    return nullptr;
}

void meterPool::print()
{
    for(const auto& meter: meters)
    {
        if(meter && meter->pzem)
        {
            meter->pzem->print();
        }
    }
}

void meterPool::resetPoll() const
{
    for(const auto& meter: meters)
    {
        if(meter && meter->pzem)
        {
            meter->pzem->resetPoll();
        }
    }
}
const char* meterPool::getNameTag(uint8_t id) const
{
    const pzemCore::PZEMDevice* pzem = pzem_by_id(id);
    if(pzem)
    {
        return pzem->getNameTag();
    }

    return nullptr;
};

const JobCard& meterPool::getJobCard(uint8_t id) const
{
    const auto* pzem = pzem_by_id(id);
    if(pzem) return pzem->getJobCard();

    return _defaultJobCard;
};

const pzemCore::powerMeasure& meterPool::getMeasures(uint8_t id) const
{
    const auto* pzem = pzem_by_id(id);

    if(pzem) return pzem->getMeasures();

    return _defaultMeasure;
}
void meterPool::updateMetrics()
{
    for(auto i = meters.cbegin(); i != meters.cend(); ++i)
    {
        uint8_t id = i->get()->pzem->id;
        const auto* pzem = pzem_by_id(id);
        if(pzem) pzem->updateJobCard();
    }
}
void meterPool::resetEnergyCounter(uint8_t pzem_id)
{
    for(auto& i: meters)
    {
        if(i->pzem->id == pzem_id)
        {
            i->pzem->resetEnergyCounter();
            return;
        }
    }
}

} // namespace nestPzem