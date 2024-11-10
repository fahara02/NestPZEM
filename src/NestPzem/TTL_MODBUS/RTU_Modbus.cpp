
#include <cstring> // for memcpy
#include <utility>
#include "RTU_Modbus.hpp"
#include "ModbusMessage.h"
// using namespace pzemCore;
namespace tmbus
{

transmit* RTUModbus::sendEnergyResetMsg(uint8_t slaveAddr, uint16_t regAddr)
{
    uint8_t cmd = static_cast<uint8_t>(FunctionCode::RESET);
    _device->manageList(static_cast<uint8_t>(cmd), regAddr);
    TTLModbusMessage Msg(slaveAddr, static_cast<uint8_t>(cmd), regAddr, 1);
    size_t msgSize = TTL_MODBUS_ENERGY_RST_FRAME_SIZE;
    uint8_t* buffer = new uint8_t[msgSize];
    Msg.serializeMessage(buffer, true);
    return new transmit(buffer, msgSize);
}
transmit* RTUModbus::sendMsg(FunctionCode cmd, uint16_t regAddr, uint16_t valueOrNumRegs,
                             uint8_t slaveAddr, bool waitForReply)
{
    _device->manageList(static_cast<uint8_t>(cmd), regAddr);
    TTLModbusMessage Msg(slaveAddr, static_cast<uint8_t>(cmd), regAddr, valueOrNumRegs);
    size_t msgSize = TTL_MODBUS_NORMAL_FRAME_SIZE;
    uint8_t* buffer = new uint8_t[msgSize];
    Msg.serializeMessage(buffer, true);
    return new transmit(buffer, msgSize, waitForReply);
}

} // namespace tmbus