#ifndef PZEM_MODBUS_HPP
#define PZEM_MODBUS_HPP
#include "ModbusServerTCPasync.h"
#include "PZEMDevice.hpp"
ModbusServerTCPasync MBserver;
class PZEMModbus
{
   private:
    PZEMModbus(ModbusServerTCPasync* MBServer, pzemCore::PZEMDevice* device) {}

   public:
};

#endif