#pragma once

#ifndef SIM_MODBUS
#include <modbus.h>
#else
#include "sim_modbus.h"
#endif

#include "L7P.h"

#define MAX_DATA_SIZE   2   // DINT, double int, int = 16 bits, total 32 bits <-- modbus

#define ACT_POS 0x600E
#define ACT_SPD 0x6018

enum class ObjType {
    Unknown = 0, FP32 = 0, STRING = 0,
    DINT = 2, INT32 = 2, UDINT = 2,
    INT = 1, SINT = 1, UINT = 1, UINT16 = 1, USINT = 1,
};

enum class ObjAccess {
    RW, RO
};

int32_t getObjType(int32_t address);
int32_t setAxisParameter(modbus_t* ctx, int32_t id, int32_t index, int32_t value, OnOff read_check);
int32_t getAxisParameter(modbus_t* ctx, int32_t id, int32_t index, int32_t* value);