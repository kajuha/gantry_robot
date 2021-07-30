#pragma once

#include <modbus.h>

#define READ_COIL_ADDR  0x00
#define READ_COIL_SIZE  0x40

enum class OnOff {
    on = 1,
    off = 0
};

enum class SingleCoilCmd {
    stop = 0x03, mode = 0x08,
    emg = 0x0A, a_rst = 0x0B,
    sv_on = 0x0C, start = 0x10, pause = 0x11, hstart = 0x13,
    jstart = 0x1B, jdir = 0x1C
};

int setSingleCoilCmd(modbus_t* ctx, SingleCoilCmd singleCoilCmd, OnOff onOff);

enum class ModbusCmd {
    ReadCoils, WriteSingleCoil, WriteMultiCoils,
    ReadHoldRegs, WriteSingleReg, WriteMultiRegs
};

class ModbusMsg {
    uint8_t unitId;
    ModbusCmd cmd;
    uint16_t address;
    uint16_t quantity;
    uint16_t* values;
};

int getSingleCoils(modbus_t* ctx, uint8_t* read_bits);