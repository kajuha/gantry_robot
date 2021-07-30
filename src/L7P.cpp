#include <modbus.h>

#include "L7P.h"

int setSingleCoilCmd(modbus_t* ctx, SingleCoilCmd singleCoilCmd, OnOff onOff) {
    return modbus_write_bit(ctx, (int)singleCoilCmd, (int)onOff);
}

int getSingleCoils(modbus_t* ctx, uint8_t* read_bits) {
    modbus_read_bits(ctx, READ_COIL_ADDR, READ_COIL_SIZE, read_bits);
}