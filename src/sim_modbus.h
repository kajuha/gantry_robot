#pragma once

#include <ros/ros.h>

typedef int32_t modbus_t;

modbus_t* modbus_new_rtu(const char *device, int baud, char parity, int data_bit, int stop_bit);

int modbus_connect(modbus_t *ctx);
void modbus_close(modbus_t *ctx);

void modbus_free(modbus_t *ctx);

void modbus_set_debug(modbus_t *ctx, int boolean);

const char *modbus_strerror(int errnum);

int modbus_set_slave(modbus_t* ctx, int slave);

int modbus_read_bits(modbus_t *ctx, int addr, int nb, uint8_t *dest);
int modbus_read_registers(modbus_t *ctx, int addr, int nb, uint16_t *dest);
int modbus_write_bit(modbus_t *ctx, int addr, int status);
int modbus_write_registers(modbus_t *ctx, int addr, int nb, const uint16_t *data);
