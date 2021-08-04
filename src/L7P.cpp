#include <modbus.h>
#include <string.h>
#include <stdio.h>

#include <gantry_robot/Status.h>

#include "L7P.h"
#include "ObjectDictionary.h"

int32_t setAxisCommand(modbus_t* ctx, AxisCommand axisCommand, OnOff onOff) {
    static int32_t ret = 0;

    ret =  modbus_write_bit(ctx, (int)axisCommand, (int)onOff);

    return ret;
}

int32_t getAxisStatus(modbus_t* ctx, gantry_robot::Status* status) {
    static uint8_t read_bits[READ_COIL_SIZE] = {0, };
    static int32_t ret = 0;

    ret = modbus_read_bits(ctx, READ_COIL_ADDR, READ_COIL_SIZE, read_bits);
    memcpy((uint8_t*)status, read_bits, sizeof(read_bits));

    #if 0
    printf("read : \n");
    for (int32_t i=0; i<READ_COIL_SIZE; i++) {
        if(strcmp(bit_read_subject[i], "RESERVED")) {
            printf("[%s, 0x%02x] : %x\n", bit_read_subject[i], i, read_bits[i]);
        }
    }
    #endif

    return ret;
}

int32_t setHomingParameters(modbus_t* ctx, int32_t speed, int32_t offset, OnOff done_behaviour) {
#define HOMING_MINIMUM_SPEED   5000 
    if (speed < HOMING_MINIMUM_SPEED) {
        speed = HOMING_MINIMUM_SPEED;
    }
    // Homing Parameter Set
#define DATA_SIZE	2
    int32_t addr;
    int32_t nb;
    uint16_t write_data[DATA_SIZE] = {0, };
    uint16_t read_data[DATA_SIZE] = {0, };

#define HOMING_METHOD   0x603E
#define HOMING_METHOD_VALUE 24
    addr = HOMING_METHOD;
    nb = getObjType(HOMING_METHOD);
    // printf("nb: %d, addr: 0x%04x\n", nb, addr);
    write_data[0] = HOMING_METHOD_VALUE;
    // if (modbus_write_and_read_registers(ctx, addr, nb, write_data, addr, nb, read_data)) {
    if (modbus_write_registers(ctx, addr, nb, write_data)) {
        // for(int i=0; i<sizeof(write_data); i++) printf("[%02x] ", write_data[i]); printf("\n");
        // memset(write_data, '\0', sizeof(write_data));
        // for(int i=0; i<sizeof(write_data); i++) printf("[%02x] ", write_data[i]); printf("\n");
        // modbus_read_registers(ctx, addr, nb, write_data);
        // for(int i=0; i<sizeof(write_data); i++) printf("[%02x] ", write_data[i]); printf("\n");
        // printf("0x%04x: write:%10d, read:%10d\n", addr, write_data[0], read_data[0]);
    } else {
        printf("0x%04x: error\n", addr);

        return -1;
    }

#define HOMING_SWITCH_SPEED 0x6041
    addr = HOMING_SWITCH_SPEED;
    nb = getObjType(HOMING_SWITCH_SPEED);
    // printf("nb: %d, addr: 0x%04x\n", nb, addr);
    *((int32_t*)write_data) = speed;
    // if (modbus_write_and_read_registers(ctx, addr, nb, write_data, addr, nb, read_data)) {
    if (modbus_write_registers(ctx, addr, nb, write_data)) {
        // for(int i=0; i<sizeof(write_data); i++) printf("[%02x] ", write_data[i]); printf("\n");
        // memset(write_data, '\0', sizeof(write_data));
        // for(int i=0; i<sizeof(write_data); i++) printf("[%02x] ", write_data[i]); printf("\n");
        // modbus_read_registers(ctx, addr, nb, write_data);
        // for(int i=0; i<sizeof(write_data); i++) printf("[%02x] ", write_data[i]); printf("\n");
        // printf("0x%04x: write:%10d, read:%10d\n", addr, *((int32_t*)write_data), *((int32_t*)read_data));
    } else {
        printf("0x%04x: error\n", addr);

        return -1;
    }

#define HOMING_ZERO_SPEED   0x6043
    addr = HOMING_ZERO_SPEED;
    nb = getObjType(HOMING_ZERO_SPEED);
    // printf("nb: %d, addr: 0x%04x\n", nb, addr);
    *((int32_t*)write_data) = HOMING_MINIMUM_SPEED;
    // if (modbus_write_and_read_registers(ctx, addr, nb, write_data, addr, nb, read_data)) {
    if (modbus_write_registers(ctx, addr, nb, write_data)) {
        // for(int i=0; i<sizeof(write_data); i++) printf("[%02x] ", write_data[i]); printf("\n");
        // memset(write_data, '\0', sizeof(write_data));
        // for(int i=0; i<sizeof(write_data); i++) printf("[%02x] ", write_data[i]); printf("\n");
        // modbus_read_registers(ctx, addr, nb, write_data);
        // for(int i=0; i<sizeof(write_data); i++) printf("[%02x] ", write_data[i]); printf("\n");
        // printf("0x%04x: write:%10d, read:%10d\n", addr, *((int32_t*)write_data), *((int32_t*)read_data));
    } else {
        printf("0x%04x: error\n", addr);

        return -1;
    }

#define HOMING_ACCELERATION 0x6045
#define HOMING_ACCELERATION_VALUE   10000 
    addr = HOMING_ACCELERATION;
    nb = getObjType(HOMING_ACCELERATION);
    // printf("nb: %d, addr: 0x%04x\n", nb, addr);
    *((int32_t*)write_data) = HOMING_ACCELERATION_VALUE;
    // if (modbus_write_and_read_registers(ctx, addr, nb, write_data, addr, nb, read_data)) {
    if (modbus_write_registers(ctx, addr, nb, write_data)) {
        // for(int i=0; i<sizeof(write_data); i++) printf("[%02x] ", write_data[i]); printf("\n");
        // memset(write_data, '\0', sizeof(write_data));
        // for(int i=0; i<sizeof(write_data); i++) printf("[%02x] ", write_data[i]); printf("\n");
        // modbus_read_registers(ctx, addr, nb, write_data);
        // for(int i=0; i<sizeof(write_data); i++) printf("[%02x] ", write_data[i]); printf("\n");
        // printf("0x%04x: write:%10d, read:%10d\n", addr, *((int32_t*)write_data), *((int32_t*)read_data));
    } else {
        printf("0x%04x: error\n", addr);

        return -1;
    }

#define HOMING_OFFSET   0x6024
    addr = HOMING_OFFSET;
    nb = getObjType(HOMING_OFFSET);
    // printf("nb: %d, addr: 0x%04x\n", nb, addr);
    *((int32_t*)write_data) = offset;
    // if (modbus_write_and_read_registers(ctx, addr, nb, write_data, addr, nb, read_data)) {
    if (modbus_write_registers(ctx, addr, nb, write_data)) {
        // for(int i=0; i<sizeof(write_data); i++) printf("[%02x] ", write_data[i]); printf("\n");
        // memset(write_data, '\0', sizeof(write_data));
        // for(int i=0; i<sizeof(write_data); i++) printf("[%02x] ", write_data[i]); printf("\n");
        // modbus_read_registers(ctx, addr, nb, write_data);
        // for(int i=0; i<sizeof(write_data); i++) printf("[%02x] ", write_data[i]); printf("\n");
        // printf("0x%04x: write:%10d, read:%10d\n", addr, *((int32_t*)write_data), *((int32_t*)read_data));
    } else {
        printf("0x%04x: error\n", addr);

        return -1;
    }

#define QUICKSTOP_DECELERATION  0x6034
#define QUICKSTOP_DECELERATION_VALUE    10000
    addr = QUICKSTOP_DECELERATION;
    nb = getObjType(QUICKSTOP_DECELERATION);
    // printf("nb: %d, addr: 0x%04x\n", nb, addr);
    *((int32_t*)write_data) = QUICKSTOP_DECELERATION_VALUE;
    // if (modbus_write_and_read_registers(ctx, addr, nb, write_data, addr, nb, read_data)) {
    if (modbus_write_registers(ctx, addr, nb, write_data)) {
        // for(int i=0; i<sizeof(write_data); i++) printf("[%02x] ", write_data[i]); printf("\n");
        // memset(write_data, '\0', sizeof(write_data));
        // for(int i=0; i<sizeof(write_data); i++) printf("[%02x] ", write_data[i]); printf("\n");
        // modbus_read_registers(ctx, addr, nb, write_data);
        // for(int i=0; i<sizeof(write_data); i++) printf("[%02x] ", write_data[i]); printf("\n");
        // printf("0x%04x: write:%10d, read:%10d\n", addr, *((int32_t*)write_data), *((int32_t*)read_data));
    } else {
        printf("0x%04x: error\n", addr);

        return -1;
    }

#define HOMING_DONE_BEHAVIOUR   0x201F
    addr = HOMING_DONE_BEHAVIOUR;
    nb = getObjType(HOMING_DONE_BEHAVIOUR);
    // printf("nb: %d, addr: 0x%04x\n", nb, addr);
    write_data[0] = (uint16_t)done_behaviour;
    // if (modbus_write_and_read_registers(ctx, addr, nb, write_data, addr, nb, read_data)) {
    if (modbus_write_registers(ctx, addr, nb, write_data)) {
        // for(int i=0; i<sizeof(write_data); i++) printf("[%02x] ", write_data[i]); printf("\n");
        // memset(write_data, '\0', sizeof(write_data));
        // for(int i=0; i<sizeof(write_data); i++) printf("[%02x] ", write_data[i]); printf("\n");
        // modbus_read_registers(ctx, addr, nb, write_data);
        // for(int i=0; i<sizeof(write_data); i++) printf("[%02x] ", write_data[i]); printf("\n");
        // printf("0x%04x: write:%10d, read:%10d\n", addr, write_data[0], read_data[0]);
    } else {
        printf("0x%04x: error\n", addr);

        return -1;
    }

    return 0;
}