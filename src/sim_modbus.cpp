#include <gantry_robot/Info.h>

#include "sim_modbus.h"
#include "L7P.h"
#include "ObjectDictionary.h"

#define MODBUS_SUCCESS  1
#define MODBUS_FAIL     0
#define MODBUS_ERROR    -1

modbus_t g_modbus;
int g_slave;
int32_t g_Q_STOP_DECELERATION_VAL;
int32_t g_HOMING_METHOD_VAL;
int32_t g_HOMING_SWITCH_SPEED_VAL;
int32_t g_HOMING_ZERO_SPEED_VAL;
int32_t g_HOMING_ACCELERATION_VAL;
int32_t g_HOMING_OFFSET_VAL;
int32_t g_HOMING_DONE_BEHAVIOUR_VAL;
int32_t g_JOG_SPEED_VAL;
int32_t g_JOG_ACCELERATION_VAL;
int32_t g_JOG_DECELERATION_VAL;
int32_t g_JOG_S_CURVE_VAL;
int32_t g_JOG_SERVO_LOCK_VAL;
int32_t g_POS_CTRL_MODE_VAL;
int32_t g_POS_START_INDEX_NUMBER_VAL;
int32_t g_POS_INDEX_TYPE_VAL;
int32_t g_POS_REG_DISTANCE_VAL;
int32_t g_POS_REG_VELOCITY_VAL;
int32_t g_POS_REPEAT_COUNT_VAL;
int32_t g_POS_DWELLTIME_VAL;
int32_t g_POS_NEXT_INDEX_VAL;
int32_t g_POS_ACTION_VAL;
int32_t g_POS_DISTANCE_VAL;
int32_t g_POS_VELOCITY_VAL;
int32_t g_POS_ACCELERATION_VAL;
int32_t g_POS_DECELERATION_VAL;

modbus_t* modbus_new_rtu(const char *device, int baud, char parity, int data_bit, int stop_bit) {
    printf("simulation modbus_new_rtu success\n");
    
    return &g_modbus;
}

int modbus_connect(modbus_t *ctx) {
    printf("simulation modbus_connect success\n");
    
    return 0;
}
void modbus_close(modbus_t *ctx) {
    printf("simulation modbus_close success\n");
    
    return;
}

void modbus_free(modbus_t *ctx) {
    printf("simulation modbus_free success\n");
    
    return;
}

void modbus_set_debug(modbus_t *ctx, int boolean) {
    printf("simulation modbus_set_debug success\n");
    
    return;
}

const char *modbus_strerror(int errnum) {
    printf("simulation modbus_strerror occured\n");
    while(ros::ok());
    
    return "\0";
}

int modbus_set_slave(modbus_t* ctx, int slave) {
    g_slave = slave;
    // printf("simulation modbus_set_slave success\n");
    
    return 0;
}

int modbus_read_bits(modbus_t *ctx, int addr, int nb, uint8_t *dest) {
    // printf("modbus_read_bits [id:%d] [addr:0x%04x] [nb:%d]\n", g_slave, addr, nb);
    static gantry_robot::InfoAxis infoAxis;
    infoAxis.status.output.alarm = 0;
    infoAxis.status.output.ready = 1;
    infoAxis.status.output.inpos1 = 1;
    infoAxis.status.output.inspd = 1;
    infoAxis.status.output.org = 1;
    infoAxis.status.output.eos = 1;
    memcpy(dest, (uint8_t*)&(infoAxis.status), nb);
    
    return MODBUS_SUCCESS;
}
int modbus_read_registers(modbus_t *ctx, int addr, int nb, uint16_t *dest) {
    // printf("modbus_read_registers [id:%d] [addr:0x%04x] [nb:%d]\n", g_slave, addr, nb);
    switch (addr) {
        case ACT_POS:
            *((int32_t*)dest) = g_slave;
        break;
        case ACT_SPD:
            *((int32_t*)dest) = g_slave + 10;
        break;
        case Q_STOP_DECELERATION_ADDR:
            *((int32_t*)dest) = g_Q_STOP_DECELERATION_VAL;
        break;
        case HOMING_METHOD_ADDR:
            *((int32_t*)dest) = g_HOMING_METHOD_VAL;
        break;
        case HOMING_SWITCH_SPEED_ADDR:
            *((int32_t*)dest) = g_HOMING_SWITCH_SPEED_VAL;
        break;
        case HOMING_ZERO_SPEED_ADDR:
            *((int32_t*)dest) = g_HOMING_ZERO_SPEED_VAL;
        break;
        case HOMING_ACCELERATION_ADDR:
            *((int32_t*)dest) = g_HOMING_ACCELERATION_VAL;
        break;
        case HOMING_OFFSET_ADDR:
            *((int32_t*)dest) = g_HOMING_OFFSET_VAL;
        break;
        case HOMING_DONE_BEHAVIOUR_ADDR:
            *((int32_t*)dest) = g_HOMING_DONE_BEHAVIOUR_VAL;
        break;
        case JOG_SPEED_ADDR:
            *((int32_t*)dest) = g_JOG_SPEED_VAL;
        break;
        case JOG_ACCELERATION_ADDR:
            *((int32_t*)dest) = g_JOG_ACCELERATION_VAL;
        break;
        case JOG_DECELERATION_ADDR:
            *((int32_t*)dest) = g_JOG_DECELERATION_VAL;
        break;
        case JOG_S_CURVE_ADDR:
            *((int32_t*)dest) = g_JOG_S_CURVE_VAL;
        break;
        case JOG_SERVO_LOCK_ADDR:
            *((int32_t*)dest) = g_JOG_SERVO_LOCK_VAL;
        break;
        case POS_CTRL_MODE_ADDR:
            *((int32_t*)dest) = g_POS_CTRL_MODE_VAL;
        break;
        case POS_START_INDEX_NUMBER_ADDR:
            *((int32_t*)dest) = g_POS_START_INDEX_NUMBER_VAL;
        break;
        case POS_INDEX_TYPE_ADDR:
            *((int32_t*)dest) = g_POS_INDEX_TYPE_VAL;
        break;
        case POS_REG_DISTANCE_ADDR:
            *((int32_t*)dest) = g_POS_REG_DISTANCE_VAL;
        break;
        case POS_REG_VELOCITY_ADDR:
            *((int32_t*)dest) = g_POS_REG_VELOCITY_VAL;
        break;
        case POS_REPEAT_COUNT_ADDR:
            *((int32_t*)dest) = g_POS_REPEAT_COUNT_VAL;
        break;
        case POS_DWELLTIME_ADDR:
            *((int32_t*)dest) = g_POS_DWELLTIME_VAL;
        break;
        case POS_NEXT_INDEX_ADDR:
            *((int32_t*)dest) = g_POS_NEXT_INDEX_VAL;
        break;
        case POS_ACTION_ADDR:
            *((int32_t*)dest) = g_POS_ACTION_VAL;
        break;
        case POS_DISTANCE_ADDR:
            *((int32_t*)dest) = g_POS_DISTANCE_VAL;
        break;
        case POS_VELOCITY_ADDR:
            *((int32_t*)dest) = g_POS_VELOCITY_VAL;
        break;
        case POS_ACCELERATION_ADDR:
            *((int32_t*)dest) = g_POS_ACCELERATION_VAL;
        break;
        case POS_DECELERATION_ADDR:
            *((int32_t*)dest) = g_POS_DECELERATION_VAL;
        break;
        default:
            printf("modbus_read_registers unknown switch-case [addr:0x%x]\n", addr);
            while(ros::ok());
        break;
    }
    
    return getObjType(addr);
}
int modbus_write_bit(modbus_t *ctx, int addr, int status) {
    // printf("modbus_write_bit [id:%d] [addr:0x%04x] [status:%d]\n", g_slave, addr, status);
    // stop = 0x03, mode = 0x08,
    // emg = 0x0A, a_rst = 0x0B,
    // sv_on = 0x0C, start = 0x10, pause = 0x11, hstart = 0x13,
    // jstart = 0x1B, jdir = 0x1C
    switch (addr) {
        case (int)AxisCommand::a_rst:
        break;
        case (int)AxisCommand::emg:
        break;
        case (int)AxisCommand::stop:
        break;
        case (int)AxisCommand::sv_on:
        break;
        default:
            printf("modbus_write_bit unknown switch-case [addr:0x%x]\n", addr);
            while(ros::ok());
        break;
    }
    
    return MODBUS_SUCCESS;
}
int modbus_write_registers(modbus_t *ctx, int addr, int nb, const uint16_t *data) {
    // printf("modbus_write_registers [id:%d] [addr:0x%04x] [nb:%d]\n", g_slave, addr, nb);
    switch (addr) {
        case Q_STOP_DECELERATION_ADDR:
            g_Q_STOP_DECELERATION_VAL = *((int32_t*)data);
        break;
        case HOMING_METHOD_ADDR:
            g_HOMING_METHOD_VAL = *((int32_t*)data);
        break;
        case HOMING_SWITCH_SPEED_ADDR:
            g_HOMING_SWITCH_SPEED_VAL = *((int32_t*)data);
        break;
        case HOMING_ZERO_SPEED_ADDR:
            g_HOMING_ZERO_SPEED_VAL = *((int32_t*)data);
        break;
        case HOMING_ACCELERATION_ADDR:
            g_HOMING_ACCELERATION_VAL = *((int32_t*)data);
        break;
        case HOMING_OFFSET_ADDR:
            g_HOMING_OFFSET_VAL = *((int32_t*)data);
        break;
        case HOMING_DONE_BEHAVIOUR_ADDR:
            g_HOMING_DONE_BEHAVIOUR_VAL = *((int32_t*)data);
        break;
        case JOG_SPEED_ADDR:
            g_JOG_SPEED_VAL = *((int32_t*)data);
        break;
        case JOG_ACCELERATION_ADDR:
            g_JOG_ACCELERATION_VAL = *((int32_t*)data);
        break;
        case JOG_DECELERATION_ADDR:
            g_JOG_DECELERATION_VAL = *((int32_t*)data);
        break;
        case JOG_S_CURVE_ADDR:
            g_JOG_S_CURVE_VAL = *((int32_t*)data);
        break;
        case JOG_SERVO_LOCK_ADDR:
            g_JOG_SERVO_LOCK_VAL = *((int32_t*)data);
        break;
        case POS_CTRL_MODE_ADDR:
            g_POS_CTRL_MODE_VAL = *((int32_t*)data);
        break;
        case POS_START_INDEX_NUMBER_ADDR:
            g_POS_START_INDEX_NUMBER_VAL = *((int32_t*)data);
        break;
        case POS_INDEX_TYPE_ADDR:
            g_POS_INDEX_TYPE_VAL = *((int32_t*)data);
        break;
        case POS_REG_DISTANCE_ADDR:
            g_POS_REG_DISTANCE_VAL = *((int32_t*)data);
        break;
        case POS_REG_VELOCITY_ADDR:
            g_POS_REG_VELOCITY_VAL = *((int32_t*)data);
        break;
        case POS_REPEAT_COUNT_ADDR:
            g_POS_REPEAT_COUNT_VAL = *((int32_t*)data);
        break;
        case POS_DWELLTIME_ADDR:
            g_POS_DWELLTIME_VAL = *((int32_t*)data);
        break;
        case POS_NEXT_INDEX_ADDR:
            g_POS_NEXT_INDEX_VAL = *((int32_t*)data);
        break;
        case POS_ACTION_ADDR:
            g_POS_ACTION_VAL = *((int32_t*)data);
        break;
        case POS_DISTANCE_ADDR:
            g_POS_DISTANCE_VAL = *((int32_t*)data);
        break;
        case POS_VELOCITY_ADDR:
            g_POS_VELOCITY_VAL = *((int32_t*)data);
        break;
        case POS_ACCELERATION_ADDR:
            g_POS_ACCELERATION_VAL = *((int32_t*)data);
        break;
        case POS_DECELERATION_ADDR:
            g_POS_DECELERATION_VAL = *((int32_t*)data);
        break;
        default:
            printf("modbus_write_registers unknown switch-case [addr:0x%x]\n", addr);
            while(ros::ok());
        break;
    }
    
    return getObjType(addr);
}
