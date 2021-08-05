#include <modbus.h>
#include <string.h>
#include <stdio.h>
#include <queue>

#include <gantry_robot/Status.h>

#include "L7P.h"
#include "ObjectDictionary.h"

void setAxisCommandMsg(std::queue<AxisMsg>* que, uint8_t id, AxisCommand axisCommand, OnOff onOff) {
    static AxisMsg axisMsg;

	axisMsg.type = CommandType::setCommand;

	axisMsg.id = id;
	axisMsg.axisCommand = AxisCommand::emg;
	axisMsg.onOff = OnOff::off;

	que->push(axisMsg);
}

int32_t setAxisCommand(modbus_t* ctx, int32_t id, AxisCommand axisCommand, OnOff onOff) {
    static int32_t ret = 0;

    if (modbus_set_slave(ctx, id) == -1) {
        printf("Unable to set the slave ID in context: %s \n", modbus_strerror(errno));

        return -1;
    } else {
        ret = modbus_write_bit(ctx, (int)axisCommand, (int)onOff);
    }

    return ret;
}

int32_t getAxisStatus(modbus_t* ctx, int32_t id, gantry_robot::Status* status) {
    static uint8_t read_bits[READ_COIL_SIZE] = {0, };
    static int32_t ret = 0;

    if (modbus_set_slave(ctx, id) == -1) {
        printf("Unable to set the slave ID in context: %s \n", modbus_strerror(errno));

        return -1;
    } else {
        ret = modbus_read_bits(ctx, READ_COIL_ADDR, READ_COIL_SIZE, read_bits);
        memcpy((uint8_t*)status, read_bits, sizeof(read_bits));
    }

    return ret;
}

int32_t setHomingParameters(modbus_t* ctx, int32_t id, int32_t speed, int32_t offset, OnOff done_behaviour) {
#define HOMING_MINIMUM_SPEED   5000
    if (speed < HOMING_MINIMUM_SPEED) {
        speed = HOMING_MINIMUM_SPEED;
    }

#define HOMING_METHOD   0x603E
#define HOMING_METHOD_VALUE 24
    if (!setAxisParameter(ctx, id, HOMING_METHOD, HOMING_METHOD_VALUE, OnOff::on)) printf("setAxisParameter HOMING_METHOD error\n");

#define HOMING_SWITCH_SPEED 0x6041
    if (!setAxisParameter(ctx, id, HOMING_SWITCH_SPEED, speed, OnOff::on)) printf("setAxisParameter HOMING_SWITCH_SPEED error\n");

#define HOMING_ZERO_SPEED   0x6043
    if (!setAxisParameter(ctx, id, HOMING_ZERO_SPEED, HOMING_MINIMUM_SPEED, OnOff::on)) printf("setAxisParameter HOMING_ZERO_SPEED error\n");

#define HOMING_ACCELERATION 0x6045
#define HOMING_ACCELERATION_VALUE   10000 
    if (!setAxisParameter(ctx, id, HOMING_ACCELERATION, HOMING_ACCELERATION_VALUE, OnOff::on)) printf("setAxisParameter HOMING_ACCELERATION error\n");

#define HOMING_OFFSET   0x6024
    if (!setAxisParameter(ctx, id, HOMING_OFFSET, offset, OnOff::on)) printf("setAxisParameter HOMING_OFFSET error\n");

#define QUICKSTOP_DECELERATION  0x6034
#define QUICKSTOP_DECELERATION_VALUE    10000
    if (!setAxisParameter(ctx, id, QUICKSTOP_DECELERATION, QUICKSTOP_DECELERATION_VALUE, OnOff::on)) printf("setAxisParameter QUICKSTOP_DECELERATION error\n");

#define HOMING_DONE_BEHAVIOUR   0x201F
    if (!setAxisParameter(ctx, id, HOMING_DONE_BEHAVIOUR, (uint16_t)done_behaviour, OnOff::on)) printf("setAxisParameter HOMING_DONE_BEHAVIOUR error\n");

    // setHomingParameters is success : 1, fail : -1
    return 1;
}