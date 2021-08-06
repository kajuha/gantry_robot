#include <modbus.h>
#include <string.h>
#include <stdio.h>
#include <queue>

#include <gantry_robot/Status.h>

#include "L7P.h"
#include "ObjectDictionary.h"
#include "main.h"

void setAxisCommandMsg(std::queue<AxisMsg>* que, int32_t id, AxisCommand axisCommand, OnOff onOff) {
    static AxisMsg axisMsg;

	axisMsg.type = CommandType::setCommand;

	axisMsg.id = id;
	axisMsg.axisCommand = axisCommand;
	axisMsg.onOff = onOff;

	que->push(axisMsg);
}

int32_t setAxisCommand(modbus_t* ctx, int32_t id, AxisCommand axisCommand, OnOff onOff) {
    static int32_t ret = 0;

    if (modbus_set_slave(ctx, id) == -1) {
        reprintf(ScreenOutput::ERROR, "[%s{%s}(%d)] : modbus_set_slave error, slave: #%d, axisCommand: 0x%02x, onOff: 0x%02x, error msg: %s \n",
            __FILENAME__, __FUNCTION__, __LINE__, id, (int32_t)axisCommand, (int32_t)onOff, modbus_strerror(errno));

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
        reprintf(ScreenOutput::ERROR, "[%s{%s}(%d)] : modbus_set_slave error, slave: #%d, error msg: %s \n",
            __FILENAME__, __FUNCTION__, __LINE__, id, modbus_strerror(errno));

        return -1;
    } else {
        ret = modbus_read_bits(ctx, READ_COIL_ADDR, READ_COIL_SIZE, read_bits);
        memcpy((uint8_t*)status, read_bits, sizeof(read_bits));
    }

    return ret;
}

void setHomingParametersMsg(std::queue<AxisMsg>* que, int32_t id, int32_t speed, int32_t offset, OnOff done_behaviour) {
    static AxisMsg axisMsg;

	axisMsg.type = CommandType::setHomingParameters;

	axisMsg.id = id;
	axisMsg.speed = speed;
	axisMsg.offset = offset;
	axisMsg.done_behaviour = done_behaviour;

	que->push(axisMsg);
}

int32_t setHomingParameters(modbus_t* ctx, int32_t id, int32_t speed, int32_t offset, OnOff done_behaviour) {
    if (speed < HOMING_MIN_SPEED_VAL) {
        speed = HOMING_MIN_SPEED_VAL;
    }

    if (!setAxisParameter(ctx, id, HOMING_METHOD_ADDR, HOMING_METHOD_VAL, OnOff::on)) {
        reprintf(ScreenOutput::ERROR, "[%s{%s}(%d)] : HOMING_METHOD error\n", __FILENAME__, __FUNCTION__, __LINE__);

        return -1;
    }

    if (!setAxisParameter(ctx, id, HOMING_SWITCH_SPEED_ADDR, speed, OnOff::on)) {
        reprintf(ScreenOutput::ERROR, "[%s{%s}(%d)] : HOMING_SWITCH_SPEED error\n", __FILENAME__, __FUNCTION__, __LINE__);

        return -1;
    }

    if (!setAxisParameter(ctx, id, HOMING_ZERO_SPEED_ADDR, HOMING_MIN_SPEED_VAL, OnOff::on)) {
        reprintf(ScreenOutput::ERROR, "[%s{%s}(%d)] : HOMING_ZERO_SPEED error\n", __FILENAME__, __FUNCTION__, __LINE__);

        return -1;
    }

    if (!setAxisParameter(ctx, id, HOMING_ACCELERATION_ADDR, HOMING_ACCELERATION_VAL, OnOff::on)) {
        reprintf(ScreenOutput::ERROR, "[%s{%s}(%d)] : HOMING_ACCELERATION error\n", __FILENAME__, __FUNCTION__, __LINE__);

        return -1;
    }

    if (!setAxisParameter(ctx, id, HOMING_OFFSET_ADDR, offset, OnOff::on)) {
        reprintf(ScreenOutput::ERROR, "[%s{%s}(%d)] : HOMING_OFFSET error\n", __FILENAME__, __FUNCTION__, __LINE__);

        return -1;
    }

    if (!setAxisParameter(ctx, id, Q_STOP_DECELERATION_ADDR, Q_STOP_DECELERATION_VAL, OnOff::on)) {
        reprintf(ScreenOutput::ERROR, "[%s{%s}(%d)] : Q_STOP_DECELERATION error\n", __FILENAME__, __FUNCTION__, __LINE__);

        return -1;
    }

    if (!setAxisParameter(ctx, id, HOMING_DONE_BEHAVIOUR_ADDR, (uint16_t)done_behaviour, OnOff::on)) {
        reprintf(ScreenOutput::ERROR, "[%s{%s}(%d)] : HOMING_DONE_BEHAVIOUR error\n", __FILENAME__, __FUNCTION__, __LINE__);

        return -1;
    }

    // setHomingParameters is success : 1, fail : -1
    return 1;
}

void setPosParametersMsg(std::queue<AxisMsg>* que, int32_t id, int32_t speed, int32_t offset, OnOff done_behaviour) {
    static AxisMsg axisMsg;

	axisMsg.type = CommandType::setPosParameters;

	axisMsg.id = id;
	axisMsg.speed = speed;
	axisMsg.offset = offset;
	axisMsg.done_behaviour = done_behaviour;

	que->push(axisMsg);
}

int32_t setPosParameters(modbus_t* ctx, int32_t id, int32_t speed, int32_t offset, OnOff done_behaviour) {
#define POS_CTRL_MODE_ADDR          0x3000
#define POS_CTRL_MODE_VAL           0
#define POS_START_INDEX_NUMBER_ADDR 0x3009
#define POS_START_INDEX_NUMBER_VAL  0
#define POS_INDEX_TYPE_ADDR         0x3101
#define POS_INDEX_TYPE_VAL          0
#define POS_REG_DISTANCE_ADDR       0x310A
#define POS_REG_DISTANCE_VAL        0
#define POS_REG_VELOCITY_ADDR       0x310C
#define POS_REG_VELOCITY_VAL        1
#define POS_REPEAT_COUNT_ADDR       0x310E
#define POS_REPEAT_COUNT_VAL        1
#define POS_DWELLTIME_ADDR          0x310F
#define POS_DWELLTIME_VAL           0
#define POS_NEXT_INDEX_ADDR         0x3110
#define POS_NEXT_INDEX_VAL          0
#define POS_ACTION_ADDR             0x3111
#define POS_ACTION_VAL              0
#define POS_DISTANCE_ADDR           0x3102
#define POS_DISTANCE_VAL            0
#define POS_VELOCITY_ADDR           0x3104
#define POS_VELOCITY_VAL            10000
#define POS_ACCELERATION_ADDR       0x3106
#define POS_ACCELERATION_VAL        100000
#define POS_DECELERATION_ADDR       0x3108
#define POS_DECELERATION_VAL        100000
    if (!setAxisParameter(ctx, id, HOMING_METHOD_ADDR, HOMING_METHOD_VAL, OnOff::on)) {
        reprintf(ScreenOutput::ERROR, "[%s{%s}(%d)] : HOMING_METHOD error\n", __FILENAME__, __FUNCTION__, __LINE__);

        return -1;
    }

    // setPosParameters is success : 1, fail : -1
    return 1;
}