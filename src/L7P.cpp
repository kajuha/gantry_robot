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

    if (!setAxisParameter(ctx, id, HOMING_DONE_BEHAVIOUR_ADDR, (int32_t)done_behaviour, OnOff::on)) {
        reprintf(ScreenOutput::ERROR, "[%s{%s}(%d)] : HOMING_DONE_BEHAVIOUR error\n", __FILENAME__, __FUNCTION__, __LINE__);

        return -1;
    }

    // setHomingParameters is success : 1, fail : -1
    return 1;
}

void setJogParametersMsg(std::queue<AxisMsg>* que, int32_t id, int32_t speed, int32_t acc, int32_t dec, int32_t s_curve, OnOff servo_lock) {
    static AxisMsg axisMsg;

	axisMsg.type = CommandType::setJogParameters;

	axisMsg.id = id;
	axisMsg.speed = speed;
	axisMsg.acc = acc;
	axisMsg.dec = dec;
	axisMsg.s_curve = s_curve;
	axisMsg.servo_lock = servo_lock;

	que->push(axisMsg);
}

int32_t setJogParameters(modbus_t* ctx, int32_t id, int32_t speed, int32_t acc, int32_t dec, int32_t s_curve, OnOff servo_lock) {
    if (speed < JOG_MIN_SPEED_VAL) {
        speed = JOG_MIN_SPEED_VAL;
    }
    
    if (!setAxisParameter(ctx, id, JOG_SPEED_ADDR, speed, OnOff::on)) {
        reprintf(ScreenOutput::ERROR, "[%s{%s}(%d)] : JOG_SPEED_ADDR error\n", __FILENAME__, __FUNCTION__, __LINE__);

        return -1;
    }

    if (!setAxisParameter(ctx, id, JOG_ACCELERATION_ADDR, acc, OnOff::on)) {
        reprintf(ScreenOutput::ERROR, "[%s{%s}(%d)] : JOG_ACCELERATION_ADDR error\n", __FILENAME__, __FUNCTION__, __LINE__);

        return -1;
    }

    if (!setAxisParameter(ctx, id, JOG_DECELERATION_ADDR, dec, OnOff::on)) {
        reprintf(ScreenOutput::ERROR, "[%s{%s}(%d)] : JOG_DECELERATION_ADDR error\n", __FILENAME__, __FUNCTION__, __LINE__);

        return -1;
    }

    if (!setAxisParameter(ctx, id, JOG_S_CURVE_ADDR, s_curve, OnOff::on)) {
        reprintf(ScreenOutput::ERROR, "[%s{%s}(%d)] : JOG_S_CURVE_ADDR error\n", __FILENAME__, __FUNCTION__, __LINE__);

        return -1;
    }

    if (!setAxisParameter(ctx, id, JOG_SERVO_LOCK_ADDR, (int32_t)servo_lock, OnOff::on)) {
        reprintf(ScreenOutput::ERROR, "[%s{%s}(%d)] : JOG_SERVO_LOCK_ADDR error\n", __FILENAME__, __FUNCTION__, __LINE__);

        return -1;
    }

    // setJogParameters is success : 1, fail : -1
    return 1;
}

void setPosParametersMsg(std::queue<AxisMsg>* que, int32_t id) {
    static AxisMsg axisMsg;

	axisMsg.type = CommandType::setPosParameters;

	axisMsg.id = id;

	que->push(axisMsg);
}

int32_t setPosParameters(modbus_t* ctx, int32_t id) {
    if (!setAxisParameter(ctx, id, POS_CTRL_MODE_ADDR, POS_CTRL_MODE_VAL, OnOff::on)) {
        reprintf(ScreenOutput::ERROR, "[%s{%s}(%d)] : POS_CTRL_MODE_ADDR error\n", __FILENAME__, __FUNCTION__, __LINE__);

        return -1;
    }

    if (!setAxisParameter(ctx, id, POS_START_INDEX_NUMBER_ADDR, POS_START_INDEX_NUMBER_VAL, OnOff::on)) {
        reprintf(ScreenOutput::ERROR, "[%s{%s}(%d)] : POS_START_INDEX_NUMBER_ADDR error\n", __FILENAME__, __FUNCTION__, __LINE__);

        return -1;
    }

    if (!setAxisParameter(ctx, id, POS_INDEX_TYPE_ADDR, POS_INDEX_TYPE_VAL, OnOff::on)) {
        reprintf(ScreenOutput::ERROR, "[%s{%s}(%d)] : POS_INDEX_TYPE_ADDR error\n", __FILENAME__, __FUNCTION__, __LINE__);

        return -1;
    }

    if (!setAxisParameter(ctx, id, POS_REG_DISTANCE_ADDR, POS_REG_DISTANCE_VAL, OnOff::on)) {
        reprintf(ScreenOutput::ERROR, "[%s{%s}(%d)] : POS_REG_DISTANCE_ADDR error\n", __FILENAME__, __FUNCTION__, __LINE__);

        return -1;
    }

    if (!setAxisParameter(ctx, id, POS_REG_VELOCITY_ADDR, POS_REG_VELOCITY_VAL, OnOff::on)) {
        reprintf(ScreenOutput::ERROR, "[%s{%s}(%d)] : POS_REG_VELOCITY_ADDR error\n", __FILENAME__, __FUNCTION__, __LINE__);

        return -1;
    }

    if (!setAxisParameter(ctx, id, POS_REPEAT_COUNT_ADDR, POS_REPEAT_COUNT_VAL, OnOff::on)) {
        reprintf(ScreenOutput::ERROR, "[%s{%s}(%d)] : POS_REPEAT_COUNT_ADDR error\n", __FILENAME__, __FUNCTION__, __LINE__);

        return -1;
    }

    if (!setAxisParameter(ctx, id, POS_DWELLTIME_ADDR, POS_DWELLTIME_VAL, OnOff::on)) {
        reprintf(ScreenOutput::ERROR, "[%s{%s}(%d)] : POS_DWELLTIME_ADDR error\n", __FILENAME__, __FUNCTION__, __LINE__);

        return -1;
    }

    if (!setAxisParameter(ctx, id, POS_NEXT_INDEX_ADDR, POS_NEXT_INDEX_VAL, OnOff::on)) {
        reprintf(ScreenOutput::ERROR, "[%s{%s}(%d)] : POS_NEXT_INDEX_ADDR error\n", __FILENAME__, __FUNCTION__, __LINE__);

        return -1;
    }

    if (!setAxisParameter(ctx, id, POS_ACTION_ADDR, POS_ACTION_VAL, OnOff::on)) {
        reprintf(ScreenOutput::ERROR, "[%s{%s}(%d)] : POS_ACTION_ADDR error\n", __FILENAME__, __FUNCTION__, __LINE__);

        return -1;
    }

    if (!setAxisParameter(ctx, id, POS_DISTANCE_ADDR, POS_DISTANCE_VAL, OnOff::on)) {
        reprintf(ScreenOutput::ERROR, "[%s{%s}(%d)] : POS_DISTANCE_ADDR error\n", __FILENAME__, __FUNCTION__, __LINE__);

        return -1;
    }

    if (!setAxisParameter(ctx, id, POS_VELOCITY_ADDR, POS_VELOCITY_VAL, OnOff::on)) {
        reprintf(ScreenOutput::ERROR, "[%s{%s}(%d)] : POS_VELOCITY_ADDR error\n", __FILENAME__, __FUNCTION__, __LINE__);

        return -1;
    }

    if (!setAxisParameter(ctx, id, POS_ACCELERATION_ADDR, POS_ACCELERATION_VAL, OnOff::on)) {
        reprintf(ScreenOutput::ERROR, "[%s{%s}(%d)] : POS_ACCELERATION_ADDR error\n", __FILENAME__, __FUNCTION__, __LINE__);

        return -1;
    }

    if (!setAxisParameter(ctx, id, POS_DECELERATION_ADDR, POS_DECELERATION_VAL, OnOff::on)) {
        reprintf(ScreenOutput::ERROR, "[%s{%s}(%d)] : POS_DECELERATION_ADDR error\n", __FILENAME__, __FUNCTION__, __LINE__);

        return -1;
    }

    // setPosParameters is success : 1, fail : -1
    return 1;
}

void setPositionMsg(std::queue<AxisMsg>* que, int32_t id, int32_t position, int32_t speed, int32_t acc, int32_t dec) {
    static AxisMsg axisMsg;

	axisMsg.type = CommandType::setPosition;

	axisMsg.id = id;
    axisMsg.position = position;
    axisMsg.speed = speed;
    axisMsg.acc = acc;
    axisMsg.dec = dec;

	que->push(axisMsg);
}

int32_t setPosition(modbus_t* ctx, int32_t id, int32_t position, int32_t speed, int32_t acc, int32_t dec) {
    if (!setAxisParameter(ctx, id, POS_DISTANCE_ADDR, position, OnOff::on)) {
        reprintf(ScreenOutput::ERROR, "[%s{%s}(%d)] : POS_DISTANCE_ADDR, error\n", __FILENAME__, __FUNCTION__, __LINE__);

        return -1;
    }

    if (!setAxisParameter(ctx, id, POS_VELOCITY_ADDR, speed, OnOff::on)) {
        reprintf(ScreenOutput::ERROR, "[%s{%s}(%d)] : POS_VELOCITY_ADDR, error\n", __FILENAME__, __FUNCTION__, __LINE__);

        return -1;
    }

    if (!setAxisParameter(ctx, id, POS_ACCELERATION_ADDR, acc, OnOff::on)) {
        reprintf(ScreenOutput::ERROR, "[%s{%s}(%d)] : POS_ACCELERATION_ADDR, error\n", __FILENAME__, __FUNCTION__, __LINE__);

        return -1;
    }

    if (!setAxisParameter(ctx, id, POS_DECELERATION_ADDR, dec, OnOff::on)) {
        reprintf(ScreenOutput::ERROR, "[%s{%s}(%d)] : POS_DECELERATION_ADDR, error\n", __FILENAME__, __FUNCTION__, __LINE__);

        return -1;
    }

    // setPosition is success : 1, fail : -1
    return 1;
}