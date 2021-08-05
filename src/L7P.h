#pragma once

#include <gantry_robot/Status.h>

#include <modbus.h>
#include <queue>

#define AXIS_X  11
#define AXIS_Y  12
#define AXIS_Z  13

#define HOMING_SPEED_X  5000
#define HOMING_SPEED_Y  5000
#define HOMING_SPEED_Z  5000
#define HOMING_OFFSET_X 131072
#define HOMING_OFFSET_Y 5160
#define HOMING_OFFSET_Z 8097
#define HOMING_DONE_BEHAVIOUR_X OnOff::off
#define HOMING_DONE_BEHAVIOUR_Y OnOff::on
#define HOMING_DONE_BEHAVIOUR_Z OnOff::on

#define READ_COIL_ADDR  0x00
#define READ_COIL_SIZE  0x40
#define READ_REG_SIZE   0x10
#define WRITE_COIL_SIZE 0x20

enum class CommandCase {
	NONE, HOME, POSITION, JOG
};

enum class FunctionCase {
	INIT, SET, ACTION, IDLE
};

enum class OnOff {
    on = 1,
    off = 0
};

enum class AxisCommand {
    stop = 0x03, mode = 0x08,
    emg = 0x0A, a_rst = 0x0B,
    sv_on = 0x0C, start = 0x10, pause = 0x11, hstart = 0x13,
    jstart = 0x1B, jdir = 0x1C
};

enum class CommandType {
    setCommand, getStatus,
    setParameter, getParameter,
    setHomingParameters
};

struct AxisMsg {
    uint8_t id;
    CommandType type;
    AxisCommand axisCommand;
    OnOff onOff;
    gantry_robot::Status* status;
    int32_t speed;
    int32_t offset;
    OnOff done_behaviour;
};

void setAxisCommandMsg(std::queue<AxisMsg>* que, uint8_t id, AxisCommand axisCommand, OnOff onOff);
int32_t setAxisCommand(modbus_t* ctx, int32_t id, AxisCommand axisCommand, OnOff onOff);
int32_t getAxisStatus(modbus_t* ctx, int32_t id, gantry_robot::Status* status);

int32_t setHomingParameters(modbus_t* ctx, int32_t id, int32_t speed, int32_t offset, OnOff done_behaviour);