#pragma once

#include <gantry_robot/Status.h>

#include <modbus.h>
#include <queue>

#define AXIS_X  11
#define AXIS_Y  12
#define AXIS_Z  13

// GENERAL INFO
#define ENCODER_PPR_AXIS_X          524288
#define ENCODER_PPR_AXIS_Y          524288
#define ENCODER_PPR_AXIS_Z          262144
#define STAGE_MAX_AXIS_X            500
#define STAGE_MAX_AXIS_Y            500
#define STAGE_MAX_AXIS_Z            500
#define STAGE_LEAD_AXIS_X           24
#define STAGE_LEAD_AXIS_Y           25
#define STAGE_LEAD_AXIS_Z           20
#define RATIO_GEAR_AXIS_X           64      // 이미 모터드라이버에 적용되어 있음
#define RATIO_GEAR_AXIS_Y           64      // 이미 모터드라이버에 적용되어 있음
#define RATIO_GEAR_AXIS_Z           64      // 이미 모터드라이버에 적용되어 있음
#define RATIO_SHAFT_AXIS_X          64      // 이미 모터드라이버에 적용되어 있음
#define RATIO_SHAFT_AXIS_Y          64      // 이미 모터드라이버에 적용되어 있음
#define RATIO_SHAFT_AXIS_Z          64      // 이미 모터드라이버에 적용되어 있음


// GENERAL DEFAULT
#define Q_STOP_DECELERATION_ADDR    0x6034
#define Q_STOP_DECELERATION_VAL     10000

// HOMING DEFAULT
#define HOMING_METHOD_ADDR          0x603E
#define HOMING_METHOD_VAL           24
#define HOMING_SWITCH_SPEED_ADDR    0x6041
#define HOMING_ZERO_SPEED_ADDR      0x6043
#define HOMING_MIN_SPEED_VAL        5000
#define HOMING_SPEED_X_VAL          5000
#define HOMING_SPEED_Y_VAL          5000
#define HOMING_SPEED_Z_VAL          5000
#define HOMING_ACCELERATION_ADDR    0x6045
#define HOMING_ACCELERATION_VAL     10000 
#define HOMING_OFFSET_ADDR          0x6024
#define HOMING_OFFSET_X_VAL         131072
#define HOMING_OFFSET_Y_VAL         5160
#define HOMING_OFFSET_Z_VAL         8097
#define HOMING_DONE_BEHAVIOUR_ADDR  0x201F
#define HOMING_DONE_BEHAVIOUR_X_VAL OnOff::off
#define HOMING_DONE_BEHAVIOUR_Y_VAL OnOff::on
#define HOMING_DONE_BEHAVIOUR_Z_VAL OnOff::on

// POSITION DEFAULT
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

#define READ_COIL_ADDR  0x00
#define READ_COIL_SIZE  0x40
#define READ_REG_SIZE   0x10
#define WRITE_COIL_SIZE 0x20

enum class CommandCase {
	IDLE, HOME, POSITION, JOG, ERROR
};

enum class FunctionCase {
	INIT, SET, ACTION, IDLE, ERROR
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
    setHomingParameters,
    setPosParameters, setPosition
};

struct AxisMsg {
    int32_t id;
    CommandType type;
    AxisCommand axisCommand;
    OnOff onOff;
    gantry_robot::Status* status;
    int32_t position;
    int32_t speed;
    int32_t acc;
    int32_t dec;
    int32_t offset;
    OnOff done_behaviour;
};

void setAxisCommandMsg(std::queue<AxisMsg>* que, int32_t id, AxisCommand axisCommand, OnOff onOff);
int32_t setAxisCommand(modbus_t* ctx, int32_t id, AxisCommand axisCommand, OnOff onOff);
int32_t getAxisStatus(modbus_t* ctx, int32_t id, gantry_robot::Status* status);

void setHomingParametersMsg(std::queue<AxisMsg>* que, int32_t id, int32_t speed, int32_t offset, OnOff done_behaviour);
int32_t setHomingParameters(modbus_t* ctx, int32_t id, int32_t speed, int32_t offset, OnOff done_behaviour);

void setPosParametersMsg(std::queue<AxisMsg>* que, int32_t id);
int32_t setPosParameters(modbus_t* ctx, int32_t id);
void setPositionMsg(std::queue<AxisMsg>* que, int32_t id, int32_t position, int32_t speed, int32_t acc, int32_t dec);
int32_t setPosition(modbus_t* ctx, int32_t id, int32_t position, int32_t speed, int32_t acc, int32_t dec);