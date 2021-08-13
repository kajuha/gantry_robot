#pragma once

#include <gantry_robot/Status.h>

#include <modbus.h>
#include <queue>

#define AXIS_X_NUM  11
#define AXIS_Y_NUM  12
#define AXIS_Z_NUM  13

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
#define HOMING_OFFSET_X_VAL         0
// #define HOMING_OFFSET_X_VAL         131072
#define HOMING_OFFSET_Y_VAL         5160
#define HOMING_OFFSET_Z_VAL         8097
#define HOMING_DONE_BEHAVIOUR_ADDR  0x201F
#define HOMING_DONE_BEHAVIOUR_X_VAL OnOff::off
#define HOMING_DONE_BEHAVIOUR_Y_VAL OnOff::on
#define HOMING_DONE_BEHAVIOUR_Z_VAL OnOff::on

// JOG DEFAULT
#define JOG_MIN_SPEED_VAL           100
#define JOG_SPEED_ADDR              0x2300
#define JOG_SPEED_VAL               100
#define JOG_ACCELERATION_ADDR       0x2301
#define JOG_ACCELERATION_VAL        20
#define JOG_DECELERATION_ADDR       0x2302
#define JOG_DECELERATION_VAL        20
#define JOG_S_CURVE_ADDR            0x2303
#define JOG_S_CURVE_VAL             0
#define JOG_SERVO_LOCK_ADDR         0x2311
#define JOG_SERVO_LOCK_VAL          OnOff::off

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
	IDLE, HOME, JOG, POSITION, INIT, ERROR
};

enum class FunctionCase {
	INIT, SET, ACTION, DONE, IDLE, ERROR
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
    setJogParameters,
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
    int32_t s_curve;
    int32_t acc;
    int32_t dec;
    int32_t offset;
    OnOff done_behaviour;
    OnOff servo_lock;
};

class GlobalInfo {
    std::string node_name;

    std::string serial_port;
    int32_t baud_rate;
    
    int32_t axis_x_num;
    int32_t axis_y_num;
    int32_t axis_z_num;

    int32_t encoder_ppr_axis_x;
    int32_t encoder_ppr_axis_y;
    int32_t encoder_ppr_axis_z;
    int32_t stage_max_axis_x;
    int32_t stage_max_axis_y;
    int32_t stage_max_axis_z;
    int32_t stage_lead_axis_x;
    int32_t stage_lead_axis_y;
    int32_t stage_lead_axis_z;
    int32_t ratio_gear_axis_x;
    int32_t ratio_gear_axis_y;
    int32_t ratio_gear_axis_z;
    int32_t ratio_shaft_axis_x;
    int32_t ratio_shaft_axis_y;
    int32_t ratio_shaft_axis_z;

    int32_t q_stop_deceleration_val;
    
    int32_t homing_method_val;
    int32_t homing_min_speed_val;
    int32_t homing_speed_x_val;
    int32_t homing_speed_y_val;
    int32_t homing_speed_z_val;
    int32_t homing_acceleration_val;
    int32_t homing_offset_x_val;
    int32_t homing_offset_y_val;
    int32_t homing_offset_z_val;
    int32_t homing_done_behaviour_x_val;
    int32_t homing_done_behaviour_y_val;
    int32_t homing_done_behaviour_z_val;

    int32_t jog_min_speed_val;
    int32_t jog_speed_val;
    int32_t jog_acceleration_val;
    int32_t jog_deceleration_val;
    int32_t jog_s_curve_val;
    int32_t jog_servo_lock_val;

    int32_t pos_ctrl_mode_val;
    int32_t pos_start_index_number_val;
    int32_t pos_index_type_val;
    int32_t pos_reg_distance_val;
    int32_t pos_reg_velocity_val;
    int32_t pos_repeat_count_val;
    int32_t pos_dwelltime_val;
    int32_t pos_next_index_val;
    int32_t pos_action_val;
};

void setAxisCommandMsg(std::queue<AxisMsg>* que, int32_t id, AxisCommand axisCommand, OnOff onOff);
int32_t setAxisCommand(modbus_t* ctx, int32_t id, AxisCommand axisCommand, OnOff onOff);
int32_t getAxisStatus(modbus_t* ctx, int32_t id, gantry_robot::Status* status);

void setHomingParametersMsg(std::queue<AxisMsg>* que, int32_t id, int32_t speed, int32_t offset, OnOff done_behaviour);
int32_t setHomingParameters(modbus_t* ctx, int32_t id, int32_t speed, int32_t offset, OnOff done_behaviour);

void setJogParametersMsg(std::queue<AxisMsg>* que, int32_t id, int32_t speed, int32_t acc, int32_t dec, int32_t s_curve, OnOff servo_lock);
int32_t setJogParameters(modbus_t* ctx, int32_t id, int32_t speed, int32_t acc, int32_t dec, int32_t s_curve, OnOff servo_lock);

void setPosParametersMsg(std::queue<AxisMsg>* que, int32_t id);
int32_t setPosParameters(modbus_t* ctx, int32_t id);
void setPositionMsg(std::queue<AxisMsg>* que, int32_t id, int32_t position, int32_t speed, int32_t acc, int32_t dec);
int32_t setPosition(modbus_t* ctx, int32_t id, int32_t position, int32_t speed, int32_t acc, int32_t dec);