#pragma once

#include <gantry_robot/Status.h>
#include <gantry_robot/State.h>

#include <modbus.h>
#include <queue>

#define Q_STOP_DECELERATION_ADDR    0x6034

#define HOMING_METHOD_ADDR          0x603E
#define HOMING_METHOD_VAL           24
#define HOMING_SWITCH_SPEED_ADDR    0x6041
#define HOMING_ZERO_SPEED_ADDR      0x6043
#define HOMING_ACCELERATION_ADDR    0x6045
#define HOMING_OFFSET_ADDR          0x6024
#define HOMING_DONE_BEHAVIOUR_ADDR  0x201F

#define JOG_SPEED_ADDR              0x2300
#define JOG_ACCELERATION_ADDR       0x2301
#define JOG_DECELERATION_ADDR       0x2302
#define JOG_S_CURVE_ADDR            0x2303
#define JOG_SERVO_LOCK_ADDR         0x2311

#define POS_CTRL_MODE_ADDR          0x3000
#define POS_START_INDEX_NUMBER_ADDR 0x3009
#define POS_INDEX_TYPE_ADDR         0x3101
#define POS_REG_DISTANCE_ADDR       0x310A
#define POS_REG_VELOCITY_ADDR       0x310C
#define POS_REPEAT_COUNT_ADDR       0x310E
#define POS_DWELLTIME_ADDR          0x310F
#define POS_NEXT_INDEX_ADDR         0x3110
#define POS_ACTION_ADDR             0x3111

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

enum class CommandState {
	INIT, HOME, POSITION, JOG, IDLE, ERROR
};

enum class FunctionState {
	INIT, SET, ACTION, DONE, IDLE, ERROR
};

enum class InitState {
    HOME_SET, HOME_ACTION, POSITION_SET, POSITION_ACTION, JOG_SET, JOG_ACTION, IDLE
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

class AxisMsg {
public:
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
public:
    std::string node_name;

    std::string serial_port;
    int32_t baud_rate;

    // ERROR
    int32_t isError;
    std::string errorMessage;

    // STATE
	CommandState cmdState;
	FunctionState funcState;
    
    int32_t axis_x_num;
    int32_t axis_y_num;
    int32_t axis_z_num;

    // GENERAL INFO
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

    // GENERAL DEFAULT
    int32_t q_stop_deceleration_val;
    
    // HOMING DEFAULT
    int32_t homing_method_val;
    int32_t homing_min_speed_val;
    int32_t homing_speed_x_val;
    int32_t homing_speed_y_val;
    int32_t homing_speed_z_val;
    int32_t homing_acceleration_val;
    int32_t homing_offset_x_val;
    int32_t homing_offset_y_val;
    int32_t homing_offset_z_val;
    OnOff homing_done_behaviour_x_val;
    OnOff homing_done_behaviour_y_val;
    OnOff homing_done_behaviour_z_val;

    // JOG DEFAULT
    int32_t jog_min_speed_val;
    int32_t jog_speed_val;
    int32_t jog_acceleration_val;
    int32_t jog_deceleration_val;
    int32_t jog_s_curve_val;
    int32_t jog_servo_lock_val;

    // POSITION DEFAULT
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

int32_t axisToId(std::string axis);

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