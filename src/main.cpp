#include <ros/ros.h>
#include <unistd.h>
#include <pthread.h>
#include <semaphore.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/time.h>
#include <boost/thread.hpp>
#include <iostream>
#include <queue>
#include <modbus.h>
#include <gantry_robot/Info.h>
#include <gantry_robot/Position.h>
#include <gantry_robot/Homing.h>

#include "L7P.h"
#include "ObjectDictionary.h"
#include "main.h"

std::queue<AxisMsg> que;

gantry_robot::Info info;
GlobalInfo gInfo;

bool servicePositionCallback(gantry_robot::Position::Request &req, gantry_robot::Position::Response &res) {
    ros::Time time = ros::Time::now();

	// setPosParametersMsg(&que, axisToId(req.axis));
	setPositionMsg(&que, axisToId(req.axis), req.position, req.speed, req.acc, req.dec);

	setAxisCommandMsg(&que, axisToId(req.axis), AxisCommand::start, OnOff::on);
	setAxisCommandMsg(&que, axisToId(req.axis), AxisCommand::start, OnOff::off);

	res.success = 1;
	
	reprintf(ScreenOutput::ALWAYS, "[%s{%s}(%d)]\n", __FILENAME__, __FUNCTION__, __LINE__);

    return true;
}

bool serviceHomingCallback(gantry_robot::Homing::Request &req, gantry_robot::Homing::Response &res) {
    ros::Time time = ros::Time::now();

	setAxisCommandMsg(&que, axisToId(req.axis), AxisCommand::emg, OnOff::off);
	setAxisCommandMsg(&que, axisToId(req.axis), AxisCommand::a_rst, OnOff::on);
	setAxisCommandMsg(&que, axisToId(req.axis), AxisCommand::a_rst, OnOff::off);
	setAxisCommandMsg(&que, axisToId(req.axis), AxisCommand::stop, OnOff::off);
	setAxisCommandMsg(&que, axisToId(req.axis), AxisCommand::sv_on, OnOff::on);

	if (gInfo.axis_x_num == axisToId(req.axis)) {
		if (info.axisX.status.output.org == (uint8_t)OnOff::on && req.setForce || !info.axisX.status.output.org) {
			setHomingParametersMsg(&que, axisToId(req.axis), gInfo.homing_speed_x_val, gInfo.homing_offset_x_val, gInfo.homing_done_behaviour_x_val);
			setAxisCommandMsg(&que, axisToId(req.axis), AxisCommand::hstart, OnOff::on);
			setAxisCommandMsg(&que, axisToId(req.axis), AxisCommand::hstart, OnOff::off);
		}
	} else if (gInfo.axis_y_num == axisToId(req.axis)) {
		if (info.axisY.status.output.org == (uint8_t)OnOff::on && req.setForce || !info.axisY.status.output.org) {
			setHomingParametersMsg(&que, axisToId(req.axis), gInfo.homing_speed_y_val, gInfo.homing_offset_y_val, gInfo.homing_done_behaviour_y_val);
			setAxisCommandMsg(&que, axisToId(req.axis), AxisCommand::hstart, OnOff::on);
			setAxisCommandMsg(&que, axisToId(req.axis), AxisCommand::hstart, OnOff::off);
		}
	} else if (gInfo.axis_z_num == axisToId(req.axis)) {
		if (info.axisZ.status.output.org == (uint8_t)OnOff::on && req.setForce || !info.axisZ.status.output.org) {
			setHomingParametersMsg(&que, axisToId(req.axis), gInfo.homing_speed_z_val, gInfo.homing_offset_z_val, gInfo.homing_done_behaviour_z_val);
			setAxisCommandMsg(&que, axisToId(req.axis), AxisCommand::hstart, OnOff::on);
			setAxisCommandMsg(&que, axisToId(req.axis), AxisCommand::hstart, OnOff::off);
		}
	} else {
		res.success = -1;
		reprintf(ScreenOutput::ERROR, "[%s{%s}(%d)]\n", __FILENAME__, __FUNCTION__, __LINE__);

		return true;
	}

	res.success = 1;
	
	reprintf(ScreenOutput::ALWAYS, "[%s{%s}(%d)]\n", __FILENAME__, __FUNCTION__, __LINE__);

    return true;
}

void modbusLoop(int rate, std::queue<AxisMsg>* que, ModbusLoopState* modbusLoopState, ros::Publisher* pub_info, modbus_t* ctx) {
    int size;
	ros::Time ts_now;
	AxisMsg axisMsg;

	ros::Rate r(rate);

	while (ros::ok() && *modbusLoopState!=ModbusLoopState::FINISH)
	{
        size = que->size();

        if (size) {
			axisMsg = que->front();
			switch(axisMsg.type) {
				case CommandType::setCommand:
					reprintf(ScreenOutput::NO, "[%s{%s}(%d)] : CommandType::setCommand\n", __FILENAME__, __FUNCTION__, __LINE__);
					if (!setAxisCommand(ctx, axisMsg.id, axisMsg.axisCommand, axisMsg.onOff)) {
						reprintf(ScreenOutput::ERROR, "[%s{%s}(%d)] : setAxisCommand error\n", __FILENAME__, __FUNCTION__, __LINE__);
					} else {
						que->pop();
					}
				break;
				case CommandType::getStatus:
					reprintf(ScreenOutput::NO, "[%s{%s}(%d)] : CommandType::getStatus\n", __FILENAME__, __FUNCTION__, __LINE__);
				break;
				case CommandType::setParameter:
					reprintf(ScreenOutput::NO, "[%s{%s}(%d)] : CommandType::setParameter\n", __FILENAME__, __FUNCTION__, __LINE__);
				break;
				case CommandType::getParameter:
					reprintf(ScreenOutput::NO, "[%s{%s}(%d)] : CommandType::getParameter\n", __FILENAME__, __FUNCTION__, __LINE__);
				break;
				case CommandType::setHomingParameters:
					reprintf(ScreenOutput::NO, "[%s{%s}(%d)] : CommandType::setHomingParameters\n", __FILENAME__, __FUNCTION__, __LINE__);
					if (!setHomingParameters(ctx, axisMsg.id, axisMsg.speed, axisMsg.offset, axisMsg.done_behaviour)) {
						reprintf(ScreenOutput::ERROR, "[%s{%s}(%d)] : setHomingParameters error\n", __FILENAME__, __FUNCTION__, __LINE__);
					} else {
						que->pop();
					}
				break;
				case CommandType::setJogParameters:
					reprintf(ScreenOutput::NO, "[%s{%s}(%d)] : CommandType::setJogParameters\n", __FILENAME__, __FUNCTION__, __LINE__);
					if (!setJogParameters(ctx, axisMsg.id, axisMsg.speed, axisMsg.acc, axisMsg.dec, axisMsg.s_curve, axisMsg.servo_lock)) {
						reprintf(ScreenOutput::ERROR, "[%s{%s}(%d)] : setJogParameters error\n", __FILENAME__, __FUNCTION__, __LINE__);
					} else {
						que->pop();
					}
				break;
				case CommandType::setPosParameters:
					reprintf(ScreenOutput::NO, "[%s{%s}(%d)] : CommandType::setPosParameters\n", __FILENAME__, __FUNCTION__, __LINE__);
					if (!setPosParameters(ctx, axisMsg.id)) {
						reprintf(ScreenOutput::ERROR, "[%s{%s}(%d)] : setPosParameters error\n", __FILENAME__, __FUNCTION__, __LINE__);
					} else {
						que->pop();
					}
				break;
				case CommandType::setPosition:
					reprintf(ScreenOutput::DEFAULT, "[%s{%s}(%d)] : CommandType::setPosition\n", __FILENAME__, __FUNCTION__, __LINE__);
					if (!setPosition(ctx, axisMsg.id, axisMsg.position, axisMsg.speed, axisMsg.acc, axisMsg.dec)) {
						reprintf(ScreenOutput::ERROR, "[%s{%s}(%d)] : setPosition error\n", __FILENAME__, __FUNCTION__, __LINE__);
					} else {
						que->pop();
					}
				break;
				default:
				break;
			}
		} else {
		}

		if (!getAxisStatus(ctx, gInfo.axis_x_num, &info.axisX.status)) reprintf(ScreenOutput::ERROR, "getAxisStatus AXIS_X error\n");		
		if (!getAxisParameter(ctx, gInfo.axis_x_num, ACT_POS, &info.axisX.position)) reprintf(ScreenOutput::ERROR, "getAxisParameter AXIS_X error\n");
		if (!getAxisParameter(ctx, gInfo.axis_x_num, ACT_SPD, &info.axisX.speed)) reprintf(ScreenOutput::ERROR, "getAxisParameter AXIS_X error\n");

		if (!getAxisStatus(ctx, gInfo.axis_y_num, &info.axisY.status)) reprintf(ScreenOutput::ERROR, "getAxisStatus AXIS_Y error\n");		
		if (!getAxisParameter(ctx, gInfo.axis_y_num, ACT_POS, &info.axisY.position)) reprintf(ScreenOutput::ERROR, "getAxisParameter AXIS_Y error\n");
		if (!getAxisParameter(ctx, gInfo.axis_y_num, ACT_SPD, &info.axisY.speed)) reprintf(ScreenOutput::ERROR, "getAxisParameter AXIS_Y error\n");

		if (!getAxisStatus(ctx, gInfo.axis_z_num, &info.axisZ.status)) reprintf(ScreenOutput::ERROR, "getAxisStatus AXIS_Z error\n");		
		if (!getAxisParameter(ctx, gInfo.axis_z_num, ACT_POS, &info.axisZ.position)) reprintf(ScreenOutput::ERROR, "getAxisParameter AXIS_Z error\n");
		if (!getAxisParameter(ctx, gInfo.axis_z_num, ACT_SPD, &info.axisZ.speed)) reprintf(ScreenOutput::ERROR, "getAxisParameter AXIS_Z error\n");

		ts_now = ros::Time::now();
        info.header.stamp = ts_now;
        pub_info->publish(info);

		if (*modbusLoopState!=ModbusLoopState::FINISH) {
			*modbusLoopState = ModbusLoopState::RUNNING;
		}

		r.sleep();
	}

	setAxisCommand(ctx, gInfo.axis_x_num, AxisCommand::stop, OnOff::on);
	setAxisCommand(ctx, gInfo.axis_y_num, AxisCommand::stop, OnOff::on);
	setAxisCommand(ctx, gInfo.axis_z_num, AxisCommand::stop, OnOff::on);
}

int main(int argc, char* argv[]) {
	ros::init(argc, argv, "gantry_robot");
	ros::NodeHandle nh("~");

	int32_t id = 0;
	ModbusLoopState modbusLoopState = ModbusLoopState::INIT;

#if 0
	ros::param::get("~node_name", node_name);
	ros::param::get("~serial_port", serial_port);
	ros::param::get("~baud_rate", baud_rate);
#else	
    nh.getParam("node_name", gInfo.node_name);
    nh.getParam("serial_port", gInfo.serial_port);
    nh.getParam("baud_rate", gInfo.baud_rate);
    nh.getParam("axis_x_num", gInfo.axis_x_num);
    nh.getParam("axis_y_num", gInfo.axis_y_num);
    nh.getParam("axis_z_num", gInfo.axis_z_num);
    nh.getParam("encoder_ppr_axis_x", gInfo.encoder_ppr_axis_x);
    nh.getParam("encoder_ppr_axis_y", gInfo.encoder_ppr_axis_y);
    nh.getParam("encoder_ppr_axis_z", gInfo.encoder_ppr_axis_z);
    nh.getParam("stage_max_axis_x", gInfo.stage_max_axis_x);
    nh.getParam("stage_max_axis_y", gInfo.stage_max_axis_y);
    nh.getParam("stage_max_axis_z", gInfo.stage_max_axis_z);
    nh.getParam("stage_lead_axis_x", gInfo.stage_lead_axis_x);
    nh.getParam("stage_lead_axis_y", gInfo.stage_lead_axis_y);
    nh.getParam("stage_lead_axis_z", gInfo.stage_lead_axis_z);
    nh.getParam("ratio_gear_axis_x", gInfo.ratio_gear_axis_x);
    nh.getParam("ratio_gear_axis_y", gInfo.ratio_gear_axis_y);
    nh.getParam("ratio_gear_axis_z", gInfo.ratio_gear_axis_z);
    nh.getParam("ratio_shaft_axis_x", gInfo.ratio_shaft_axis_x);
    nh.getParam("ratio_shaft_axis_y", gInfo.ratio_shaft_axis_y);
    nh.getParam("ratio_shaft_axis_z", gInfo.ratio_shaft_axis_z);
    nh.getParam("q_stop_deceleration_val", gInfo.q_stop_deceleration_val);
    nh.getParam("homing_method_val", gInfo.homing_method_val);
    nh.getParam("homing_min_speed_val", gInfo.homing_min_speed_val);
    nh.getParam("homing_speed_x_val", gInfo.homing_speed_x_val);
    nh.getParam("homing_speed_y_val", gInfo.homing_speed_y_val);
    nh.getParam("homing_speed_z_val", gInfo.homing_speed_z_val);
    nh.getParam("homing_acceleration_val", gInfo.homing_acceleration_val);
    nh.getParam("homing_offset_x_val", gInfo.homing_offset_x_val);
    nh.getParam("homing_offset_y_val", gInfo.homing_offset_y_val);
    nh.getParam("homing_offset_z_val", gInfo.homing_offset_z_val);
	int32_t temp_val;
    nh.getParam("homing_done_behaviour_x_val", temp_val);
	temp_val!=0?gInfo.homing_done_behaviour_x_val=OnOff::on:gInfo.homing_done_behaviour_x_val=OnOff::off;
    nh.getParam("homing_done_behaviour_y_val", temp_val);
	temp_val!=0?gInfo.homing_done_behaviour_y_val=OnOff::on:gInfo.homing_done_behaviour_y_val=OnOff::off;
    nh.getParam("homing_done_behaviour_z_val", temp_val);
	temp_val!=0?gInfo.homing_done_behaviour_z_val=OnOff::on:gInfo.homing_done_behaviour_z_val=OnOff::off;
    nh.getParam("jog_min_speed_val", gInfo.jog_min_speed_val);
    nh.getParam("jog_speed_val", gInfo.jog_speed_val);
    nh.getParam("jog_acceleration_val", gInfo.jog_acceleration_val);
    nh.getParam("jog_deceleration_val", gInfo.jog_deceleration_val);
    nh.getParam("jog_s_curve_val", gInfo.jog_s_curve_val);
    nh.getParam("jog_servo_lock_val", gInfo.jog_servo_lock_val);
    nh.getParam("pos_ctrl_mode_val", gInfo.pos_ctrl_mode_val);
    nh.getParam("pos_start_index_number_val", gInfo.pos_start_index_number_val);
    nh.getParam("pos_index_type_val", gInfo.pos_index_type_val);
    nh.getParam("pos_reg_distance_val", gInfo.pos_reg_distance_val);
    nh.getParam("pos_reg_velocity_val", gInfo.pos_reg_velocity_val);
    nh.getParam("pos_repeat_count_val", gInfo.pos_repeat_count_val);
    nh.getParam("pos_dwelltime_val", gInfo.pos_dwelltime_val);
    nh.getParam("pos_next_index_val", gInfo.pos_next_index_val);
    nh.getParam("pos_action_val", gInfo.pos_action_val);
#endif

	CommandCase cmdCase = CommandCase::INIT;
	CommandCase cmdCasePre = cmdCase;
	FunctionCase funcCase = FunctionCase::INIT;

	modbus_t* ctx;

	ctx = modbus_new_rtu(gInfo.serial_port.c_str(), gInfo.baud_rate, 'N', 8, 1);

	// modbus_new_rtu return
	// pointer : successful
	// NULL : error, set errno
	if (ctx == NULL) {
        reprintf(ScreenOutput::ERROR, "[%s{%s}(%d)] : Unable to allocate libmodbus context, error msg: %s \n",
			__FILENAME__, __FUNCTION__, __LINE__, modbus_strerror(errno));

		return -1;
	} else {
		reprintf(ScreenOutput::DEFAULT, "[%s{%s}(%d)] : serial port : %s, baudrate : %d\n",
			__FILENAME__, __FUNCTION__, __LINE__, gInfo.serial_port.c_str(), gInfo.baud_rate);
	}

	// modbus_set_slave return
	// 0: successful
	// -1 : error, set errno
	if (modbus_set_slave(ctx, id) == -1) {
        reprintf(ScreenOutput::ERROR, "[%s{%s}(%d)] : modbus_set_slave error, slave: #%d, error msg: %s \n",
			__FILENAME__, __FUNCTION__, __LINE__, id, modbus_strerror(errno));

		return -1;
	}

	#if 0
	// modbus_set_debug no return
	modbus_set_debug(ctx, TRUE);
	#endif

	// modbus_connect return
	// 0: successful
	// -1 : error, set errno
	if (modbus_connect(ctx) == -1) {
		reprintf(ScreenOutput::ERROR, "[%s{%s}(%d)] : Connection failed: %s \n",
			__FILENAME__, __FUNCTION__, __LINE__, id, modbus_strerror(errno));

		return -1;
	}

    ros::ServiceServer service_position = nh.advertiseService("gantry_robot_position", servicePositionCallback);
    ros::ServiceServer service_homing = nh.advertiseService("gantry_robot_homing", serviceHomingCallback);

    ros::Publisher pub_info = nh.advertise<gantry_robot::Info>("gantry_robot_info", 100);

    int main_hz = 1000;
	AxisMsg axisMsg;
    boost::thread threadModbusLoop(modbusLoop, main_hz, &que, &modbusLoopState, &pub_info, ctx);

	#define STEP_TIME 1.0
	double time_cur;
	double time_pre;
	double time_diff;

	ros::Rate r(1000);

	time_cur = ros::Time::now().toSec();
	time_pre = time_cur;
	time_diff;

	while (ros::ok()) {
		time_cur = ros::Time::now().toSec();
		time_diff = time_cur - time_pre;
		if ( time_diff > STEP_TIME ) {
			time_pre = time_cur;
		}

		if (cmdCase != cmdCasePre) {
			funcCase = FunctionCase::INIT;
		}
		cmdCasePre = cmdCase;
		reprintf(ScreenOutput::NO, "[%s{%s}(%d)] : CommandCase::%d FunctionCase::%d\n", __FILENAME__, __FUNCTION__, __LINE__, (int32_t)cmdCase, (int32_t)funcCase);

		switch (cmdCase) {
			case CommandCase::INIT:
				switch (funcCase) {
					case FunctionCase::INIT:
						funcCase = FunctionCase::SET;
					break;
					case FunctionCase::SET:
						funcCase = FunctionCase::ACTION;
					break;
					case FunctionCase::ACTION:
						if (modbusLoopState == ModbusLoopState::RUNNING) {
							reprintf(ScreenOutput::DEFAULT, "[%s{%s}(%d)] : CommandCase::INIT FunctionCase::ACTION success\n", __FILENAME__, __FUNCTION__, __LINE__);
							cmdCase = CommandCase::HOME;
							funcCase = FunctionCase::INIT;
						} else {
						}
					break;
					case FunctionCase::DONE:
						funcCase = FunctionCase::IDLE;
					break;
					case FunctionCase::IDLE:
						funcCase = FunctionCase::IDLE;
					break;
					case FunctionCase::ERROR:
						cmdCase = CommandCase::ERROR;
						funcCase = FunctionCase::INIT;
					break;
				}
			break;
			case CommandCase::HOME:
				switch (funcCase) {
					case FunctionCase::INIT:
						// Axis Reset
						setAxisCommandMsg(&que, gInfo.axis_x_num, AxisCommand::emg, OnOff::off);
						setAxisCommandMsg(&que, gInfo.axis_y_num, AxisCommand::emg, OnOff::off);
						setAxisCommandMsg(&que, gInfo.axis_z_num, AxisCommand::emg, OnOff::off);

						setAxisCommandMsg(&que, gInfo.axis_x_num, AxisCommand::a_rst, OnOff::on);
						setAxisCommandMsg(&que, gInfo.axis_y_num, AxisCommand::a_rst, OnOff::on);
						setAxisCommandMsg(&que, gInfo.axis_z_num, AxisCommand::a_rst, OnOff::on);

						setAxisCommandMsg(&que, gInfo.axis_x_num, AxisCommand::a_rst, OnOff::off);
						setAxisCommandMsg(&que, gInfo.axis_y_num, AxisCommand::a_rst, OnOff::off);
						setAxisCommandMsg(&que, gInfo.axis_z_num, AxisCommand::a_rst, OnOff::off);

						setAxisCommandMsg(&que, gInfo.axis_x_num, AxisCommand::stop, OnOff::off);
						setAxisCommandMsg(&que, gInfo.axis_y_num, AxisCommand::stop, OnOff::off);
						setAxisCommandMsg(&que, gInfo.axis_z_num, AxisCommand::stop, OnOff::off);

						setAxisCommandMsg(&que, gInfo.axis_x_num, AxisCommand::sv_on, OnOff::on);
						setAxisCommandMsg(&que, gInfo.axis_y_num, AxisCommand::sv_on, OnOff::on);
						setAxisCommandMsg(&que, gInfo.axis_z_num, AxisCommand::sv_on, OnOff::on);

#if 0
						if (info.axisZ.status.output.org == (uint8_t)OnOff::on) {
							reprintf(ScreenOutput::DEFAULT, "[%s{%s}(%d)] : CommandCase::HOME FunctionCase::INIT !already done!\n", __FILENAME__, __FUNCTION__, __LINE__);
							funcCase = FunctionCase::IDLE;
						} else {
							reprintf(ScreenOutput::DEFAULT, "[%s{%s}(%d)] : CommandCase::HOME FunctionCase::INIT\n", __FILENAME__, __FUNCTION__, __LINE__);
							funcCase = FunctionCase::SET;
						}
#else
						funcCase = FunctionCase::IDLE;
#endif
					break;
					case FunctionCase::SET:
						reprintf(ScreenOutput::DEFAULT, "[%s{%s}(%d)] : CommandCase::HOME FunctionCase::SET\n", __FILENAME__, __FUNCTION__, __LINE__);
						// Homing Parameter
						setHomingParametersMsg(&que, gInfo.axis_z_num, gInfo.homing_speed_z_val, gInfo.homing_offset_z_val, gInfo.homing_done_behaviour_z_val);
						funcCase = FunctionCase::ACTION;
					break;
					case FunctionCase::ACTION:
						reprintf(ScreenOutput::DEFAULT, "[%s{%s}(%d)] : CommandCase::HOME FunctionCase::ACTION\n", __FILENAME__, __FUNCTION__, __LINE__);
						// Homing
						setAxisCommandMsg(&que, gInfo.axis_z_num, AxisCommand::hstart, OnOff::on);
						setAxisCommandMsg(&que, gInfo.axis_z_num, AxisCommand::hstart, OnOff::off);
						funcCase = FunctionCase::DONE;
					break;
					case FunctionCase::DONE:
						funcCase = FunctionCase::IDLE;
					break;
					case FunctionCase::IDLE:
						// funcCase = FunctionCase::IDLE;
						reprintf(ScreenOutput::DEFAULT, "[%s{%s}(%d)] : CommandCase::HOME FunctionCase::IDLE\n", __FILENAME__, __FUNCTION__, __LINE__);
						cmdCase = CommandCase::POSITION;
						// cmdCase = CommandCase::JOG;
						funcCase = FunctionCase::INIT;
					break;
					case FunctionCase::ERROR:
						reprintf(ScreenOutput::DEFAULT, "[%s{%s}(%d)] : CommandCase::HOME FunctionCase::ERROR\n", __FILENAME__, __FUNCTION__, __LINE__);
						cmdCase = CommandCase::ERROR;
						funcCase = FunctionCase::INIT;
					break;
				}
			break;
			case CommandCase::POSITION:
				switch (funcCase) {
					case FunctionCase::INIT:
#if 0
						if (info.axisZ.status.output.inpos1 == (uint8_t)OnOff::on &&
							info.axisZ.status.output.inspd == (uint8_t)OnOff::on &&
							info.axisZ.status.output.org == (uint8_t)OnOff::on) {
							funcCase = FunctionCase::SET;
						} else {
						}
						reprintf(ScreenOutput::DEFAULT, "[%s{%s}(%d)] : CommandCase::POSITION FunctionCase::INIT\n", __FILENAME__, __FUNCTION__, __LINE__);
#else
						funcCase = FunctionCase::SET;
#endif
					break;
					case FunctionCase::SET:
						reprintf(ScreenOutput::DEFAULT, "[%s{%s}(%d)] : CommandCase::POSITION FunctionCase::SET\n", __FILENAME__, __FUNCTION__, __LINE__);
						// Position Parameter
						setPosParametersMsg(&que, gInfo.axis_x_num);
						setPosParametersMsg(&que, gInfo.axis_y_num);
						setPosParametersMsg(&que, gInfo.axis_z_num);

						funcCase = FunctionCase::ACTION;
					break;
					case FunctionCase::ACTION:

						funcCase = FunctionCase::DONE;
					break;
					case FunctionCase::DONE:
						funcCase = FunctionCase::IDLE;
					break;
					case FunctionCase::IDLE:
						// if (info.axisZ.status.output.inpos1 == (uint8_t)OnOff::on) {
						// 	funcCase = FunctionCase::SET;
						// } else {
							cmdCase = CommandCase::IDLE;
							funcCase = FunctionCase::IDLE;
						// }
					break;
					case FunctionCase::ERROR:
						cmdCase = CommandCase::ERROR;
						funcCase = FunctionCase::INIT;
					break;
				}
			break;
			case CommandCase::JOG:
				switch (funcCase) {
					case FunctionCase::INIT:
						if (info.axisZ.status.output.org == (uint8_t)OnOff::on) {
							funcCase = FunctionCase::SET;
						} else {
						}
						reprintf(ScreenOutput::DEFAULT, "[%s{%s}(%d)] : CommandCase::JOG FunctionCase::INIT\n", __FILENAME__, __FUNCTION__, __LINE__);
					break;
					case FunctionCase::SET:
						reprintf(ScreenOutput::DEFAULT, "[%s{%s}(%d)] : CommandCase::JOG FunctionCase::SET\n", __FILENAME__, __FUNCTION__, __LINE__);
						// Jog Parameter
						setJogParametersMsg(&que, gInfo.axis_z_num, gInfo.jog_min_speed_val, gInfo.jog_acceleration_val, gInfo.jog_deceleration_val, gInfo.jog_s_curve_val, OnOff::off);
						funcCase = FunctionCase::ACTION;
					break;
					case FunctionCase::ACTION:
						funcCase = FunctionCase::DONE;
					break;
					case FunctionCase::DONE:
						funcCase = FunctionCase::IDLE;
					break;
					case FunctionCase::IDLE:
						funcCase = FunctionCase::IDLE;
					break;
					case FunctionCase::ERROR:
						cmdCase = CommandCase::ERROR;
						funcCase = FunctionCase::INIT;
					break;
				}
			break;
			case CommandCase::IDLE:
				switch (funcCase) {
					case FunctionCase::INIT:
						funcCase = FunctionCase::SET;
					break;
					case FunctionCase::SET:
						funcCase = FunctionCase::ACTION;
					break;
					case FunctionCase::ACTION:
						funcCase = FunctionCase::DONE;
					break;
					case FunctionCase::DONE:
							reprintf(ScreenOutput::ALWAYS, "[%s{%s}(%d)] : CommandCase::IDLE FunctionCase::DONE\n", __FILENAME__, __FUNCTION__, __LINE__);
						funcCase = FunctionCase::IDLE;
					break;
					case FunctionCase::IDLE:
						funcCase = FunctionCase::IDLE;
					break;
					case FunctionCase::ERROR:
						cmdCase = CommandCase::ERROR;
						funcCase = FunctionCase::INIT;
					break;
				}
			break;
			case CommandCase::ERROR:
				switch (funcCase) {
					case FunctionCase::INIT:
						funcCase = FunctionCase::SET;
					break;
					case FunctionCase::SET:
						funcCase = FunctionCase::ACTION;
					break;
					case FunctionCase::ACTION:
						funcCase = FunctionCase::IDLE;
					break;
					case FunctionCase::IDLE:
						funcCase = FunctionCase::IDLE;
					break;
					case FunctionCase::ERROR:
						cmdCase = CommandCase::ERROR;
						funcCase = FunctionCase::INIT;
					break;
				}
			break;
		}

		ros::spinOnce();

		r.sleep();
	}

	modbusLoopState = ModbusLoopState::FINISH;
    threadModbusLoop.join();

	reprintf(ScreenOutput::ALWAYS, "[%s{%s}(%d)] : threadModbusLoop joined\n", __FILENAME__, __FUNCTION__, __LINE__);

	// modbus_close no return
	modbus_close(ctx);
	// modbus_free no return
	modbus_free(ctx);

	reprintf(ScreenOutput::ALWAYS, "[%s{%s}(%d)] : program end\n", __FILENAME__, __FUNCTION__, __LINE__);

	return 0;
}

void reprintf(ScreenOutput screenOutput, const char* format, ...) {
    va_list argptr;

    if (screenOutput == ScreenOutput::ALWAYS || screenOutput == ScreenOutput::ERROR) {
		printf("[%lf]", ros::Time::now().toSec());
        va_start(argptr, format);
        vprintf(format, argptr);
        va_end(argptr);
    } else if (screenOutput == ScreenOutput::TEMP) {
#if OUTPUT_TEMP
		printf("[%lf]", ros::Time::now().toSec());
        va_start(argptr, format);
        vprintf(format, argptr);
        va_end(argptr);
#endif
    } else if (screenOutput == ScreenOutput::DEFAULT) {
#if OUTPUT_DEFAULT
		printf("[%lf]", ros::Time::now().toSec());
        va_start(argptr, format);
        vprintf(format, argptr);
        va_end(argptr);
#endif
    } else if (screenOutput == ScreenOutput::NO) {
    } else {
    }
}