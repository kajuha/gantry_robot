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
#include <gantry_robot/Location.h>
#include <gantry_robot/Homing.h>
#include <gantry_robot/Command.h>
#ifdef YAPPER_ENABLE
#include <yapper/YapIn.h>
#endif

#include "L7P.h"
#include "ObjectDictionary.h"
#include "main.h"

std::queue<AxisMsg> queueModbus;
std::queue<CommandState> queueCommandState;

gantry_robot::Info info;
GlobalInfo gInfo;

void setState();

bool servicePositionCallback(gantry_robot::Position::Request &req, gantry_robot::Position::Response &res) {
    ros::Time time = ros::Time::now();

	std::string axis = req.axis;
	std::transform(axis.begin(), axis.end(), axis.begin(), ::tolower);
	static double position, speed, acc, dec;

	if (axis == "x" || axis == "y" || axis == "z") {
		if (gInfo.node_name == "serial_robot") {
			reprintf(ScreenOutput::ALWAYS, "serial_robot pre: %lf %lf %lf %lf\n", req.position, req.speed, req.acc, req.dec);
			position = DEG_TO_RAD(req.position);
			speed = DEG_TO_RAD(req.speed);
			acc = DEG_TO_RAD(req.acc);
			dec = DEG_TO_RAD(req.dec);
			reprintf(ScreenOutput::ALWAYS, "serial_robot mid: %lf %lf %lf %lf\n", position, speed, acc, dec);

			if (axis == "x") {
				position = position * ((gInfo.enc_pulse_per_rev_axis_x * gInfo.ratio_gear_axis_x)/(DEG_TO_RAD(gInfo.stage_mm_per_rev_axis_x) * gInfo.ratio_shaft_axis_x));
				speed = speed * ((gInfo.enc_pulse_per_rev_axis_x * gInfo.ratio_gear_axis_x)/(DEG_TO_RAD(gInfo.stage_mm_per_rev_axis_x) * gInfo.ratio_shaft_axis_x));;
				acc = acc * ((gInfo.enc_pulse_per_rev_axis_x * gInfo.ratio_gear_axis_x)/(DEG_TO_RAD(gInfo.stage_mm_per_rev_axis_x) * gInfo.ratio_shaft_axis_x));;
				dec = dec * ((gInfo.enc_pulse_per_rev_axis_x * gInfo.ratio_gear_axis_x)/(DEG_TO_RAD(gInfo.stage_mm_per_rev_axis_x) * gInfo.ratio_shaft_axis_x));;
			} else if (axis == "y") {
				position = position * ((gInfo.enc_pulse_per_rev_axis_y * gInfo.ratio_gear_axis_y)/(DEG_TO_RAD(gInfo.stage_mm_per_rev_axis_y) * gInfo.ratio_shaft_axis_y));
				speed = speed * ((gInfo.enc_pulse_per_rev_axis_y * gInfo.ratio_gear_axis_y)/(DEG_TO_RAD(gInfo.stage_mm_per_rev_axis_y) * gInfo.ratio_shaft_axis_y));;
				acc = acc * ((gInfo.enc_pulse_per_rev_axis_y * gInfo.ratio_gear_axis_y)/(DEG_TO_RAD(gInfo.stage_mm_per_rev_axis_y) * gInfo.ratio_shaft_axis_y));;
				dec = dec * ((gInfo.enc_pulse_per_rev_axis_y * gInfo.ratio_gear_axis_y)/(DEG_TO_RAD(gInfo.stage_mm_per_rev_axis_y) * gInfo.ratio_shaft_axis_y));;
			} else if (axis == "z") {
				position = position * ((gInfo.enc_pulse_per_rev_axis_z * gInfo.ratio_gear_axis_z)/(DEG_TO_RAD(gInfo.stage_mm_per_rev_axis_z) * gInfo.ratio_shaft_axis_z));
				speed = speed * ((gInfo.enc_pulse_per_rev_axis_z * gInfo.ratio_gear_axis_z)/(DEG_TO_RAD(gInfo.stage_mm_per_rev_axis_z) * gInfo.ratio_shaft_axis_z));;
				acc = acc * ((gInfo.enc_pulse_per_rev_axis_z * gInfo.ratio_gear_axis_z)/(DEG_TO_RAD(gInfo.stage_mm_per_rev_axis_z) * gInfo.ratio_shaft_axis_z));;
				dec = dec * ((gInfo.enc_pulse_per_rev_axis_z * gInfo.ratio_gear_axis_z)/(DEG_TO_RAD(gInfo.stage_mm_per_rev_axis_z) * gInfo.ratio_shaft_axis_z));;
			} else {
				res.success = SRV_FAIL;

				return true;
			}
			reprintf(ScreenOutput::ALWAYS, "serial_robot post: %lf %lf %lf %lf\n", position, speed, acc, dec);
		} else {
			reprintf(ScreenOutput::ALWAYS, "gantry_robot pre: %lf %lf %lf %lf\n", req.position, req.speed, req.acc, req.dec);
			position = M_TO_MM(req.position);
			speed = M_TO_MM(req.speed);
			acc = M_TO_MM(req.acc);
			dec = M_TO_MM(req.dec);
			reprintf(ScreenOutput::ALWAYS, "gantry_robot mid: %lf %lf %lf %lf\n", position, speed, acc, dec);

			if (axis == "x") {
				position = position * ((gInfo.enc_pulse_per_rev_axis_x * gInfo.ratio_gear_axis_x)/(gInfo.stage_mm_per_rev_axis_x * gInfo.ratio_shaft_axis_x));
				speed = speed * ((gInfo.enc_pulse_per_rev_axis_x * gInfo.ratio_gear_axis_x)/(gInfo.stage_mm_per_rev_axis_x * gInfo.ratio_shaft_axis_x));;
				acc = acc * ((gInfo.enc_pulse_per_rev_axis_x * gInfo.ratio_gear_axis_x)/(gInfo.stage_mm_per_rev_axis_x * gInfo.ratio_shaft_axis_x));;
				dec = dec * ((gInfo.enc_pulse_per_rev_axis_x * gInfo.ratio_gear_axis_x)/(gInfo.stage_mm_per_rev_axis_x * gInfo.ratio_shaft_axis_x));;
			} else if (axis == "y") {
				position = position * ((gInfo.enc_pulse_per_rev_axis_y * gInfo.ratio_gear_axis_y)/(gInfo.stage_mm_per_rev_axis_y * gInfo.ratio_shaft_axis_y));
				speed = speed * ((gInfo.enc_pulse_per_rev_axis_y * gInfo.ratio_gear_axis_y)/(gInfo.stage_mm_per_rev_axis_y * gInfo.ratio_shaft_axis_y));;
				acc = acc * ((gInfo.enc_pulse_per_rev_axis_y * gInfo.ratio_gear_axis_y)/(gInfo.stage_mm_per_rev_axis_y * gInfo.ratio_shaft_axis_y));;
				dec = dec * ((gInfo.enc_pulse_per_rev_axis_y * gInfo.ratio_gear_axis_y)/(gInfo.stage_mm_per_rev_axis_y * gInfo.ratio_shaft_axis_y));;
			} else if (axis == "z") {
				position = position * ((gInfo.enc_pulse_per_rev_axis_z * gInfo.ratio_gear_axis_z)/(gInfo.stage_mm_per_rev_axis_z * gInfo.ratio_shaft_axis_z));
				speed = speed * ((gInfo.enc_pulse_per_rev_axis_z * gInfo.ratio_gear_axis_z)/(gInfo.stage_mm_per_rev_axis_z * gInfo.ratio_shaft_axis_z));;
				acc = acc * ((gInfo.enc_pulse_per_rev_axis_z * gInfo.ratio_gear_axis_z)/(gInfo.stage_mm_per_rev_axis_z * gInfo.ratio_shaft_axis_z));;
				dec = dec * ((gInfo.enc_pulse_per_rev_axis_z * gInfo.ratio_gear_axis_z)/(gInfo.stage_mm_per_rev_axis_z * gInfo.ratio_shaft_axis_z));;
			} else {
				res.success = SRV_FAIL;

				return true;
			}
			reprintf(ScreenOutput::ALWAYS, "gantry_robot post: %lf %lf %lf %lf\n", position, speed, acc, dec);
		}
		setPosParametersMsg(&queueModbus, axisToId(axis));
		setPositionMsg(&queueModbus, axisToId(axis), (int32_t)position, (int32_t)speed, (int32_t)acc, (int32_t)dec);

		setAxisCommandMsg(&queueModbus, axisToId(axis), AxisCommand::start, OnOff::on);
		setAxisCommandMsg(&queueModbus, axisToId(axis), AxisCommand::start, OnOff::off);

		res.success = SRV_SUCCESS;

		reprintf(ScreenOutput::ALWAYS, "[%s{%s}(%d)]\n", __FILENAME__, __FUNCTION__, __LINE__);
	} else {
		res.success = SRV_FAIL;

		reprintf(ScreenOutput::ERROR, "[%s{%s}(%d)]\n", __FILENAME__, __FUNCTION__, __LINE__);
	}

    return true;
}

bool serviceLocationCallback(gantry_robot::Location::Request &req, gantry_robot::Location::Response &res) {
    ros::Time time = ros::Time::now();
	static double position_x, position_y, position_z, speed_x, speed_y, speed_z, acc_x, acc_y, acc_z, dec_x, dec_y, dec_z;
	static double position, speed, acc, dec;

	if (gInfo.node_name == "serial_robot") {
		speed = DEG_TO_RAD(gInfo.pos_speed_val);
		acc = DEG_TO_RAD(gInfo.pos_acc_dec_val);
		dec = DEG_TO_RAD(gInfo.pos_acc_dec_val);

		position = DEG_TO_RAD(req.x);
		position_x = position * ((gInfo.enc_pulse_per_rev_axis_x * gInfo.ratio_gear_axis_x)/(DEG_TO_RAD(gInfo.stage_mm_per_rev_axis_x) * gInfo.ratio_shaft_axis_x));
		speed_x = speed * ((gInfo.enc_pulse_per_rev_axis_x * gInfo.ratio_gear_axis_x)/(DEG_TO_RAD(gInfo.stage_mm_per_rev_axis_x) * gInfo.ratio_shaft_axis_x));
		acc_x = acc * ((gInfo.enc_pulse_per_rev_axis_x * gInfo.ratio_gear_axis_x)/(DEG_TO_RAD(gInfo.stage_mm_per_rev_axis_x) * gInfo.ratio_shaft_axis_x));
		dec_x = dec * ((gInfo.enc_pulse_per_rev_axis_x * gInfo.ratio_gear_axis_x)/(DEG_TO_RAD(gInfo.stage_mm_per_rev_axis_x) * gInfo.ratio_shaft_axis_x));
		setPosParametersMsg(&queueModbus, axisToId("x"));
		setPositionMsg(&queueModbus, axisToId("x"), (int32_t)position_x, (int32_t)speed_x, (int32_t)acc_x, (int32_t)dec_x);
		
		position = DEG_TO_RAD(req.y);
		position_y = position * ((gInfo.enc_pulse_per_rev_axis_y * gInfo.ratio_gear_axis_y)/(DEG_TO_RAD(gInfo.stage_mm_per_rev_axis_y) * gInfo.ratio_shaft_axis_y));
		speed_y = speed * ((gInfo.enc_pulse_per_rev_axis_y * gInfo.ratio_gear_axis_y)/(DEG_TO_RAD(gInfo.stage_mm_per_rev_axis_y) * gInfo.ratio_shaft_axis_y));
		acc_y = acc * ((gInfo.enc_pulse_per_rev_axis_y * gInfo.ratio_gear_axis_y)/(DEG_TO_RAD(gInfo.stage_mm_per_rev_axis_y) * gInfo.ratio_shaft_axis_y));
		dec_y = dec * ((gInfo.enc_pulse_per_rev_axis_y * gInfo.ratio_gear_axis_y)/(DEG_TO_RAD(gInfo.stage_mm_per_rev_axis_y) * gInfo.ratio_shaft_axis_y));
		setPosParametersMsg(&queueModbus, axisToId("y"));
		setPositionMsg(&queueModbus, axisToId("y"), (int32_t)position_y, (int32_t)speed_y, (int32_t)acc_y, (int32_t)dec_y);
		
		position = DEG_TO_RAD(req.z);
		position_z = position * ((gInfo.enc_pulse_per_rev_axis_z * gInfo.ratio_gear_axis_z)/(DEG_TO_RAD(gInfo.stage_mm_per_rev_axis_z) * gInfo.ratio_shaft_axis_z));
		speed_z = speed * ((gInfo.enc_pulse_per_rev_axis_z * gInfo.ratio_gear_axis_z)/(DEG_TO_RAD(gInfo.stage_mm_per_rev_axis_z) * gInfo.ratio_shaft_axis_z));
		acc_z = acc * ((gInfo.enc_pulse_per_rev_axis_z * gInfo.ratio_gear_axis_z)/(DEG_TO_RAD(gInfo.stage_mm_per_rev_axis_z) * gInfo.ratio_shaft_axis_z));
		dec_z = dec * ((gInfo.enc_pulse_per_rev_axis_z * gInfo.ratio_gear_axis_z)/(DEG_TO_RAD(gInfo.stage_mm_per_rev_axis_z) * gInfo.ratio_shaft_axis_z));
		setPosParametersMsg(&queueModbus, axisToId("z"));
		setPositionMsg(&queueModbus, axisToId("z"), (int32_t)position_z, (int32_t)speed_z, (int32_t)acc_z, (int32_t)dec_z);
	} else {
		speed = M_TO_MM(gInfo.pos_speed_val);
		acc = M_TO_MM(gInfo.pos_acc_dec_val);
		dec = M_TO_MM(gInfo.pos_acc_dec_val);

		position = M_TO_MM(req.x);
		position_x = position * ((gInfo.enc_pulse_per_rev_axis_x * gInfo.ratio_gear_axis_x)/(gInfo.stage_mm_per_rev_axis_x * gInfo.ratio_shaft_axis_x));
		speed_x = speed * ((gInfo.enc_pulse_per_rev_axis_x * gInfo.ratio_gear_axis_x)/(gInfo.stage_mm_per_rev_axis_x * gInfo.ratio_shaft_axis_x));
		acc_x = acc * ((gInfo.enc_pulse_per_rev_axis_x * gInfo.ratio_gear_axis_x)/(gInfo.stage_mm_per_rev_axis_x * gInfo.ratio_shaft_axis_x));
		dec_x = dec * ((gInfo.enc_pulse_per_rev_axis_x * gInfo.ratio_gear_axis_x)/(gInfo.stage_mm_per_rev_axis_x * gInfo.ratio_shaft_axis_x));
		setPosParametersMsg(&queueModbus, axisToId("x"));
		setPositionMsg(&queueModbus, axisToId("x"), (int32_t)position_x, (int32_t)speed_x, (int32_t)acc_x, (int32_t)dec_x);
		
		position = M_TO_MM(req.y);
		position_y = position * ((gInfo.enc_pulse_per_rev_axis_y * gInfo.ratio_gear_axis_y)/(gInfo.stage_mm_per_rev_axis_y * gInfo.ratio_shaft_axis_y));
		speed_y = speed * ((gInfo.enc_pulse_per_rev_axis_y * gInfo.ratio_gear_axis_y)/(gInfo.stage_mm_per_rev_axis_y * gInfo.ratio_shaft_axis_y));
		acc_y = acc * ((gInfo.enc_pulse_per_rev_axis_y * gInfo.ratio_gear_axis_y)/(gInfo.stage_mm_per_rev_axis_y * gInfo.ratio_shaft_axis_y));
		dec_y = dec * ((gInfo.enc_pulse_per_rev_axis_y * gInfo.ratio_gear_axis_y)/(gInfo.stage_mm_per_rev_axis_y * gInfo.ratio_shaft_axis_y));
		setPosParametersMsg(&queueModbus, axisToId("y"));
		setPositionMsg(&queueModbus, axisToId("y"), (int32_t)position_y, (int32_t)speed_y, (int32_t)acc_y, (int32_t)dec_y);
		
		position = M_TO_MM(req.z);
		position_z = position * ((gInfo.enc_pulse_per_rev_axis_z * gInfo.ratio_gear_axis_z)/(gInfo.stage_mm_per_rev_axis_z * gInfo.ratio_shaft_axis_z));
		speed_z = speed * ((gInfo.enc_pulse_per_rev_axis_z * gInfo.ratio_gear_axis_z)/(gInfo.stage_mm_per_rev_axis_z * gInfo.ratio_shaft_axis_z));
		acc_z = acc * ((gInfo.enc_pulse_per_rev_axis_z * gInfo.ratio_gear_axis_z)/(gInfo.stage_mm_per_rev_axis_z * gInfo.ratio_shaft_axis_z));
		dec_z = dec * ((gInfo.enc_pulse_per_rev_axis_z * gInfo.ratio_gear_axis_z)/(gInfo.stage_mm_per_rev_axis_z * gInfo.ratio_shaft_axis_z));
		setPosParametersMsg(&queueModbus, axisToId("z"));
		setPositionMsg(&queueModbus, axisToId("z"), (int32_t)position_z, (int32_t)speed_z, (int32_t)acc_z, (int32_t)dec_z);
	}

	setAxisCommandMsg(&queueModbus, axisToId("x"), AxisCommand::start, OnOff::on);
	setAxisCommandMsg(&queueModbus, axisToId("y"), AxisCommand::start, OnOff::on);
	setAxisCommandMsg(&queueModbus, axisToId("z"), AxisCommand::start, OnOff::on);

	setAxisCommandMsg(&queueModbus, axisToId("x"), AxisCommand::start, OnOff::off);
	setAxisCommandMsg(&queueModbus, axisToId("y"), AxisCommand::start, OnOff::off);
	setAxisCommandMsg(&queueModbus, axisToId("z"), AxisCommand::start, OnOff::off);

	res.success = SRV_SUCCESS;

	reprintf(ScreenOutput::ALWAYS, "[%s{%s}(%d)]\n", __FILENAME__, __FUNCTION__, __LINE__);

    return true;
}

bool serviceCommandCallback(gantry_robot::Command::Request &req, gantry_robot::Command::Response &res) {
    ros::Time time = ros::Time::now();

	if (req.command == (int32_t)CommandState::INIT) {
		res.success = SRV_SUCCESS;

		reprintf(ScreenOutput::ALWAYS, "[%s{%s}(%d)]\n", __FILENAME__, __FUNCTION__, __LINE__);
	} else {
		res.success = SRV_FAIL;

		reprintf(ScreenOutput::ERROR, "[%s{%s}(%d)]\n", __FILENAME__, __FUNCTION__, __LINE__);
	}


    return true;
}

bool serviceHomingCallback(gantry_robot::Homing::Request &req, gantry_robot::Homing::Response &res) {
    ros::Time time = ros::Time::now();

	std::string axis = req.axis;
	std::transform(axis.begin(), axis.end(), axis.begin(), ::tolower);

	if (axis == "x" || axis == "y" || axis == "z") {
		setAxisCommandMsg(&queueModbus, axisToId(axis), AxisCommand::emg, OnOff::off);
		setAxisCommandMsg(&queueModbus, axisToId(axis), AxisCommand::a_rst, OnOff::on);
		setAxisCommandMsg(&queueModbus, axisToId(axis), AxisCommand::a_rst, OnOff::off);
		setAxisCommandMsg(&queueModbus, axisToId(axis), AxisCommand::stop, OnOff::off);
		setAxisCommandMsg(&queueModbus, axisToId(axis), AxisCommand::sv_on, OnOff::on);

		if (gInfo.axis_x_num == axisToId(axis)) {
			if (info.axisX.status.output.org == (uint8_t)OnOff::on && req.setForce || !info.axisX.status.output.org) {
				setHomingParametersMsg(&queueModbus, axisToId(axis), gInfo.homing_speed_x_val, gInfo.homing_offset_x_val, gInfo.homing_done_behaviour_x_val);
				setAxisCommandMsg(&queueModbus, axisToId(axis), AxisCommand::hstart, OnOff::on);
				setAxisCommandMsg(&queueModbus, axisToId(axis), AxisCommand::hstart, OnOff::off);
			}
		} else if (gInfo.axis_y_num == axisToId(axis)) {
			if (info.axisY.status.output.org == (uint8_t)OnOff::on && req.setForce || !info.axisY.status.output.org) {
				setHomingParametersMsg(&queueModbus, axisToId(axis), gInfo.homing_speed_y_val, gInfo.homing_offset_y_val, gInfo.homing_done_behaviour_y_val);
				setAxisCommandMsg(&queueModbus, axisToId(axis), AxisCommand::hstart, OnOff::on);
				setAxisCommandMsg(&queueModbus, axisToId(axis), AxisCommand::hstart, OnOff::off);
			}
		} else if (gInfo.axis_z_num == axisToId(axis)) {
			if (info.axisZ.status.output.org == (uint8_t)OnOff::on && req.setForce || !info.axisZ.status.output.org) {
				setHomingParametersMsg(&queueModbus, axisToId(axis), gInfo.homing_speed_z_val, gInfo.homing_offset_z_val, gInfo.homing_done_behaviour_z_val);
				setAxisCommandMsg(&queueModbus, axisToId(axis), AxisCommand::hstart, OnOff::on);
				setAxisCommandMsg(&queueModbus, axisToId(axis), AxisCommand::hstart, OnOff::off);
			}
		} else {
		}

		res.success = SRV_SUCCESS;

		reprintf(ScreenOutput::ALWAYS, "[%s{%s}(%d)]\n", __FILENAME__, __FUNCTION__, __LINE__);
	} else {
		res.success = SRV_FAIL;
		reprintf(ScreenOutput::ERROR, "[%s{%s}(%d)]\n", __FILENAME__, __FUNCTION__, __LINE__);
	}

    return true;
}

#ifdef YAPPER_ENABLE
void yapLocalCallBack(const yapper::YapIn yapIn) {
	static yapper::YapIn yapInPre;

	if (yapInPre.jogInfo.x_p != yapIn.jogInfo.x_p)
	if (yapIn.jogInfo.x_p) {
		reprintf(ScreenOutput::DEFAULT, "[%s{%s}(%d)] : yapIn.jogInfo.x_p pushed\n", __FILENAME__, __FUNCTION__, __LINE__);
		setAxisCommandMsg(&queueModbus, gInfo.axis_x_num, AxisCommand::jdir, OnOff::on);
		setAxisCommandMsg(&queueModbus, gInfo.axis_x_num, AxisCommand::jstart, OnOff::on);
	} else {
		reprintf(ScreenOutput::DEFAULT, "[%s{%s}(%d)] : yapIn.jogInfo.x_p released\n", __FILENAME__, __FUNCTION__, __LINE__);
		setAxisCommandMsg(&queueModbus, gInfo.axis_x_num, AxisCommand::jdir, OnOff::on);
		setAxisCommandMsg(&queueModbus, gInfo.axis_x_num, AxisCommand::jstart, OnOff::off);
	}
	if (yapInPre.jogInfo.x_n != yapIn.jogInfo.x_n)
	if (yapIn.jogInfo.x_n) {
		reprintf(ScreenOutput::DEFAULT, "[%s{%s}(%d)] : yapIn.jogInfo.x_n pushed\n", __FILENAME__, __FUNCTION__, __LINE__);
		setAxisCommandMsg(&queueModbus, gInfo.axis_x_num, AxisCommand::jdir, OnOff::off);
		setAxisCommandMsg(&queueModbus, gInfo.axis_x_num, AxisCommand::jstart, OnOff::on);
	} else {
		reprintf(ScreenOutput::DEFAULT, "[%s{%s}(%d)] : yapIn.jogInfo.x_n released\n", __FILENAME__, __FUNCTION__, __LINE__);
		setAxisCommandMsg(&queueModbus, gInfo.axis_x_num, AxisCommand::jdir, OnOff::off);
		setAxisCommandMsg(&queueModbus, gInfo.axis_x_num, AxisCommand::jstart, OnOff::off);
	}
	if (yapInPre.jogInfo.y_p != yapIn.jogInfo.y_p)
	if (yapIn.jogInfo.y_p) {
		reprintf(ScreenOutput::DEFAULT, "[%s{%s}(%d)] : yapIn.jogInfo.y_p pushed\n", __FILENAME__, __FUNCTION__, __LINE__);
		setAxisCommandMsg(&queueModbus, gInfo.axis_y_num, AxisCommand::jdir, OnOff::on);
		setAxisCommandMsg(&queueModbus, gInfo.axis_y_num, AxisCommand::jstart, OnOff::on);
	} else {
		reprintf(ScreenOutput::DEFAULT, "[%s{%s}(%d)] : yapIn.jogInfo.y_p released\n", __FILENAME__, __FUNCTION__, __LINE__);
		setAxisCommandMsg(&queueModbus, gInfo.axis_y_num, AxisCommand::jdir, OnOff::on);
		setAxisCommandMsg(&queueModbus, gInfo.axis_y_num, AxisCommand::jstart, OnOff::off);
	}
	if (yapInPre.jogInfo.y_n != yapIn.jogInfo.y_n)
	if (yapIn.jogInfo.y_n) {
		reprintf(ScreenOutput::DEFAULT, "[%s{%s}(%d)] : yapIn.jogInfo.y_n pushed\n", __FILENAME__, __FUNCTION__, __LINE__);
		setAxisCommandMsg(&queueModbus, gInfo.axis_y_num, AxisCommand::jdir, OnOff::off);
		setAxisCommandMsg(&queueModbus, gInfo.axis_y_num, AxisCommand::jstart, OnOff::on);
	} else {
		reprintf(ScreenOutput::DEFAULT, "[%s{%s}(%d)] : yapIn.jogInfo.y_n released\n", __FILENAME__, __FUNCTION__, __LINE__);
		setAxisCommandMsg(&queueModbus, gInfo.axis_y_num, AxisCommand::jdir, OnOff::off);
		setAxisCommandMsg(&queueModbus, gInfo.axis_y_num, AxisCommand::jstart, OnOff::off);
	}
	if (yapInPre.jogInfo.z_p != yapIn.jogInfo.z_p)
	if (yapIn.jogInfo.z_p) {
		reprintf(ScreenOutput::DEFAULT, "[%s{%s}(%d)] : yapIn.jogInfo.z_p pushed\n", __FILENAME__, __FUNCTION__, __LINE__);
		setAxisCommandMsg(&queueModbus, gInfo.axis_z_num, AxisCommand::jdir, OnOff::on);
		setAxisCommandMsg(&queueModbus, gInfo.axis_z_num, AxisCommand::jstart, OnOff::on);
	} else {
		reprintf(ScreenOutput::DEFAULT, "[%s{%s}(%d)] : yapIn.jogInfo.z_p released\n", __FILENAME__, __FUNCTION__, __LINE__);
		setAxisCommandMsg(&queueModbus, gInfo.axis_z_num, AxisCommand::jdir, OnOff::on);
		setAxisCommandMsg(&queueModbus, gInfo.axis_z_num, AxisCommand::jstart, OnOff::off);
	}
	if (yapInPre.jogInfo.z_n != yapIn.jogInfo.z_n)
	if (yapIn.jogInfo.z_n) {
		reprintf(ScreenOutput::DEFAULT, "[%s{%s}(%d)] : yapIn.jogInfo.z_n pushed\n", __FILENAME__, __FUNCTION__, __LINE__);
		setAxisCommandMsg(&queueModbus, gInfo.axis_z_num, AxisCommand::jdir, OnOff::off);
		setAxisCommandMsg(&queueModbus, gInfo.axis_z_num, AxisCommand::jstart, OnOff::on);
	} else {
		reprintf(ScreenOutput::DEFAULT, "[%s{%s}(%d)] : yapIn.jogInfo.z_n released\n", __FILENAME__, __FUNCTION__, __LINE__);
		setAxisCommandMsg(&queueModbus, gInfo.axis_z_num, AxisCommand::jdir, OnOff::off);
		setAxisCommandMsg(&queueModbus, gInfo.axis_z_num, AxisCommand::jstart, OnOff::off);
	}
	yapInPre = yapIn;
    # if 0
#define STEP_TIME 1
    static double time_cur = ros::Time::now().toSec();
    static double time_pre = time_cur;

    time_cur = ros::Time::now().toSec();
    if (time_cur - time_pre > STEP_TIME) {
        time_pre = time_cur;
    }
    #endif

    #if TICK_LOG
    static double time_cur = ros::Time::now().toSec();
    static double time_pre = time_cur;
    static uint32_t count = 0;

    time_cur = ros::Time::now().toSec();

    printf("%s count: %05d, time_diff(ms): %lf, ts: %lf\n", __FUNCTION__, count++, time_cur-time_pre, time_cur);
    time_pre = time_cur;
    #endif
}
#endif

void modbusLoop(int rate, std::queue<AxisMsg>* queueModbus, ModbusLoopState* modbusLoopState, ros::Publisher* pub_info, modbus_t* ctx) {
    int size;
	ros::Time ts_now;
	AxisMsg axisMsg;

	ros::Rate r(rate);

	while (ros::ok() && *modbusLoopState!=ModbusLoopState::FINISH)
	{
		// 3축 모두 모니터링(position, speed, io)할 경우 6.96 Hz
		// 3축 position, io할 경우 10.43 Hz
		// 3축 position할 경우 20.87 Hz
		// 3축 speed 경우 20.87 Hz
		// 3축 io 경우 20.87 Hz
		// 타임아웃관련 문제일 수 있음
        size = queueModbus->size();

        if (size) {
			axisMsg = queueModbus->front();
			switch(axisMsg.type) {
				case CommandType::setCommand:
					reprintf(ScreenOutput::NO, "[%s{%s}(%d)] : CommandType::setCommand\n", __FILENAME__, __FUNCTION__, __LINE__);
					if (!setAxisCommand(ctx, axisMsg.id, axisMsg.axisCommand, axisMsg.onOff)) {
						reprintf(ScreenOutput::ERROR, "[%s{%s}(%d)] : setAxisCommand error\n", __FILENAME__, __FUNCTION__, __LINE__);
						gInfo.isError = ERR_SET;
						gInfo.errorMessage = "setAxisCommand error";
					} else {
						queueModbus->pop();
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
						gInfo.isError = ERR_SET;
						gInfo.errorMessage = "setHomingParameters error";
					} else {
						queueModbus->pop();
					}
				break;
				case CommandType::setJogParameters:
					reprintf(ScreenOutput::NO, "[%s{%s}(%d)] : CommandType::setJogParameters\n", __FILENAME__, __FUNCTION__, __LINE__);
					if (!setJogParameters(ctx, axisMsg.id, axisMsg.speed, axisMsg.acc, axisMsg.dec, axisMsg.s_curve, axisMsg.servo_lock)) {
						reprintf(ScreenOutput::ERROR, "[%s{%s}(%d)] : setJogParameters error\n", __FILENAME__, __FUNCTION__, __LINE__);
						gInfo.isError = ERR_SET;
						gInfo.errorMessage = "setJogParameters error";
					} else {
						queueModbus->pop();
					}
				break;
				case CommandType::setPosParameters:
					reprintf(ScreenOutput::NO, "[%s{%s}(%d)] : CommandType::setPosParameters\n", __FILENAME__, __FUNCTION__, __LINE__);
					if (!setPosParameters(ctx, axisMsg.id)) {
						reprintf(ScreenOutput::ERROR, "[%s{%s}(%d)] : setPosParameters error\n", __FILENAME__, __FUNCTION__, __LINE__);
						gInfo.isError = ERR_SET;
						gInfo.errorMessage = "setPosParameters error";
					} else {
						queueModbus->pop();
					}
				break;
				case CommandType::setPosition:
					reprintf(ScreenOutput::DEFAULT, "[%s{%s}(%d)] : CommandType::setPosition\n", __FILENAME__, __FUNCTION__, __LINE__);
					if (!setPosition(ctx, axisMsg.id, axisMsg.position, axisMsg.speed, axisMsg.acc, axisMsg.dec)) {
						reprintf(ScreenOutput::ERROR, "[%s{%s}(%d)] : setPosition error\n", __FILENAME__, __FUNCTION__, __LINE__);
						gInfo.isError = ERR_SET;
						gInfo.errorMessage = "setPosition error";
					} else {
						queueModbus->pop();
					}
				break;
				default:
				break;
			}
		} else {
		}

		if (!getAxisStatus(ctx, gInfo.axis_x_num, &info.axisX)) {
			reprintf(ScreenOutput::ERROR, "getAxisStatus AXIS_X error\n");
			gInfo.isError = ERR_SET;
			gInfo.errorMessage = "getAxisStatus AXIS_X error";
		}
		if (!getAxisParameter(ctx, gInfo.axis_x_num, ACT_POS, &info.axisX.position)) {
			reprintf(ScreenOutput::ERROR, "getAxisParameter AXIS_X error\n");
			gInfo.isError = ERR_SET;
			gInfo.errorMessage = "getAxisParameter AXIS_X error";
		} else {
			info.axisX.location = encToUU(gInfo.axis_x_num, info.axisX.position);
		}
		// if (!getAxisParameter(ctx, gInfo.axis_x_num, ACT_SPD, &info.axisX.speed)) {
		// 	reprintf(ScreenOutput::ERROR, "getAxisParameter AXIS_X error\n");
		// 	gInfo.isError = ERR_SET;
		// 	gInfo.errorMessage = "getAxisParameter AXIS_X error";
		// }

		if (!getAxisStatus(ctx, gInfo.axis_y_num, &info.axisY)) {
			reprintf(ScreenOutput::ERROR, "getAxisStatus AXIS_Y error\n");
			gInfo.isError = ERR_SET;
			gInfo.errorMessage = "getAxisStatus AXIS_Y error";
		}
		if (!getAxisParameter(ctx, gInfo.axis_y_num, ACT_POS, &info.axisY.position)) {
			reprintf(ScreenOutput::ERROR, "getAxisParameter AXIS_Y error\n");
			gInfo.isError = ERR_SET;
			gInfo.errorMessage = "getAxisParameter AXIS_Y error";
		} else {
			info.axisY.location = encToUU(gInfo.axis_y_num, info.axisY.position);
		}
		// if (!getAxisParameter(ctx, gInfo.axis_y_num, ACT_SPD, &info.axisY.speed)) {
		// 	reprintf(ScreenOutput::ERROR, "getAxisParameter AXIS_Y error\n");
		// 	gInfo.isError = ERR_SET;
		// 	gInfo.errorMessage = "getAxisParameter AXIS_Y error";
		// }

		if (!getAxisStatus(ctx, gInfo.axis_z_num, &info.axisZ)) {
			reprintf(ScreenOutput::ERROR, "getAxisStatus AXIS_Z error\n");
			gInfo.isError = ERR_SET;
			gInfo.errorMessage = "getAxisStatus AXIS_Z error";
		}
		if (!getAxisParameter(ctx, gInfo.axis_z_num, ACT_POS, &info.axisZ.position)) {
			reprintf(ScreenOutput::ERROR, "getAxisParameter AXIS_Z error\n");
			gInfo.isError = ERR_SET;
			gInfo.errorMessage = "getAxisParameter AXIS_Z error";
		} else {
			info.axisZ.location = encToUU(gInfo.axis_z_num, info.axisZ.position);
		}
		// if (!getAxisParameter(ctx, gInfo.axis_z_num, ACT_SPD, &info.axisZ.speed)) {
		// 	reprintf(ScreenOutput::ERROR, "getAxisParameter AXIS_Z error\n");
		// 	gInfo.isError = ERR_SET;
		// 	gInfo.errorMessage = "getAxisParameter AXIS_Z error";
		// }

		ts_now = ros::Time::now();
        info.header.stamp = ts_now;
		setState();
		info.error.isError = gInfo.isError;
		info.error.errorMessage = gInfo.errorMessage;
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
	// test ros::param::get vs nh.getParam
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
    nh.getParam("enc_pulse_per_rev_axis_x", gInfo.enc_pulse_per_rev_axis_x);
    nh.getParam("enc_pulse_per_rev_axis_y", gInfo.enc_pulse_per_rev_axis_y);
    nh.getParam("enc_pulse_per_rev_axis_z", gInfo.enc_pulse_per_rev_axis_z);
    nh.getParam("stage_max_mm_axis_x", gInfo.stage_max_mm_axis_x);
    nh.getParam("stage_max_mm_axis_y", gInfo.stage_max_mm_axis_y);
    nh.getParam("stage_max_mm_axis_z", gInfo.stage_max_mm_axis_z);
    nh.getParam("stage_mm_per_rev_axis_x", gInfo.stage_mm_per_rev_axis_x);
    nh.getParam("stage_mm_per_rev_axis_y", gInfo.stage_mm_per_rev_axis_y);
    nh.getParam("stage_mm_per_rev_axis_z", gInfo.stage_mm_per_rev_axis_z);
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
    nh.getParam("pos_speed_val", gInfo.pos_speed_val);
    nh.getParam("pos_acc_dec_val", gInfo.pos_acc_dec_val);
#endif

	// CommandState cmdState = CommandState::INIT;
	CommandState cmdState = CommandState::HOME;
	CommandState cmdStatePre = cmdState;
	FunctionState funcState = FunctionState::INIT;
	InitState initState = InitState::HOME_SET;

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
		reprintf(ScreenOutput::DEFAULT, "[%s{%s}(%d)] : serial port : %s, baudrate : %d open success\n",
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

	// modbus_test_start 20210827_2007
	// struct timeval byte_timeout;
	// modbus_get_byte_timeout(ctx, &byte_timeout);
	// printf("modbus_get_byte_timeout: %ld, %ld\n", byte_timeout.tv_sec, byte_timeout.tv_usec);
	// modbus_get_response_timeout(ctx, &byte_timeout);
	// printf("modbus_get_byte_timeout: %ld, %ld\n", byte_timeout.tv_sec, byte_timeout.tv_usec);
	// modbus_test_end

    ros::ServiceServer service_position = nh.advertiseService("gantry_robot_position", servicePositionCallback);
    ros::ServiceServer service_homing = nh.advertiseService("gantry_robot_homing", serviceHomingCallback);
    ros::ServiceServer service_command = nh.advertiseService("gantry_robot_command", serviceCommandCallback);
    ros::ServiceServer service_location = nh.advertiseService("gantry_robot_location", serviceLocationCallback);

#ifdef YAPPER_ENABLE
    ros::Subscriber yapper_local_sub = nh.subscribe("/yapper_local/yapIn_topic", 100, yapLocalCallBack);
#endif

    ros::Publisher pub_info = nh.advertise<gantry_robot::Info>("gantry_robot_info", 100);

    int main_hz = 1000;
	AxisMsg axisMsg;
    boost::thread threadModbusLoop(modbusLoop, main_hz, &queueModbus, &modbusLoopState, &pub_info, ctx);

	#define STEP_TIME 1.0
	double ts_run;
	double ts_cur;
	double ts_pre;
	double ts_diff;
	double ts_elap;

	ros::Rate r(1000);

	ts_run = ros::Time::now().toSec();
	ts_pre = ts_cur = ts_run;
	ts_diff;

	while (ros::ok()) {
		ts_cur = ros::Time::now().toSec();
		ts_diff = ts_cur - ts_pre;
		if ( ts_diff > STEP_TIME ) {
			ts_pre = ts_cur;
		}

		if (cmdState != cmdStatePre) {
			funcState = FunctionState::INIT;
		}
		cmdStatePre = cmdState;
		reprintf(ScreenOutput::NO, "[%s{%s}(%d)] : CommandState::%d FunctionState::%d\n", __FILENAME__, __FUNCTION__, __LINE__, (int32_t)cmdState, (int32_t)funcState);

		switch (cmdState) {
			case CommandState::INIT:
				switch (funcState) {
					case FunctionState::INIT:
						ts_run = ros::Time::now().toSec();

						funcState = FunctionState::SET;
					break;
					case FunctionState::SET:
						ts_elap = ts_cur - ts_run;
						if (modbusLoopState == ModbusLoopState::RUNNING) {
							reprintf(ScreenOutput::DEFAULT, "[%s{%s}(%d)] : CommandState::INIT FunctionState::SET success\n", __FILENAME__, __FUNCTION__, __LINE__);
							funcState = FunctionState::ACTION;
							initState = InitState::HOME_SET;
						} else if (ts_elap > TS_ELAPSE_ERROR){
							reprintf(ScreenOutput::DEFAULT, "[%s{%s}(%d)] : CommandState::INIT FunctionState::SET fail\n", __FILENAME__, __FUNCTION__, __LINE__);
							cmdState = CommandState::ERROR;
						}
					break;
					case FunctionState::ACTION:
						switch (initState) {
							case InitState::HOME_SET:
								setAxisCommandMsg(&queueModbus, gInfo.axis_x_num, AxisCommand::emg, OnOff::off);
								setAxisCommandMsg(&queueModbus, gInfo.axis_y_num, AxisCommand::emg, OnOff::off);
								setAxisCommandMsg(&queueModbus, gInfo.axis_z_num, AxisCommand::emg, OnOff::off);

								setAxisCommandMsg(&queueModbus, gInfo.axis_x_num, AxisCommand::a_rst, OnOff::on);
								setAxisCommandMsg(&queueModbus, gInfo.axis_y_num, AxisCommand::a_rst, OnOff::on);
								setAxisCommandMsg(&queueModbus, gInfo.axis_z_num, AxisCommand::a_rst, OnOff::on);

								setAxisCommandMsg(&queueModbus, gInfo.axis_x_num, AxisCommand::a_rst, OnOff::off);
								setAxisCommandMsg(&queueModbus, gInfo.axis_y_num, AxisCommand::a_rst, OnOff::off);
								setAxisCommandMsg(&queueModbus, gInfo.axis_z_num, AxisCommand::a_rst, OnOff::off);

								setAxisCommandMsg(&queueModbus, gInfo.axis_x_num, AxisCommand::stop, OnOff::off);
								setAxisCommandMsg(&queueModbus, gInfo.axis_y_num, AxisCommand::stop, OnOff::off);
								setAxisCommandMsg(&queueModbus, gInfo.axis_z_num, AxisCommand::stop, OnOff::off);

								setAxisCommandMsg(&queueModbus, gInfo.axis_x_num, AxisCommand::sv_on, OnOff::on);
								setAxisCommandMsg(&queueModbus, gInfo.axis_y_num, AxisCommand::sv_on, OnOff::on);
								setAxisCommandMsg(&queueModbus, gInfo.axis_z_num, AxisCommand::sv_on, OnOff::on);

								reprintf(ScreenOutput::DEFAULT, "[%s{%s}(%d)] : CommandState::INIT FunctionState::ACTION InitState::HOME_SET success\n",
									__FILENAME__, __FUNCTION__, __LINE__);
								initState = InitState::HOME_ACTION;
							break;
							case InitState::HOME_ACTION:
								setHomingParametersMsg(&queueModbus, gInfo.axis_x_num, gInfo.homing_speed_x_val, gInfo.homing_offset_x_val, gInfo.homing_done_behaviour_x_val);
								setHomingParametersMsg(&queueModbus, gInfo.axis_y_num, gInfo.homing_speed_y_val, gInfo.homing_offset_y_val, gInfo.homing_done_behaviour_y_val);
								setHomingParametersMsg(&queueModbus, gInfo.axis_z_num, gInfo.homing_speed_z_val, gInfo.homing_offset_z_val, gInfo.homing_done_behaviour_z_val);

								reprintf(ScreenOutput::DEFAULT, "[%s{%s}(%d)] : CommandState::INIT FunctionState::ACTION InitState::HOME_ACTION success\n",
									__FILENAME__, __FUNCTION__, __LINE__);
								initState = InitState::POSITION_SET;
							break;
							case InitState::POSITION_SET:
								setPosParametersMsg(&queueModbus, gInfo.axis_x_num);
								setPosParametersMsg(&queueModbus, gInfo.axis_y_num);
								setPosParametersMsg(&queueModbus, gInfo.axis_z_num);

								reprintf(ScreenOutput::DEFAULT, "[%s{%s}(%d)] : CommandState::INIT FunctionState::ACTION InitState::POSITION_SET success\n",
									__FILENAME__, __FUNCTION__, __LINE__);
								initState = InitState::POSITION_ACTION;
							break;
							case InitState::POSITION_ACTION:
								initState = InitState::JOG_SET;
							break;
							case InitState::JOG_SET:
								setJogParametersMsg(&queueModbus, gInfo.axis_z_num, gInfo.jog_min_speed_val, gInfo.jog_acceleration_val, gInfo.jog_deceleration_val, gInfo.jog_s_curve_val, OnOff::off);

								reprintf(ScreenOutput::DEFAULT, "[%s{%s}(%d)] : CommandState::INIT FunctionState::ACTION InitState::JOG_SET success\n",
									__FILENAME__, __FUNCTION__, __LINE__);
								initState = InitState::JOG_ACTION;
							break;
							case InitState::JOG_ACTION:
								initState = InitState::IDLE;
							break;
							case InitState::IDLE:
								funcState = FunctionState::DONE;
								initState = InitState::IDLE;
							break;
							default:
								cmdState = CommandState::ERROR;
							break;
						}
					break;
					case FunctionState::DONE:
						reprintf(ScreenOutput::DEFAULT, "[%s{%s}(%d)] : CommandState::INIT FunctionState::DONE success\n",
									__FILENAME__, __FUNCTION__, __LINE__);
						funcState = FunctionState::IDLE;
					break;
					case FunctionState::IDLE:
						cmdState = CommandState::IDLE;
						funcState = FunctionState::IDLE;
					break;
					case FunctionState::ERROR:
						cmdState = CommandState::ERROR;
					break;
				}
			break;
			case CommandState::HOME:
				switch (funcState) {
					case FunctionState::INIT:
						// 강제로 호밍할 경우 호밍이 없어지길 기다리기
						// 각 축마다 호밍할 경우 이외의 상태검사 제외하기
						// 에러가 발생하면 에러스테이트로 이동
						setAxisCommandMsg(&queueModbus, gInfo.axis_x_num, AxisCommand::emg, OnOff::off);
						setAxisCommandMsg(&queueModbus, gInfo.axis_y_num, AxisCommand::emg, OnOff::off);
						setAxisCommandMsg(&queueModbus, gInfo.axis_z_num, AxisCommand::emg, OnOff::off);

						setAxisCommandMsg(&queueModbus, gInfo.axis_x_num, AxisCommand::a_rst, OnOff::on);
						setAxisCommandMsg(&queueModbus, gInfo.axis_y_num, AxisCommand::a_rst, OnOff::on);
						setAxisCommandMsg(&queueModbus, gInfo.axis_z_num, AxisCommand::a_rst, OnOff::on);

						setAxisCommandMsg(&queueModbus, gInfo.axis_x_num, AxisCommand::a_rst, OnOff::off);
						setAxisCommandMsg(&queueModbus, gInfo.axis_y_num, AxisCommand::a_rst, OnOff::off);
						setAxisCommandMsg(&queueModbus, gInfo.axis_z_num, AxisCommand::a_rst, OnOff::off);

						setAxisCommandMsg(&queueModbus, gInfo.axis_x_num, AxisCommand::stop, OnOff::off);
						setAxisCommandMsg(&queueModbus, gInfo.axis_y_num, AxisCommand::stop, OnOff::off);
						setAxisCommandMsg(&queueModbus, gInfo.axis_z_num, AxisCommand::stop, OnOff::off);

						setAxisCommandMsg(&queueModbus, gInfo.axis_x_num, AxisCommand::sv_on, OnOff::on);
						setAxisCommandMsg(&queueModbus, gInfo.axis_y_num, AxisCommand::sv_on, OnOff::on);
						setAxisCommandMsg(&queueModbus, gInfo.axis_z_num, AxisCommand::sv_on, OnOff::on);
						reprintf(ScreenOutput::DEFAULT, "[%s{%s}(%d)] : CommandState::HOME FunctionState::INIT success\n", __FILENAME__, __FUNCTION__, __LINE__);
						funcState = FunctionState::SET;
					break;
					case FunctionState::SET:
						// 에러가 발생하면 에러스테이트로 이동
						setHomingParametersMsg(&queueModbus, gInfo.axis_x_num, gInfo.homing_speed_x_val, gInfo.homing_offset_x_val, gInfo.homing_done_behaviour_x_val);
						setHomingParametersMsg(&queueModbus, gInfo.axis_y_num, gInfo.homing_speed_y_val, gInfo.homing_offset_y_val, gInfo.homing_done_behaviour_y_val);
						setHomingParametersMsg(&queueModbus, gInfo.axis_z_num, gInfo.homing_speed_z_val, gInfo.homing_offset_z_val, gInfo.homing_done_behaviour_z_val);
						reprintf(ScreenOutput::DEFAULT, "[%s{%s}(%d)] : CommandState::HOME FunctionState::SET success\n", __FILENAME__, __FUNCTION__, __LINE__);
						funcState = FunctionState::ACTION;
					break;
					case FunctionState::ACTION:
						static double sec_now, sec_start;
#define ERROR_SEC	60
						// 에러가 발생하면 에러스테이트로 이동
						setAxisCommandMsg(&queueModbus, axisToId("z"), AxisCommand::hstart, OnOff::on);
						setAxisCommandMsg(&queueModbus, axisToId("z"), AxisCommand::hstart, OnOff::off);

						// 호밍이 완료되면 다음 절차로 아니면 시간초과일 경우 에러스테이트로 이동
						sec_start = sec_now = ros::Time::now().toSec();
						while(ros::ok()) {
							sec_now = ros::Time::now().toSec();
							if (sec_now - sec_start > ERROR_SEC) {
								reprintf(ScreenOutput::DEFAULT, "[%s{%s}(%d)] : CommandState::HOME FunctionState::ACTION homing Z failed\n", __FILENAME__, __FUNCTION__, __LINE__);
								funcState = FunctionState::ERROR;
								break;
							}
							if (info.axisZ.status.output.org) {
								reprintf(ScreenOutput::DEFAULT, "[%s{%s}(%d)] : CommandState::HOME FunctionState::ACTION homing Z success\n", __FILENAME__, __FUNCTION__, __LINE__);
								break;
							}
							ros::spinOnce();
							r.sleep();
						}

						// 에러가 발생하면 에러스테이트로 이동
						setAxisCommandMsg(&queueModbus, axisToId("x"), AxisCommand::hstart, OnOff::on);
						setAxisCommandMsg(&queueModbus, axisToId("y"), AxisCommand::hstart, OnOff::on);
						setAxisCommandMsg(&queueModbus, axisToId("x"), AxisCommand::hstart, OnOff::off);
						setAxisCommandMsg(&queueModbus, axisToId("y"), AxisCommand::hstart, OnOff::off);

						// 호밍이 완료되면 다음 절차로 아니면 시간초과일 경우 에러스테이트로 이동
						sec_start = sec_now = ros::Time::now().toSec();
						while(ros::ok()) {
							sec_now = ros::Time::now().toSec();
							if (sec_now - sec_start > ERROR_SEC) {
								reprintf(ScreenOutput::DEFAULT, "[%s{%s}(%d)] : CommandState::HOME FunctionState::ACTION homing XY failed(%lf)\n", __FILENAME__, __FUNCTION__, __LINE__, sec_now - sec_start);
								funcState = FunctionState::ERROR;
								break;
							}
							if (info.axisX.status.output.org && info.axisY.status.output.org) {
								reprintf(ScreenOutput::DEFAULT, "[%s{%s}(%d)] : CommandState::HOME FunctionState::ACTION homing XY success\n", __FILENAME__, __FUNCTION__, __LINE__);
								break;
							}
							ros::spinOnce();
							r.sleep();
						}

						if (info.axisX.status.output.org && info.axisY.status.output.org && info.axisZ.status.output.org) {
							funcState = FunctionState::DONE;
						}
					break;
					case FunctionState::DONE:
						reprintf(ScreenOutput::DEFAULT, "[%s{%s}(%d)] : CommandState::HOME FunctionState::DONE homing done\n", __FILENAME__, __FUNCTION__, __LINE__);
						funcState = FunctionState::IDLE;
					break;
					case FunctionState::IDLE:
						funcState = FunctionState::IDLE;
					break;
					case FunctionState::ERROR:
						cmdState = CommandState::ERROR;
					break;
				}
			break;
			case CommandState::POSITION:
				switch (funcState) {
					case FunctionState::INIT:
						funcState = FunctionState::SET;
					break;
					case FunctionState::SET:
						funcState = FunctionState::ACTION;
					break;
					case FunctionState::ACTION:
						funcState = FunctionState::DONE;
					break;
					case FunctionState::DONE:
						funcState = FunctionState::IDLE;
					break;
					case FunctionState::IDLE:
						funcState = FunctionState::IDLE;
					break;
					case FunctionState::ERROR:
						cmdState = CommandState::ERROR;
					break;
				}
			break;
			case CommandState::JOG:
				switch (funcState) {
					case FunctionState::INIT:
							funcState = FunctionState::SET;
					break;
					case FunctionState::SET:
						funcState = FunctionState::ACTION;
					break;
					case FunctionState::ACTION:
						funcState = FunctionState::DONE;
					break;
					case FunctionState::DONE:
						funcState = FunctionState::IDLE;
					break;
					case FunctionState::IDLE:
						funcState = FunctionState::IDLE;
					break;
					case FunctionState::ERROR:
						cmdState = CommandState::ERROR;
						funcState = FunctionState::INIT;
					break;
				}
			break;
			case CommandState::STOP:
				switch (funcState) {
					case FunctionState::INIT:
						funcState = FunctionState::SET;
					break;
					case FunctionState::SET:
						funcState = FunctionState::ACTION;
					break;
					case FunctionState::ACTION:
						funcState = FunctionState::DONE;
					break;
					case FunctionState::DONE:
						funcState = FunctionState::IDLE;
					break;
					case FunctionState::IDLE:
						funcState = FunctionState::IDLE;
					break;
					case FunctionState::ERROR:
						cmdState = CommandState::ERROR;
					break;
				}
			break;
			case CommandState::IDLE:
				switch (funcState) {
					case FunctionState::INIT:
						funcState = FunctionState::SET;
					break;
					case FunctionState::SET:
						funcState = FunctionState::ACTION;
					break;
					case FunctionState::ACTION:
						funcState = FunctionState::DONE;
					break;
					case FunctionState::DONE:
						funcState = FunctionState::IDLE;
					break;
					case FunctionState::IDLE:
						funcState = FunctionState::IDLE;
					break;
					case FunctionState::ERROR:
						cmdState = CommandState::ERROR;
					break;
				}
			break;
			case CommandState::ERROR:
				switch (funcState) {
					case FunctionState::INIT:
						reprintf(ScreenOutput::DEFAULT, "[%s{%s}(%d)] : CommandState::ERROR FunctionState::INIT error occured\n",
									__FILENAME__, __FUNCTION__, __LINE__);
						funcState = FunctionState::SET;
					break;
					case FunctionState::SET:
						funcState = FunctionState::ACTION;
					break;
					case FunctionState::ACTION:
						funcState = FunctionState::IDLE;
					break;
					case FunctionState::DONE:
						funcState = FunctionState::IDLE;
					break;
					case FunctionState::IDLE:
						funcState = FunctionState::IDLE;
					break;
					case FunctionState::ERROR:
						cmdState = CommandState::ERROR;
					break;
				}
			break;
		}
		gInfo.cmdState = cmdState;
		gInfo.funcState = funcState;

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

void setState() {
	memset((uint8_t*)&info.state.cmdStateDetail, NULL_CHAR, sizeof(int32_t)*((int32_t)(CommandState::ERROR)+ENUM_MAX_VAL));
	info.state.cmdState = (int32_t)gInfo.cmdState;
	switch (gInfo.cmdState) {
		case CommandState::IDLE:
			info.state.cmdStateDetail.IDLE = VAL_SET;
		break;
		case CommandState::HOME:
			info.state.cmdStateDetail.HOME = VAL_SET;
		break;
		case CommandState::JOG:
			info.state.cmdStateDetail.JOG = VAL_SET;
		break;
		case CommandState::POSITION:
			info.state.cmdStateDetail.POSITION = VAL_SET;
		break;
		case CommandState::INIT:
			info.state.cmdStateDetail.INIT = VAL_SET;
		break;
		case CommandState::ERROR:
			info.state.cmdStateDetail.ERROR = VAL_SET;
		break;
		default:
		break;
	}
	memset((uint8_t*)&info.state.funcStateDetail, NULL_CHAR, sizeof(int32_t)*((int32_t)((int32_t)FunctionState::ERROR)+ENUM_MAX_VAL));
	info.state.funcState = (int32_t)gInfo.funcState;
	switch (gInfo.funcState) {
		case FunctionState::INIT:
			info.state.funcStateDetail.INIT = VAL_SET;
		break;
		case FunctionState::SET:
			info.state.funcStateDetail.SET = VAL_SET;
		break;
		case FunctionState::ACTION:
			info.state.funcStateDetail.ACTION = VAL_SET;
		break;
		case FunctionState::DONE:
			info.state.funcStateDetail.DONE = VAL_SET;
		break;
		case FunctionState::IDLE:
			info.state.funcStateDetail.IDLE = VAL_SET;
		break;
		case FunctionState::ERROR:
			info.state.funcStateDetail.ERROR = VAL_SET;
		break;
		default:
		break;
	}
}