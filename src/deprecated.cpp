#include "deprecated.h"
#include "main.h"
#include "L7P.h"

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
