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

bool servicePositionCallback(gantry_robot::Position::Request &req, gantry_robot::Position::Response &res) {
    ros::Time time = ros::Time::now();

	// setPosParametersMsg(&que, req.id);
	setPositionMsg(&que, req.id, req.position, req.speed, req.acc, req.dec);

	setAxisCommandMsg(&que, req.id, AxisCommand::start, OnOff::on);
	setAxisCommandMsg(&que, req.id, AxisCommand::start, OnOff::off);

	res.success = 1;
	
	reprintf(ScreenOutput::ALWAYS, "[%s{%s}(%d)]\n", __FILENAME__, __FUNCTION__, __LINE__);

    return true;
}

bool serviceHomingCallback(gantry_robot::Homing::Request &req, gantry_robot::Homing::Response &res) {
    ros::Time time = ros::Time::now();

	setAxisCommandMsg(&que, req.id, AxisCommand::emg, OnOff::off);
	setAxisCommandMsg(&que, req.id, AxisCommand::a_rst, OnOff::on);
	setAxisCommandMsg(&que, req.id, AxisCommand::a_rst, OnOff::off);
	setAxisCommandMsg(&que, req.id, AxisCommand::stop, OnOff::off);
	setAxisCommandMsg(&que, req.id, AxisCommand::sv_on, OnOff::on);

	setHomingParametersMsg(&que, req.id, req.speed, req.offset, req.done_behaviour?OnOff::on:OnOff::off);

	switch (req.id) {
		case AXIS_X_NUM:
			if (info.axisX.status.output.org == (uint8_t)OnOff::on) {
			} else {
				setAxisCommandMsg(&que, req.id, AxisCommand::hstart, OnOff::on);
				setAxisCommandMsg(&que, req.id, AxisCommand::hstart, OnOff::off);
			}
		break;
		case AXIS_Y_NUM:
			if (info.axisY.status.output.org == (uint8_t)OnOff::on) {
			} else {
				setAxisCommandMsg(&que, req.id, AxisCommand::hstart, OnOff::on);
				setAxisCommandMsg(&que, req.id, AxisCommand::hstart, OnOff::off);
			}
		break;
		case AXIS_Z_NUM:
			if (info.axisZ.status.output.org == (uint8_t)OnOff::on) {
			} else {
				setAxisCommandMsg(&que, req.id, AxisCommand::hstart, OnOff::on);
				setAxisCommandMsg(&que, req.id, AxisCommand::hstart, OnOff::off);
			}
		break;
		default:
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

		if (!getAxisStatus(ctx, AXIS_X_NUM, &info.axisX.status)) reprintf(ScreenOutput::ERROR, "getAxisStatus AXIS_X error\n");		
		if (!getAxisParameter(ctx, AXIS_X_NUM, ACT_POS, &info.axisX.position)) reprintf(ScreenOutput::ERROR, "getAxisParameter AXIS_X error\n");
		if (!getAxisParameter(ctx, AXIS_X_NUM, ACT_SPD, &info.axisX.speed)) reprintf(ScreenOutput::ERROR, "getAxisParameter AXIS_X error\n");

		if (!getAxisStatus(ctx, AXIS_Y_NUM, &info.axisY.status)) reprintf(ScreenOutput::ERROR, "getAxisStatus AXIS_Y error\n");		
		if (!getAxisParameter(ctx, AXIS_Y_NUM, ACT_POS, &info.axisY.position)) reprintf(ScreenOutput::ERROR, "getAxisParameter AXIS_Y error\n");
		if (!getAxisParameter(ctx, AXIS_Y_NUM, ACT_SPD, &info.axisY.speed)) reprintf(ScreenOutput::ERROR, "getAxisParameter AXIS_Y error\n");

		if (!getAxisStatus(ctx, AXIS_Z_NUM, &info.axisZ.status)) reprintf(ScreenOutput::ERROR, "getAxisStatus AXIS_Z error\n");		
		if (!getAxisParameter(ctx, AXIS_Z_NUM, ACT_POS, &info.axisZ.position)) reprintf(ScreenOutput::ERROR, "getAxisParameter AXIS_Z error\n");
		if (!getAxisParameter(ctx, AXIS_Z_NUM, ACT_SPD, &info.axisZ.speed)) reprintf(ScreenOutput::ERROR, "getAxisParameter AXIS_Z error\n");

		ts_now = ros::Time::now();
        info.header.stamp = ts_now;
        pub_info->publish(info);

		if (*modbusLoopState!=ModbusLoopState::FINISH) {
			*modbusLoopState = ModbusLoopState::RUNNING;
		}

		r.sleep();
	}

	setAxisCommand(ctx, AXIS_X_NUM, AxisCommand::stop, OnOff::on);
	setAxisCommand(ctx, AXIS_Y_NUM, AxisCommand::stop, OnOff::on);
	setAxisCommand(ctx, AXIS_Z_NUM, AxisCommand::stop, OnOff::on);
}

int main(int argc, char* argv[]) {
	ros::init(argc, argv, "gantry_robot");
	ros::NodeHandle nh("~");

	std::string serial_port;
	int baud_rate;
	int id = 0;
	ModbusLoopState modbusLoopState = ModbusLoopState::INIT;

#if 0
	ros::param::get("~serial_port", serial_port);
	ros::param::get("~baud_rate", baud_rate);
#else
	nh.getParam("serial_port", serial_port);
	nh.getParam("baud_rate", baud_rate);
#endif

	CommandCase cmdCase = CommandCase::INIT;
	CommandCase cmdCasePre = cmdCase;
	FunctionCase funcCase = FunctionCase::INIT;

	modbus_t* ctx;

	ctx = modbus_new_rtu(serial_port.c_str(), baud_rate, 'N', 8, 1);

	// modbus_new_rtu return
	// pointer : successful
	// NULL : error, set errno
	if (ctx == NULL) {
        reprintf(ScreenOutput::ERROR, "[%s{%s}(%d)] : Unable to allocate libmodbus context, error msg: %s \n",
			__FILENAME__, __FUNCTION__, __LINE__, modbus_strerror(errno));

		return -1;
	} else {
		reprintf(ScreenOutput::DEFAULT, "[%s{%s}(%d)] : serial port : %s, baudrate : %d\n",
			__FILENAME__, __FUNCTION__, __LINE__, serial_port.c_str(), baud_rate);
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
						setAxisCommandMsg(&que, AXIS_X_NUM, AxisCommand::emg, OnOff::off);
						setAxisCommandMsg(&que, AXIS_Y_NUM, AxisCommand::emg, OnOff::off);
						setAxisCommandMsg(&que, AXIS_Z_NUM, AxisCommand::emg, OnOff::off);
						setAxisCommandMsg(&que, AXIS_X_NUM, AxisCommand::a_rst, OnOff::on);
						setAxisCommandMsg(&que, AXIS_Y_NUM, AxisCommand::a_rst, OnOff::on);
						setAxisCommandMsg(&que, AXIS_Z_NUM, AxisCommand::a_rst, OnOff::on);
						setAxisCommandMsg(&que, AXIS_X_NUM, AxisCommand::a_rst, OnOff::off);
						setAxisCommandMsg(&que, AXIS_Y_NUM, AxisCommand::a_rst, OnOff::off);
						setAxisCommandMsg(&que, AXIS_Z_NUM, AxisCommand::a_rst, OnOff::off);
						setAxisCommandMsg(&que, AXIS_X_NUM, AxisCommand::stop, OnOff::off);
						setAxisCommandMsg(&que, AXIS_Y_NUM, AxisCommand::stop, OnOff::off);
						setAxisCommandMsg(&que, AXIS_Z_NUM, AxisCommand::stop, OnOff::off);
						setAxisCommandMsg(&que, AXIS_X_NUM, AxisCommand::sv_on, OnOff::on);
						setAxisCommandMsg(&que, AXIS_Y_NUM, AxisCommand::sv_on, OnOff::on);
						setAxisCommandMsg(&que, AXIS_Z_NUM, AxisCommand::sv_on, OnOff::on);

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
						setHomingParametersMsg(&que, AXIS_Z_NUM, HOMING_SPEED_Z_VAL, HOMING_OFFSET_Z_VAL, HOMING_DONE_BEHAVIOUR_Z_VAL);
						funcCase = FunctionCase::ACTION;
					break;
					case FunctionCase::ACTION:
						reprintf(ScreenOutput::DEFAULT, "[%s{%s}(%d)] : CommandCase::HOME FunctionCase::ACTION\n", __FILENAME__, __FUNCTION__, __LINE__);
						// Homing
						setAxisCommandMsg(&que, AXIS_Z_NUM, AxisCommand::hstart, OnOff::on);
						setAxisCommandMsg(&que, AXIS_Z_NUM, AxisCommand::hstart, OnOff::off);
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
						setPosParametersMsg(&que, AXIS_X_NUM);
						setPosParametersMsg(&que, AXIS_Y_NUM);
						setPosParametersMsg(&que, AXIS_Z_NUM);

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
						setJogParametersMsg(&que, AXIS_Z_NUM, JOG_MIN_SPEED_VAL, JOG_ACCELERATION_VAL, JOG_DECELERATION_VAL, JOG_S_CURVE_VAL, OnOff::off);
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