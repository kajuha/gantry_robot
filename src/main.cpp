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

#include "L7P.h"
#include "ObjectDictionary.h"
#include "main.h"

gantry_robot::Info info;

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
					reprintf(ScreenOutput::DEFAULT, "[%s{%s}(%d)] : CommandType::setCommand\n", __FILENAME__, __FUNCTION__, __LINE__);
					if (!setAxisCommand(ctx, axisMsg.id, axisMsg.axisCommand, axisMsg.onOff)) {
						reprintf(ScreenOutput::ERROR, "[%s{%s}(%d)] : setAxisCommand error\n", __FILENAME__, __FUNCTION__, __LINE__);
					} else {
						que->pop();
					}
				break;
				case CommandType::getStatus:
					reprintf(ScreenOutput::DEFAULT, "[%s{%s}(%d)] : CommandType::getStatus\n", __FILENAME__, __FUNCTION__, __LINE__);
				break;
				case CommandType::setParameter:
					reprintf(ScreenOutput::DEFAULT, "[%s{%s}(%d)] : CommandType::setParameter\n", __FILENAME__, __FUNCTION__, __LINE__);
				break;
				case CommandType::getParameter:
					reprintf(ScreenOutput::DEFAULT, "[%s{%s}(%d)] : CommandType::getParameter\n", __FILENAME__, __FUNCTION__, __LINE__);
				break;
				case CommandType::setHomingParameters:
					reprintf(ScreenOutput::DEFAULT, "[%s{%s}(%d)] : CommandType::setHomingParameters\n", __FILENAME__, __FUNCTION__, __LINE__);
					if (!setHomingParameters(ctx, axisMsg.id, axisMsg.speed, axisMsg.offset, axisMsg.done_behaviour)) {
						reprintf(ScreenOutput::ERROR, "[%s{%s}(%d)] : setHomingParameters error\n", __FILENAME__, __FUNCTION__, __LINE__);
					} else {
						que->pop();
					}
				break;
				default:
				break;
			}
		} else {
		}

		if (!getAxisStatus(ctx, AXIS_X, &info.axisX.status)) reprintf(ScreenOutput::ERROR, "getAxisStatus AXIS_X error\n");		
		if (!getAxisParameter(ctx, AXIS_X, ACT_POS, &info.axisX.position)) reprintf(ScreenOutput::ERROR, "getAxisParameter AXIS_X error\n");
		if (!getAxisParameter(ctx, AXIS_X, ACT_SPD, &info.axisX.speed)) reprintf(ScreenOutput::ERROR, "getAxisParameter AXIS_X error\n");

		if (!getAxisStatus(ctx, AXIS_Y, &info.axisY.status)) reprintf(ScreenOutput::ERROR, "getAxisStatus AXIS_Y error\n");		
		if (!getAxisParameter(ctx, AXIS_Y, ACT_POS, &info.axisY.position)) reprintf(ScreenOutput::ERROR, "getAxisParameter AXIS_Y error\n");
		if (!getAxisParameter(ctx, AXIS_Y, ACT_SPD, &info.axisY.speed)) reprintf(ScreenOutput::ERROR, "getAxisParameter AXIS_Y error\n");

		if (!getAxisStatus(ctx, AXIS_Z, &info.axisZ.status)) reprintf(ScreenOutput::ERROR, "getAxisStatus AXIS_Z error\n");		
		if (!getAxisParameter(ctx, AXIS_Z, ACT_POS, &info.axisZ.position)) reprintf(ScreenOutput::ERROR, "getAxisParameter AXIS_Z error\n");
		if (!getAxisParameter(ctx, AXIS_Z, ACT_SPD, &info.axisZ.speed)) reprintf(ScreenOutput::ERROR, "getAxisParameter AXIS_Z error\n");

		ts_now = ros::Time::now();
        info.header.stamp = ts_now;
        pub_info->publish(info);

		if (*modbusLoopState!=ModbusLoopState::FINISH) {
			*modbusLoopState = ModbusLoopState::RUNNING;
		}

		r.sleep();
	}
}

int main(int argc, char* argv[]) {
	ros::init(argc, argv, "gantry_robot");
	ros::NodeHandle nh("~");

	std::string serial_port;
	int baud_rate;
	int id;
	ModbusLoopState modbusLoopState = ModbusLoopState::INIT;

#if 0
	ros::param::get("~serial_port", serial_port);
	ros::param::get("~baud_rate", baud_rate);
#else
	nh.getParam("serial_port", serial_port);
	nh.getParam("baud_rate", baud_rate);
#endif

	CommandCase cmdCase = CommandCase::IDLE;
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

    ros::Publisher pub_info = nh.advertise<gantry_robot::Info>("gantry_robot_info", 100);

    int main_hz = 1000;
    std::queue<AxisMsg> que;
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
		reprintf(ScreenOutput::DEFAULT, "[%s{%s}(%d)] : CommandCase::%d FunctionCase::%d\n", __FILENAME__, __FUNCTION__, __LINE__, (int32_t)cmdCase, (int32_t)funcCase);

		switch (cmdCase) {
			case CommandCase::IDLE:
				switch (funcCase) {
					case FunctionCase::INIT:
						funcCase = FunctionCase::SET;
					break;
					case FunctionCase::SET:
						funcCase = FunctionCase::ACTION;
					break;
					case FunctionCase::ACTION:
						if (modbusLoopState == ModbusLoopState::RUNNING) {
							reprintf(ScreenOutput::DEFAULT, "[%s{%s}(%d)] : CommandCase::IDLE FunctionCase::ACTION success\n", __FILENAME__, __FUNCTION__, __LINE__);
							cmdCase = CommandCase::HOME;
							funcCase = FunctionCase::INIT;
						} else {
						}
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
						if (info.axisZ.status.output.org == (uint8_t)OnOff::on) {
							reprintf(ScreenOutput::DEFAULT, "[%s{%s}(%d)] : CommandCase::HOME FunctionCase::INIT !already done!\n", __FILENAME__, __FUNCTION__, __LINE__);
							funcCase = FunctionCase::IDLE;
						} else {
							reprintf(ScreenOutput::DEFAULT, "[%s{%s}(%d)] : CommandCase::HOME FunctionCase::INIT\n", __FILENAME__, __FUNCTION__, __LINE__);
							// Axis Reset
							setAxisCommandMsg(&que, AXIS_Z, AxisCommand::emg, OnOff::off);
							setAxisCommandMsg(&que, AXIS_Z, AxisCommand::sv_on, OnOff::on);
							funcCase = FunctionCase::SET;
						}
					break;
					case FunctionCase::SET:
						reprintf(ScreenOutput::DEFAULT, "[%s{%s}(%d)] : CommandCase::HOME FunctionCase::SET\n", __FILENAME__, __FUNCTION__, __LINE__);
						// Homing Parameter
						setHomingParametersMsg(&que, AXIS_Z, HOMING_SPEED_Z_VAL, HOMING_OFFSET_Z_VAL, HOMING_DONE_BEHAVIOUR_Z_VAL);
						funcCase = FunctionCase::ACTION;
					break;
					case FunctionCase::ACTION:
						reprintf(ScreenOutput::DEFAULT, "[%s{%s}(%d)] : CommandCase::HOME FunctionCase::ACTION\n", __FILENAME__, __FUNCTION__, __LINE__);
						// Homing
						setAxisCommandMsg(&que, AXIS_Z, AxisCommand::hstart, OnOff::on);
						setAxisCommandMsg(&que, AXIS_Z, AxisCommand::hstart, OnOff::off);
						funcCase = FunctionCase::IDLE;
					break;
					case FunctionCase::IDLE:
						// funcCase = FunctionCase::IDLE;
						reprintf(ScreenOutput::DEFAULT, "[%s{%s}(%d)] : CommandCase::HOME FunctionCase::IDLE\n", __FILENAME__, __FUNCTION__, __LINE__);
						cmdCase = CommandCase::POSITION;
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
						if (info.axisZ.status.output.inpos1 == (uint8_t)OnOff::on &&
							info.axisZ.status.output.inspd == (uint8_t)OnOff::on &&
							info.axisZ.status.output.org == (uint8_t)OnOff::on) {
							funcCase = FunctionCase::SET;
						} else {
						}
						reprintf(ScreenOutput::DEFAULT, "[%s{%s}(%d)] : CommandCase::POSITION FunctionCase::INIT\n", __FILENAME__, __FUNCTION__, __LINE__);
					break;
					case FunctionCase::SET:
						reprintf(ScreenOutput::DEFAULT, "[%s{%s}(%d)] : CommandCase::POSITION FunctionCase::SET\n", __FILENAME__, __FUNCTION__, __LINE__);
						funcCase = FunctionCase::ACTION;
					break;
					case FunctionCase::ACTION:
						funcCase = FunctionCase::IDLE;
					break;
					case FunctionCase::IDLE:
						reprintf(ScreenOutput::TEMP, "[%s{%s}(%d)] : CommandCase::POSITION FunctionCase::IDLE\n", __FILENAME__, __FUNCTION__, __LINE__);
						funcCase = FunctionCase::IDLE;
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