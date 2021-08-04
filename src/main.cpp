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

#include "L7P.h"
#include "ObjectDictionary.h"

#include <gantry_robot/Info.h>

using namespace std;

#include <queue>

class NARRAY {
    int nArray0;
    int nArray1;
    int nArray2;
    int nArray3;
    int nArray4;
};

gantry_robot::Info info;

void modbusLoop(int rate, queue<NARRAY>* qarr, int* isModbusLoopEnd, ros::Publisher* pub_info, modbus_t* ctx) {
    int size;
	ros::Time ts_now;

	ros::Rate r(rate);

	while (ros::ok() && !(*isModbusLoopEnd))
	{
        size = qarr->size();

        if (size) {
			// 첫 번째 데이터 확인
			qarr->front();
			// 첫 번째 데이터 제거
			qarr->pop();
		} else {
		}

		// if (!getAxisStatus(ctx, AXIS_X, &info.axisX.status)) printf("getAxisStatus AXIS_X error\n");		
		// if (!getAxisParameter(ctx, AXIS_X, ACT_POS, &info.axisX.position)) printf("getAxisParameter AXIS_X error\n");
		// if (!getAxisParameter(ctx, AXIS_X, ACT_SPD, &info.axisX.speed)) printf("getAxisParameter AXIS_X error\n");

		// if (!getAxisStatus(ctx, AXIS_Y, &info.axisY.status)) printf("getAxisStatus AXIS_Y error\n");		
		// if (!getAxisParameter(ctx, AXIS_Y, ACT_POS, &info.axisY.position)) printf("getAxisParameter AXIS_Y error\n");
		// if (!getAxisParameter(ctx, AXIS_Y, ACT_SPD, &info.axisY.speed)) printf("getAxisParameter AXIS_Y error\n");

		// if (!getAxisStatus(ctx, AXIS_Z, &info.axisZ.status)) printf("getAxisStatus AXIS_Z error\n");		
		// if (!getAxisParameter(ctx, AXIS_Z, ACT_POS, &info.axisZ.position)) printf("getAxisParameter AXIS_Z error\n");
		// if (!getAxisParameter(ctx, AXIS_Z, ACT_SPD, &info.axisZ.speed)) printf("getAxisParameter AXIS_Z error\n");

        info.header.stamp = ros::Time::now();
        pub_info->publish(info);

		ts_now = ros::Time::now();

		r.sleep();
	}
}

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "gantry_robot");
	ros::NodeHandle nh("~");

	std::string serial_port;
	int baud_rate;
	int slave_num;
#define END	1
#define NOT	0
	int isModbusLoopEnd = NOT;

#if 0
	ros::param::get("~serial_port", serial_port);
	ros::param::get("~baud_rate", baud_rate);
#else
	nh.getParam("serial_port", serial_port);
	nh.getParam("baud_rate", baud_rate);
#endif

	CommandCase cmdCase = CommandCase::NONE;
	FunctionCase funcCase = FunctionCase::IDLE;

	modbus_t* ctx;

	ctx = modbus_new_rtu(serial_port.c_str(), baud_rate, 'N', 8, 1);

	printf("%s %d\n", serial_port.c_str(), baud_rate);

	// modbus_new_rtu return
	// pointer : successful
	// NULL : error, set errno
	if (ctx == NULL) {
		printf("Unable to allocate libmodbus context: %s \n", modbus_strerror(errno));

		return -1;
	}

	// modbus_set_slave return
	// 0: successful
	// -1 : error, set errno
	if (modbus_set_slave(ctx, slave_num) == -1) {
		printf("Unable to set the slave ID in context: %s \n", modbus_strerror(errno));

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
		printf("Connection failed: %s \n", modbus_strerror(errno));

		return -1;
	}

    ros::Publisher pub_info = nh.advertise<gantry_robot::Info>("gantry_robot_info", 100);

    int main_hz = 100;
    queue<NARRAY> qarr;
    boost::thread threadModbusLoop(modbusLoop, main_hz, &qarr, &isModbusLoopEnd, &pub_info, ctx);

	// Axis Reset
	// if (!setAxisCommand(ctx, AXIS_X, AxisCommand::emg, OnOff::off)) printf("setAxisCommand AxisCommand::emg AXIS_X error\n");
	// if (!setAxisCommand(ctx, AXIS_X, AxisCommand::sv_on, OnOff::on)) printf("setAxisCommand AxisCommand::sv_on AXIS_X error\n");

	// if (!setAxisCommand(ctx, AXIS_Y, AxisCommand::emg, OnOff::off)) printf("setAxisCommand AxisCommand::emg AXIS_Y error\n");
	// if (!setAxisCommand(ctx, AXIS_Y, AxisCommand::sv_on, OnOff::on)) printf("setAxisCommand AxisCommand::sv_on AXIS_Y error\n");

	if (!setAxisCommand(ctx, AXIS_Z, AxisCommand::emg, OnOff::off)) printf("setAxisCommand AxisCommand::emg AXIS_Z error\n");
	if (!setAxisCommand(ctx, AXIS_Z, AxisCommand::sv_on, OnOff::on)) printf("setAxisCommand AxisCommand::sv_on AXIS_Z error\n");

	// Homing Parameter
	// if (!setHomingParameters(ctx, AXIS_X, HOMING_SPEED_X, HOMING_OFFSET_X, HOMING_DONE_BEHAVIOUR_X)) printf("setHomingParameters AXIS_X error\n");

	// if (!setHomingParameters(ctx, AXIS_Y, HOMING_SPEED_Y, HOMING_OFFSET_Y, HOMING_DONE_BEHAVIOUR_Y)) printf("setHomingParameters AXIS_Y error\n");

	if (!setHomingParameters(ctx, AXIS_Z, HOMING_SPEED_Z, HOMING_OFFSET_Z, HOMING_DONE_BEHAVIOUR_Z)) printf("setHomingParameters AXIS_Z error\n");

	// Homing    
	// if (!setAxisCommand(ctx, AXIS_X, AxisCommand::hstart, OnOff::on)) printf("setAxisCommand AxisCommand::hstart AXIS_X error\n");
	// if (!setAxisCommand(ctx, AXIS_X, AxisCommand::hstart, OnOff::off)) printf("setAxisCommand AxisCommand::hstart AXIS_X error\n");

	// if (!setAxisCommand(ctx, AXIS_Y, AxisCommand::hstart, OnOff::on)) printf("setAxisCommand AxisCommand::hstart AXIS_Y error\n");
	// if (!setAxisCommand(ctx, AXIS_Y, AxisCommand::hstart, OnOff::off)) printf("setAxisCommand AxisCommand::hstart AXIS_Y error\n");

	if (!setAxisCommand(ctx, AXIS_Z, AxisCommand::hstart, OnOff::on)) printf("setAxisCommand AxisCommand::hstart AXIS_Z error\n");
	if (!setAxisCommand(ctx, AXIS_Z, AxisCommand::hstart, OnOff::off)) printf("setAxisCommand AxisCommand::hstart AXIS_Z error\n");

	ros::Rate r(1000);

	#define STEP_TIME 1.0
	double time_cur = ros::Time::now().toSec();
	double time_pre = time_cur;
	double time_diff;

	int count = 0;

	while (ros::ok())
	{
		time_cur = ros::Time::now().toSec();
		time_diff = time_cur - time_pre;
		if ( time_diff > STEP_TIME ) {
			time_pre = time_cur;
		}

		// if ((OnOff)info.axisZ.status.output.org == OnOff::on &&
		// 	(OnOff)info.axisZ.status.output.inpos1 == OnOff::on &&
		// 	info.axisZ.position == 0 ) {
		// 		printf("axis Z done.\n");
		// 	break;
		// }

		switch (cmdCase) {
			case CommandCase::NONE:
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
				}
			break;
			case CommandCase::HOME:
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
				}
			break;
			case CommandCase::POSITION:
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
				}
			break;
		}

		ros::spinOnce();

		r.sleep();
	}

	isModbusLoopEnd = END;
    threadModbusLoop.join();

	printf("threadModbusLoop joined.\n");

	// modbus_close no return
	modbus_close(ctx);
	// modbus_free no return
	modbus_free(ctx);

	printf("program end.\n");

	return 0;
}
