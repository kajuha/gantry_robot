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

void modbusLoop(int rate, queue<NARRAY>* qarr, int* isModbusLoopEnd) {
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
	ros::param::get("~slave_num", slave_num);
#else
	nh.getParam("serial_port", serial_port);
	nh.getParam("baud_rate", baud_rate);
	nh.getParam("slave_num", slave_num);
#endif

enum class CommandCase {
	NONE, HOME, POSITION, JOG
};

enum class FunctionCase {
	INIT, SET, ACTION, IDLE
};

	CommandCase cmdCase = CommandCase::NONE;
	FunctionCase funcCase = FunctionCase::IDLE;

    int main_hz = 100;
    queue<NARRAY> qarr;
    boost::thread threadModbusLoop(modbusLoop, main_hz, &qarr, &isModbusLoopEnd);

	const char* bit_read_subject[] = {
		"POT", "NOT", "HOME", "STOP", "PCON", "GAIN2", "P_CL", "N_CL", "MODE", "RESERVED1",
		"EMG", "A_RST", "SV_ON", "SPD1_LVSF1", "SPD2_LVSF2", "SPD3", "START", "PAUSE", "REGT", "HSTART", "ISEL0",
		"ISEL1", "ISEL2", "ISEL3", "ISEL4", "ISEL5", "ABSRQ", "JSTART", "JDIR", "PCLEAR", "AOVR", "RESERVED2",
		"BRAKE", "ALARM", "READY", "ZSPD", "INPOS1", "TLMT", "VLMT", "INSPD", "WARN", "TGON", "RESERVED1",
		"RESERVED2", "RESERVED3", "RESERVED4", "RESERVED5", "RESERVED6", "ORG", "EOS", "IOUT0", "IOUT1", "IOUT2", "IOUT3",
		"IOUT4", "IOUT5", "RESERVED7", "RESERVED8", "RESERVED9", "RESERVED10", "RESERVED11", "RESERVED12", "RESERVED13", "RESERVED14"
	};

	modbus_t* ctx;
#define READ_REG_SIZE   0x10
	uint16_t read_registers[READ_REG_SIZE] = {0, };
#define WRITE_COIL_SIZE  0x20
    uint8_t write_bits[WRITE_COIL_SIZE] = {0, };

	ctx = modbus_new_rtu(serial_port.c_str(), baud_rate, 'N', 8, 1);

	printf("%s %d %d\n", serial_port.c_str(), baud_rate, slave_num);

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

	ros::Rate r(1000);

	#define STEP_TIME 1.0
	double time_cur = ros::Time::now().toSec();
	double time_pre = time_cur;
	double time_diff;

#define AXIS_X  11
#define AXIS_Y  12
#define AXIS_Z  13
    // slave_num = AXIS_X;
    // if (modbus_set_slave(ctx, slave_num) == -1) {
    //     printf("Unable to set the slave ID in context: %s \n", modbus_strerror(errno));
    // } else {
	// 	// E-STOP Disable
	// 	setAxisCommand(ctx, AxisCommand::emg, OnOff::off);
	// 	// Servo On
	// 	setAxisCommand(ctx, AxisCommand::sv_on, OnOff::on);
	// }

    // slave_num = AXIS_Y;
    // if (modbus_set_slave(ctx, slave_num) == -1) {
    //     printf("Unable to set the slave ID in context: %s \n", modbus_strerror(errno));
    // } else {
	// 	// E-STOP Disable
	// 	setAxisCommand(ctx, AxisCommand::emg, OnOff::off);
	// 	// Servo On
	// 	setAxisCommand(ctx, AxisCommand::sv_on, OnOff::on);
	// }

    slave_num = AXIS_Z;
    if (modbus_set_slave(ctx, slave_num) == -1) {
        printf("Unable to set the slave ID in context: %s \n", modbus_strerror(errno));
    } else {
		// E-STOP Disable
		setAxisCommand(ctx, AxisCommand::emg, OnOff::off);
		// Servo On
		setAxisCommand(ctx, AxisCommand::sv_on, OnOff::on);
	}

#define HOMING_SPEED_X  5000
#define HOMING_SPEED_Y  5000
#define HOMING_SPEED_Z  5000
#define HOMING_OFFSET_X 131072
#define HOMING_OFFSET_Y 5160
#define HOMING_OFFSET_Z 8097
#define HOMING_DONE_BEHAVIOUR_X OnOff::off
#define HOMING_DONE_BEHAVIOUR_Y OnOff::on
#define HOMING_DONE_BEHAVIOUR_Z OnOff::on
    slave_num = AXIS_X;
    if (modbus_set_slave(ctx, slave_num) == -1) {
        printf("Unable to set the slave ID in context: %s \n", modbus_strerror(errno));
    } else {
    	setHomingParameters(ctx, HOMING_SPEED_X, HOMING_OFFSET_X, HOMING_DONE_BEHAVIOUR_X);
	}
    
    slave_num = AXIS_Y;
    if (modbus_set_slave(ctx, slave_num) == -1) {
        printf("Unable to set the slave ID in context: %s \n", modbus_strerror(errno));
    } else {
    	setHomingParameters(ctx, HOMING_SPEED_X, HOMING_OFFSET_X, HOMING_DONE_BEHAVIOUR_X);
	}
    
    slave_num = AXIS_Z;
    if (modbus_set_slave(ctx, slave_num) == -1) {
        printf("Unable to set the slave ID in context: %s \n", modbus_strerror(errno));
    } else {
    	setHomingParameters(ctx, HOMING_SPEED_X, HOMING_OFFSET_X, HOMING_DONE_BEHAVIOUR_X);
	}

	// Homing
    // slave_num = AXIS_X;
    // if (modbus_set_slave(ctx, slave_num) == -1) {
    //     printf("Unable to set the slave ID in context: %s \n", modbus_strerror(errno));
    // } else {
	// 	setAxisCommand(ctx, AxisCommand::hstart, OnOff::on);
	// 	// Homing Signal Disable
	// 	setAxisCommand(ctx, AxisCommand::hstart, OnOff::off);
	// }

    // slave_num = AXIS_Y;
    // if (modbus_set_slave(ctx, slave_num) == -1) {
    //     printf("Unable to set the slave ID in context: %s \n", modbus_strerror(errno));
    // } else {
	// 	setAxisCommand(ctx, AxisCommand::hstart, OnOff::on);
	// 	// Homing Signal Disable
	// 	setAxisCommand(ctx, AxisCommand::hstart, OnOff::off);
	// }

    slave_num = AXIS_Z;
    if (modbus_set_slave(ctx, slave_num) == -1) {
        printf("Unable to set the slave ID in context: %s \n", modbus_strerror(errno));
    } else {
		setAxisCommand(ctx, AxisCommand::hstart, OnOff::on);
		// Homing Signal Disable
		setAxisCommand(ctx, AxisCommand::hstart, OnOff::off);
	}
    
    gantry_robot::Info info;

	while (ros::ok())
	{
		// 현재 상태 읽기
        slave_num = AXIS_X;
        if (modbus_set_slave(ctx, slave_num) == -1) {
            printf("Unable to set the slave ID in context: %s \n", modbus_strerror(errno));
        } else {
			if (getAxisStatus(ctx, &info.axisX.status)) {
			} else {
				printf("getAxisStatus error\n");
			}
			if (modbus_read_registers(ctx, ACT_POS, (int32_t)ObjType::DINT, read_registers)) {
			} else {
				printf("modbus_read_registers error\n");
			}
#if 0
			if (modbus_read_registers(ctx, ACT_SPD, (int32_t)ObjType::DINT, read_registers+2)) {
			} else {
				printf("modbus_read_registers error\n");
			}
#endif
		}

        info.axisX.position = *((int32_t*)read_registers);
        info.axisX.speed = *(((int32_t*)read_registers)+1);

		// 현재 상태 읽기
        slave_num = AXIS_Y;
        if (modbus_set_slave(ctx, slave_num) == -1) {
            printf("Unable to set the slave ID in context: %s \n", modbus_strerror(errno));
        } else {
			if (getAxisStatus(ctx, &info.axisY.status)) {
			} else {
				printf("getAxisStatus error\n");
			}
			if (modbus_read_registers(ctx, ACT_POS, (int32_t)ObjType::DINT, read_registers)) {
			} else {
				printf("modbus_read_registers error\n");
			}
#if 0
			if (modbus_read_registers(ctx, ACT_SPD, (int32_t)ObjType::DINT, read_registers+2)) {
			} else {
				printf("modbus_read_registers error\n");
			}
#endif
		}

        info.axisY.position = *((int32_t*)read_registers);
        info.axisY.speed = *(((int32_t*)read_registers)+1);
        
		// 현재 상태 읽기
        slave_num = AXIS_Z;
        if (modbus_set_slave(ctx, slave_num) == -1) {
            printf("Unable to set the slave ID in context: %s \n", modbus_strerror(errno));
        } else {
			if (getAxisStatus(ctx, &info.axisZ.status)) {
			} else {
				printf("getAxisStatus error\n");
			}
			if (modbus_read_registers(ctx, ACT_POS, (int32_t)ObjType::DINT, read_registers)) {
			} else {
				printf("modbus_read_registers error\n");
			}
#if 0
			if (modbus_read_registers(ctx, ACT_SPD, (int32_t)ObjType::DINT, read_registers+2)) {
			} else {
				printf("modbus_read_registers error\n");
			}
#endif
		}

        info.axisZ.position = *((int32_t*)read_registers);
        info.axisZ.speed = *(((int32_t*)read_registers)+1);

        info.header.stamp = ros::Time::now();
        pub_info.publish(info);

		time_cur = ros::Time::now().toSec();
		time_diff = time_cur - time_pre;
		if ( time_diff > STEP_TIME ) {
			time_pre = time_cur;
		}

		if ((OnOff)info.axisZ.status.output.org == OnOff::on &&
			(OnOff)info.axisZ.status.output.inpos1 == OnOff::on &&
			info.axisZ.position == 0 ) {
				printf("axis Z done.\n");
			break;
		}

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
