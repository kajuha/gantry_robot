#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/BatteryState.h>

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

using namespace std;

#include <queue>

class NARRAY {
    int nArray0;
    int nArray1;
    int nArray2;
    int nArray3;
    int nArray4;
};

void modbusLoop(int rate, queue<NARRAY>* qarr) {
    int size;
	ros::Time ts_now;

	ros::Rate r(rate);

	while (ros::ok())
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

#if 0
	ros::param::get("~serial_port", serial_port);
	ros::param::get("~baud_rate", baud_rate);
	ros::param::get("~slave_num", slave_num);
#else
	nh.getParam("serial_port", serial_port);
	nh.getParam("baud_rate", baud_rate);
	nh.getParam("slave_num", slave_num);
#endif

    int main_hz = 100;
    queue<NARRAY> qarr;
    boost::thread threadModbusLoop(modbusLoop, main_hz, &qarr);

	const char* bit_read_subject[] = {
		"POT", "NOT", "HOME", "STOP", "PCON", "GAIN2", "P_CL", "N_CL", "MODE", "RESERVED",
		"EMG", "A_RST", "SV_ON", "SPD1/LVSF1", "SPD2/LVSF2", "SPD3", "START", "PAUSE", "REGT", "HSTART", "ISEL0",
		"ISEL1", "ISEL2", "ISEL3", "ISEL4", "ISEL5", "ABSRQ", "JSTART", "JDIR", "PCLEAR", "AOVR", "RESERVED",
		"BRAKE", "ALARM", "READY", "ZSPD", "INPOS1", "TLMT", "VLMT", "INSPD", "WARN", "TGON", "RESERVED",
		"RESERVED", "RESERVED", "RESERVED", "RESERVED", "RESERVED", "ORG", "EOS", "IOUT0", "IOUT1", "IOUT2", "IOUT3",
		"IOUT4", "IOUT5", "RESERVED", "RESERVED", "RESERVED", "RESERVED", "RESERVED", "RESERVED", "RESERVED", "RESERVED"
	};

	const char* bit_write_subject[] = {
		"POT", "NOT", "HOME", "STOP", "PCON", "GAIN2", "P_CL", "N_CL", "MODE", "RESERVED",
		"EMG", "A_RST", "SV_ON", "SPD1/LVSF1", "SPD2/LVSF2", "SPD3", "START", "PAUSE", "REGT", "HSTART",
		"ISEL0", "ISEL1", "ISEL2", "ISEL3", "ISEL4", "ISEL5", "ABSRQ", "JSTART", "JDIR", "PCLEAR",
		"AOVR", "RESERVED"
	};

	modbus_t* ctx;
    uint8_t read_bits[READ_COIL_SIZE] = {0, };
#define READ_REG_SIZE   0x10
	uint16_t read_registers[READ_REG_SIZE] = {0, };
#define WRITE_COIL_SIZE  0x20
    uint8_t write_bits[WRITE_COIL_SIZE] = {0, };

	sensor_msgs::BatteryState batteryState;

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

	ros::Rate r(1000);

	#define STEP_TIME 1.0
	double time_cur = ros::Time::now().toSec();
	double time_pre = time_cur;
	double time_diff;

	if (modbus_read_bits(ctx, 0, READ_COIL_SIZE, read_bits)) {
	} else {
		printf("modbus_read_bits error\n");
	}

	// LS 메카피온 L7P에서는 2개의 차이가 없음 어느 것으로 읽어도 상관없을 것 같음, kajuha
	printf("read : \n");
	for (int i=0; i<READ_COIL_SIZE; i++) {
		if(strcmp(bit_read_subject[i], "RESERVED")) {
			printf("[%s, 0x%02x] : %x\n", bit_read_subject[i], i, read_bits[i]);
		}
	}

	// E-STOP Disable
	setSingleCoilCmd(ctx, SingleCoilCmd::emg, OnOff::off);
	// Servo On
	setSingleCoilCmd(ctx, SingleCoilCmd::sv_on, OnOff::on);
	// Homing Parameter Set
#define DATA_SIZE	2
	int addr;
	int nb;
	uint16_t write_data[DATA_SIZE] = {0, };
	uint16_t read_data[DATA_SIZE] = {0, };
	
	addr = 0x603E;
	nb = 1;
	write_data[0] = 24;
	// if (modbus_write_and_read_registers(ctx, addr, nb, write_data, addr, nb, read_data)) {
	if (modbus_write_registers(ctx, addr, nb, write_data)) {
		printf("0x%04x: write:%10d, read:%10d\n", addr, write_data[0], read_data[0]);
	} else {
		printf("0x%04x: error\n", addr);
	}
	
	addr = 0x6041;
	nb = 2;
	*((int*)write_data) = 5000;
	// if (modbus_write_and_read_registers(ctx, addr, nb, write_data, addr, nb, read_data)) {
	if (modbus_write_registers(ctx, addr, nb, write_data)) {
		printf("0x%04x: write:%10d, read:%10d\n", addr, *((int*)write_data), *((int*)read_data));
	} else {
		printf("0x%04x: error\n", addr);
	}
	
	addr = 0x6043;
	nb = 2;
	*((int*)write_data) = 5000;
	// if (modbus_write_and_read_registers(ctx, addr, nb, write_data, addr, nb, read_data)) {
	if (modbus_write_registers(ctx, addr, nb, write_data)) {
		printf("0x%04x: write:%10d, read:%10d\n", addr, *((int*)write_data), *((int*)read_data));
	} else {
		printf("0x%04x: error\n", addr);
	}
	
	addr = 0x6045;
	nb = 2;
	*((int*)write_data) = 10000;
	// if (modbus_write_and_read_registers(ctx, addr, nb, write_data, addr, nb, read_data)) {
	if (modbus_write_registers(ctx, addr, nb, write_data)) {
		printf("0x%04x: write:%10d, read:%10d\n", addr, *((int*)write_data), *((int*)read_data));
	} else {
		printf("0x%04x: error\n", addr);
	}
	
	addr = 0x6024;
	nb = 2;
	*((int*)write_data) = 8097;
	// if (modbus_write_and_read_registers(ctx, addr, nb, write_data, addr, nb, read_data)) {
	if (modbus_write_registers(ctx, addr, nb, write_data)) {
		printf("0x%04x: write:%10d, read:%10d\n", addr, *((int*)write_data), *((int*)read_data));
	} else {
		printf("0x%04x: error\n", addr);
	}
	
	addr = 0x6034;
	nb = 2;
	*((int*)write_data) = 10000;
	// if (modbus_write_and_read_registers(ctx, addr, nb, write_data, addr, nb, read_data)) {
	if (modbus_write_registers(ctx, addr, nb, write_data)) {
		printf("0x%04x: write:%10d, read:%10d\n", addr, *((int*)write_data), *((int*)read_data));
	} else {
		printf("0x%04x: error\n", addr);
	}
	
	addr = 0x201F;
	nb = 1;
	write_data[0] = 1;
	// if (modbus_write_and_read_registers(ctx, addr, nb, write_data, addr, nb, read_data)) {
	if (modbus_write_registers(ctx, addr, nb, write_data)) {
		printf("0x%04x: write:%10d, read:%10d\n", addr, write_data[0], read_data[0]);
	} else {
		printf("0x%04x: error\n", addr);
	}
	

	// Homing
	setSingleCoilCmd(ctx, SingleCoilCmd::hstart, OnOff::on);
	// Homing Signal Disable
	setSingleCoilCmd(ctx, SingleCoilCmd::hstart, OnOff::off);

	if (modbus_read_bits(ctx, 0, READ_COIL_SIZE, read_bits)) {
	} else {
		printf("modbus_read_bits error\n");
	}

	// LS 메카피온 L7P에서는 2개의 차이가 없음 어느 것으로 읽어도 상관없을 것 같음, kajuha
	printf("read : \n");
	for (int i=0; i<READ_COIL_SIZE; i++) {
		if(strcmp(bit_read_subject[i], "RESERVED")) {
			printf("[%s] : %x\n", bit_read_subject[i], read_bits[i]);
		}
	}

	while (ros::ok())
	{
		// modbus_read_input_registers return
		// the number of read input registers
		// -1 : error, set errno
		if (modbus_read_bits(ctx, 0, READ_COIL_SIZE, read_bits)) {
		} else {
			printf("modbus_read_bits error\n");
		}
#define ACT_POS 0x600E
#define DINT    2
		if (modbus_read_registers(ctx, ACT_POS, DINT, read_registers)) {
		} else {
			printf("modbus_read_registers error\n");
		}
#define ACT_SPD 0x6018
#define DINT    2
		if (modbus_read_registers(ctx, ACT_SPD, DINT, read_registers+2)) {
		} else {
			printf("modbus_read_registers error\n");
		}
        
		printf("inpos: %2d, org: %2d, pos: %+5d, spd: %+5d \n", read_bits[0x24], read_bits[0x30], *((int32_t*)read_registers), *(((int32_t*)read_registers)+1));

		time_cur = ros::Time::now().toSec();
		time_diff = time_cur - time_pre;
		if ( time_diff > STEP_TIME ) {
			time_pre = time_cur;
		}

		ros::spinOnce();

		r.sleep();
	}

    threadModbusLoop.join();

	// modbus_close no return
	modbus_close(ctx);
	// modbus_free no return
	modbus_free(ctx);

	return 0;
}
