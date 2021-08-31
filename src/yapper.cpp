#include "yapper.h"
#include "main.h"
#include "L7P.h"

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