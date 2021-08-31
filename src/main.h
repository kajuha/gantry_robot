#pragma once

#include <string.h>
#include <math.h>

#include "L7P.h"

extern std::queue<AxisMsg> queueModbus;
extern std::queue<CommandState> queueCommandState;

extern gantry_robot::Info info;
extern GlobalInfo gInfo;

enum class ModbusLoopState {
    INIT, RUNNING, FINISH
};

enum class ScreenOutput {
    NO, DEFAULT, ALWAYS, TEMP, ERROR
};

#define RAD_TO_DEG(X)   (X*(180.0/M_PI))
#define DEG_TO_RAD(X)   (X*(M_PI/180.0))
#define MM_TO_M(X)      (X/1000.0)
#define M_TO_MM(X)      (X*1000.0)

#define TS_ELAPSE_ERROR 10

#define SRV_HZ          1000

#define SRV_SUCCESS     1
#define SRV_CHECKING    0
#define SRV_FAIL        -1

#define SRV_AXIS_X      "x"
#define SRV_AXIS_Y      "y"
#define SRV_AXIS_Z      "z"

#define NULL_CHAR       '\0'

#define ENUM_MAX_VAL    1

#define VAL_SET         1
#define VAL_RST         0

#define ERR_SET         1
#define ERR_RST         0

#define OUTPUT_DEFAULT  1
#define OUTPUT_TEMP     1

void reprintf(ScreenOutput screenOutput, const char* format, ...);

#define __FILENAME__ (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)

void setState();
void clearQueueModbus();
void clearQueueCommandState();