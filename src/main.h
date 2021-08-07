#pragma once

#include <string.h>

enum class ModbusLoopState {
    INIT, RUNNING, FINISH
};

enum class ScreenOutput {
    NO, DEFAULT, ALWAYS, TEMP, ERROR
};

#define OUTPUT_DEFAULT  1
#define OUTPUT_TEMP     1

void reprintf(ScreenOutput screenOutput, const char* format, ...);

#define __FILENAME__ (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)