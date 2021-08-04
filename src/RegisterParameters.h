#pragma once

#include <ros/ros.h>

#include "ObjectDictionary.h"

struct RegisterParameter {
    int32_t address;
    ObjAccess access;
    ObjType type;
    const char* subject;
    const char* description;
    const char* default_value;
    const char* minimum;
    const char* maximum;
    const char* unit;
};

#define REGISTER_NUMBER 39
const RegisterParameter registerParameters[] = {
    {0x230D, ObjAccess::RW, ObjType::UINT, "Speed Limit Function Select", "", "0", "0", "3", "-"},
    {0x231A, ObjAccess::RW, ObjType::UINT, "Velocity Command Switch Select", "", "0", "0", "3", "-"},
    {0x2400, ObjAccess::RW, ObjType::UINT, "Software Position Limit Function Select", "", "0", "0", "3", "-"},
    {0x3009, ObjAccess::RW, ObjType::UINT, "Start Index Number(0~63)", "", "0", "0", "64", "-"},
    {0x300A, ObjAccess::RW, ObjType::UINT, "Index Buffer Mode", "", "0", "0", "1", "-"},
    {0x3100, ObjAccess::RW, ObjType::UINT16, "Number of entries", "", "-", "-", "-", "-"},
    {0x3101, ObjAccess::RW, ObjType::UINT16, "IndexType", "", "-", "0", "10", "-"},
    {0x3102, ObjAccess::RW, ObjType::INT32, "Distance", "", "-", "-2147483648", "2147483647", "UU"},
    {0x3104, ObjAccess::RW, ObjType::INT32, "Velocity", "", "-", "1", "2147483647", "UU/s"},
    {0x3106, ObjAccess::RW, ObjType::INT32, "Acceleration", "", "-", "1", "2147483647", "UU/s2"},
    {0x3108, ObjAccess::RW, ObjType::INT32, "Deceleration", "", "-", "1", "2147483647", "UU/s2"},
    {0x310A, ObjAccess::RW, ObjType::INT32, "RegDistance", "", "-", "-2147483648", "2147483647", "UU"},
    {0x310C, ObjAccess::RW, ObjType::INT32, "RegVelocity", "", "-", "1", "2147483647", "UU/s2"},
    {0x310E, ObjAccess::RW, ObjType::UINT16, "RepeatCount", "", "-", "1", "65535", "-"},
    {0x310F, ObjAccess::RW, ObjType::UINT16, "DwellTime", "", "-", "0", "65535", "ms"},
    {0x3110, ObjAccess::RW, ObjType::UINT16, "Next Index", "", "-", "0", "63", "-"},
    {0x3111, ObjAccess::RW, ObjType::UINT16, "Action", "", "-", "0", "2", "-"},
    {0x2002, ObjAccess::RW, ObjType::UDINT, "Encoder Pulse per Revolution", "(General)", "524288", "0", "1073741824", "pulse"},
    {0x2004, ObjAccess::RO, ObjType::UINT, "Node ID", "(General)", "-", "0", "65535", "-"},
    {0x603E, ObjAccess::RW, ObjType::INT, "Homing Method", "(Homing)1.Homing Method", "34", "-128", "127", "-"},
    {0x6041, ObjAccess::RW, ObjType::DINT, "Homing Speed (switch)", "(Homing)2.Switch Search Velocity", "500000", "0", "0x40000000", "UU/s"},
    {0x6043, ObjAccess::RW, ObjType::DINT, "Homing Speed (zero)", "(Homing)3.Marker Search Velocity", "100000", "0", "0x40000000", "UU/s"},
    {0x6045, ObjAccess::RW, ObjType::UDINT, "Homing Acceleration", "(Homing)4.Acceleration", "200000", "0", "0x40000000", "UU/s2"},
    {0x6024, ObjAccess::RW, ObjType::DINT, "Home Offset", "(Homing)5.Home Offset", "0", "-536870912", "536870911", "UU"},
    {0x6034, ObjAccess::RW, ObjType::DINT, "Quick Stop Deceleration", "(Homing)6.Deceleration", "200000", "0", "0x7FFFFFFF", "UU/s2"},
    {0x201F, ObjAccess::RW, ObjType::UINT, "Homing Done Behaviour", "(Homing)7.Homing Done Behavior", "0", "0", "1", "-"},
    {0x2409, ObjAccess::RW, ObjType::UINT, "Torque Limit at Homing Using Stopper", "(Homing)8.Torque Limit at Homing Using Stopper", "250", "0", "2000", "0.10%"},
    {0x240A, ObjAccess::RW, ObjType::UINT, "Duration Time at Homing Using Stopper", "(Homing)9.Duration Time at Homing Using Stopper", "50", "0", "1000", "ms"},
    {0x2601, ObjAccess::RO, ObjType::INT, "Command Speed", "(Indexing)Command Speed", "-", "-", "-", "rpm"},
    {0x2600, ObjAccess::RO, ObjType::INT, "Feedback Speed", "(Indexing)Feedback Speed", "-", "-", "-", "rpm"},
    {0x600E, ObjAccess::RO, ObjType::DINT, "Position Actual Value", "(Indexing)Position Actual Value", "-", "-", "-", "UU"},
    {0x600A, ObjAccess::RO, ObjType::DINT, "Position Demand Valude", "(Indexing)Position Demand Value", "-", "-", "-", "UU"},
    {0x6018, ObjAccess::RO, ObjType::DINT, "Velocity Actual Value", "(Indexing)Velocity Actual Value", "-", "-", "-", "UU/s"},
    {0x6016, ObjAccess::RO, ObjType::DINT, "Velocity Demand Value", "(Indexing)Velocity Demand Value", "-", "-", "-", "UU/s"},
    {0x2300, ObjAccess::RW, ObjType::INT, "Jog Operation Speed", "(Manual Jog)1.Speed", "500", "-6000", "6000", "rpm"},
    {0x2301, ObjAccess::RW, ObjType::UINT, "Speed Command Acceleration Time", "(Manual Jog)2.Accel Time", "200", "0", "10000", "ms"},
    {0x2302, ObjAccess::RW, ObjType::UINT, "Speed Command Deceleration Time", "(Manual Jog)3.decel Time", "200", "0", "10000", "ms"},
    {0x2303, ObjAccess::RW, ObjType::UINT, "Speed Command S-curve Time", "(Manual Jog)4.S-cruve Time", "0", "0", "1000", "ms"},
    {0x2311, ObjAccess::RW, ObjType::UINT, "Servo-Lock Function Select", "(Manual Jog)5.Servo-Lock Function Select", "0", "0", "1", "-"}
};