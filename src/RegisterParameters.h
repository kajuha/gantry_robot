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

#define REGISTER_NUMBER 316
const RegisterParameter registerParameters[] = {
    {0x1000, ObjAccess::RO, ObjType::UDINT, "Device Type", "", "0x00020192", "-", "-", "-"},
    {0x1002, ObjAccess::RO, ObjType::USINT, "Error Register", "", "0x00", "-", "-", "-"},
    {0x1004, ObjAccess::RO, ObjType::STRING, "Device Name", "", "-", "-", "-", "-"},
    {0x100A, ObjAccess::RO, ObjType::STRING, "Hardware Version", "", "-", "-", "-", "-"},
    {0x100D, ObjAccess::RO, ObjType::STRING, "Software Version", "", "-", "-", "-", "-"},
    {0x1010, ObjAccess::RW, ObjType::UDINT, "Store Parameters", "", "0", "0", "0xFFFFFFFF", "-"},
    {0x1012, ObjAccess::RW, ObjType::UDINT, "Store Parameters", "", "0", "0", "0xFFFFFFFF", "-"},
    {0x1014, ObjAccess::RW, ObjType::UDINT, "Store Parameters", "", "0", "0", "0xFFFFFFFF", "-"},
    {0x1016, ObjAccess::RW, ObjType::UDINT, "Store Parameters", "", "0", "0", "0xFFFFFFFF", "-"},
    {0x1018, ObjAccess::RW, ObjType::UDINT, "Store Parameters", "", "0", "0", "0xFFFFFFFF", "-"},
    {0x101A, ObjAccess::RW, ObjType::UDINT, "Restore Default Parameters", "", "0", "0", "0xFFFFFFFF", "-"},
    {0x101C, ObjAccess::RW, ObjType::UDINT, "Restore Default Parameters", "", "0", "0", "0xFFFFFFFF", "-"},
    {0x101E, ObjAccess::RW, ObjType::UDINT, "Restore Default Parameters", "", "0", "0", "0xFFFFFFFF", "-"},
    {0x1020, ObjAccess::RW, ObjType::UDINT, "Restore Default Parameters", "", "0", "0", "0xFFFFFFFF", "-"},
    {0x1022, ObjAccess::RW, ObjType::UDINT, "Restore Default Parameters", "", "0", "0", "0xFFFFFFFF", "-"},
    {0x1024, ObjAccess::RO, ObjType::UDINT, "Identity Object", "", "-", "-", "-", "-"},
    {0x1026, ObjAccess::RO, ObjType::UDINT, "Identity Object", "", "-", "-", "-", "-"},
    {0x1028, ObjAccess::RO, ObjType::UDINT, "Identity Object", "", "-", "-", "-", "-"},
    {0x102A, ObjAccess::RO, ObjType::UDINT, "Identity Object", "", "-", "-", "-", "-"},
    {0x2000, ObjAccess::RW, ObjType::UINT, "Motor ID", "", "13", "1", "9999", "-"},
    {0x2001, ObjAccess::RW, ObjType::UINT, "Encoder Type", "", "1", "0", "99", "-"},
    {0x2002, ObjAccess::RW, ObjType::UDINT, "Encoder Pulse per Revolution", "(General)", "524288", "0", "1073741824", "pulse"},
    {0x2004, ObjAccess::RO, ObjType::UINT, "Node ID", "(General)", "-", "0", "65535", "-"},
    {0x2005, ObjAccess::RW, ObjType::UINT, "Rotation Direction Select", "", "0", "0", "1", "-"},
    {0x2006, ObjAccess::RW, ObjType::UINT, "Absolute Encoder Configuration", "", "1", "0", "1", "-"},
    {0x2007, ObjAccess::RW, ObjType::UINT, "Main Power Fail Check Mode", "", "0", "0", "255", "-"},
    {0x2008, ObjAccess::RW, ObjType::UINT, "Main Power Fail Check Time", "", "20", "0", "5000", "ms"},
    {0x2009, ObjAccess::RW, ObjType::UINT, "7SEG Display Selection", "", "0", "0", "100", "-"},
    {0x200A, ObjAccess::RW, ObjType::UINT, "Regeneration Brake Resistor Configuration", "", "0", "0", "1", "-"},
    {0x200B, ObjAccess::RW, ObjType::UINT, "Regeneration Brake Resistor Derating Factor", "", "100", "0", "200", "%"},
    {0x200C, ObjAccess::RW, ObjType::UINT, "Regeneration Brake Resistor Value", "", "0", "0", "1000", "ohm"},
    {0x200D, ObjAccess::RW, ObjType::UINT, "Regeneration Brake Resistor Power", "", "0", "0", "30000", "watt"},
    {0x200E, ObjAccess::RW, ObjType::UINT, "Peak Power of Regeneration Brake Resistor", "", "100", "1", "50000", "watt"},
    {0x200F, ObjAccess::RW, ObjType::UINT, "Duration Time @ Peak Power of Regeneration Brake Resistor", "", "5000", "1", "50000", "ms"},
    {0x2010, ObjAccess::RW, ObjType::UINT, "Overload Check Base", "", "100", "10", "120", "%"},
    {0x2011, ObjAccess::RW, ObjType::UINT, "Overload Warning Level", "", "50", "10", "100", "%"},
    {0x2012, ObjAccess::RW, ObjType::UINT, "PWM Off Delay Time", "", "10", "0", "1000", "ms"},
    {0x2013, ObjAccess::RW, ObjType::UINT, "Dynamic Brake Control Mode", "", "0", "0", "3", "-"},
    {0x2014, ObjAccess::RW, ObjType::UINT, "Emergency Stop Configuration", "", "1", "0", "1", "-"},
    {0x2015, ObjAccess::RW, ObjType::UINT, "Warning Mask Configuration", "", "0", "0", "0xFFFF", "-"},
    {0x2016, ObjAccess::RW, ObjType::INT, "U Phase Current Offset", "", "0", "-1000", "1000", "0.10%"},
    {0x2017, ObjAccess::RW, ObjType::INT, "V Phase Current Offset", "", "0", "-1000", "1000", "0.10%"},
    {0x2018, ObjAccess::RW, ObjType::INT, "W Phase Current Offset", "", "0", "-1000", "1000", "0.10%"},
    {0x2019, ObjAccess::RW, ObjType::UINT, "Magnetic Pole Pitch", "", "2400", "1", "65535", "0.01mm"},
    {0x201A, ObjAccess::RW, ObjType::UINT, "Linear Scale Resolution", "", "1000", "1", "65535", "nm"},
    {0x201B, ObjAccess::RW, ObjType::UINT, "Commutation Method", "", "0", "0", "2", "-"},
    {0x201C, ObjAccess::RW, ObjType::UINT, "Commutation Current", "", "500", "0", "1000", "0.10%"},
    {0x201D, ObjAccess::RW, ObjType::UINT, "Commutation Time", "", "1000", "500", "5000", "ms"},
    {0x201E, ObjAccess::RW, ObjType::UINT, "Grating Period of Sinusoidal Encoder", "", "40", "1", "65535", "Um"},
    {0x201F, ObjAccess::RW, ObjType::UINT, "Homing Done Behaviour", "(Homing)7.Homing Done Behavior", "0", "0", "1", "-"},
    {0x2020, ObjAccess::RW, ObjType::UINT, "Velocity Function Select", "", "0", "0", "2", "-"},
    {0x2021, ObjAccess::RW, ObjType::UINT, "Motor Hall Phase Config", "", "0", "0", "65535", "-"},
    {0x2100, ObjAccess::RW, ObjType::UINT, "Inertia Ratio", "", "100", "0", "3000", "%"},
    {0x2101, ObjAccess::RW, ObjType::UINT, "Position Loop Gain 1", "", "50", "1", "500", "1/s"},
    {0x2102, ObjAccess::RW, ObjType::UINT, "Speed Loop Gain 1", "", "75", "1", "2000", "Hz"},
    {0x2103, ObjAccess::RW, ObjType::UINT, "Speed Loop Integral Time Constant 1", "", "50", "1", "1000", "ms"},
    {0x2104, ObjAccess::RW, ObjType::UINT, "Torque Command Filter Time Constant 1", "", "5", "0", "1000", "0.1ms"},
    {0x2105, ObjAccess::RW, ObjType::UINT, "Position Loop Gain 2", "", "30", "1", "500", "1/s"},
    {0x2106, ObjAccess::RW, ObjType::UINT, "Speed Loop Gain 2", "", "50", "1", "2000", "Hz"},
    {0x2107, ObjAccess::RW, ObjType::UINT, "Speed Loop Integral Time Constant 2", "", "50", "1", "1000", "ms"},
    {0x2108, ObjAccess::RW, ObjType::UINT, "Torque Command Filter Time Constant 2", "", "5", "0", "1000", "0.1ms"},
    {0x2109, ObjAccess::RW, ObjType::UINT, "Position Command Filter Time Constant", "", "0", "0", "10000", "0.1ms"},
    {0x210A, ObjAccess::RW, ObjType::UINT, "Position Command Average Filter Time Constant", "", "0", "0", "10000", "0.1ms"},
    {0x210B, ObjAccess::RW, ObjType::UINT, "Speed Feedback Filter Time Constant", "", "5", "0", "10000", "0.1ms"},
    {0x210C, ObjAccess::RW, ObjType::UINT, "Velocity Feed-forward Gain", "", "0", "0", "100", "%"},
    {0x210D, ObjAccess::RW, ObjType::UINT, "Velocity Feed-forward Filter Time Constant", "", "10", "0", "1000", "0.1ms"},
    {0x210E, ObjAccess::RW, ObjType::UINT, "Torque Feed-forward Gain", "", "0", "0", "100", "%"},
    {0x210F, ObjAccess::RW, ObjType::UINT, "Torque Feed-forward Filter Time Constant", "", "10", "0", "1000", "0.1ms"},
    {0x2110, ObjAccess::RW, ObjType::UINT, "Torque Limit Function Select", "", "2", "0", "4", "-"},
    {0x2111, ObjAccess::RW, ObjType::UINT, "External Positive Torque Limit Value", "", "3000", "0", "5000", "0.1%"},
    {0x2112, ObjAccess::RW, ObjType::UINT, "External Negative Torque Limit Value", "", "3000", "0", "5000", "0.1%"},
    {0x2113, ObjAccess::RW, ObjType::UINT, "Emergency Stop Torque", "", "1000", "0", "5000", "0.1%"},
    {0x2114, ObjAccess::RW, ObjType::UINT, "P/PI Control Conversion Mode", "", "0", "0", "4", "-"},
    {0x2115, ObjAccess::RW, ObjType::UINT, "P Control Switch Torque", "", "500", "0", "5000", "0.1%"},
    {0x2116, ObjAccess::RW, ObjType::UINT, "P Control Switch Speed", "", "100", "0", "6000", "rpm"},
    {0x2117, ObjAccess::RW, ObjType::UINT, "P Control Switch Acceleration", "", "1000", "0", "60000", "rpm/s"},
    {0x2118, ObjAccess::RW, ObjType::UINT, "P Control Switch Following Error", "", "100", "0", "60000", "pulse"},
    {0x2119, ObjAccess::RW, ObjType::UINT, "Gain Conversion Mode", "", "0", "0", "7", "-"},
    {0x211A, ObjAccess::RW, ObjType::UINT, "Gain Conversion Time 1", "", "2", "0", "1000", "ms"},
    {0x211B, ObjAccess::RW, ObjType::UINT, "Gain Conversion Time 2", "", "2", "0", "1000", "ms"},
    {0x211C, ObjAccess::RW, ObjType::UINT, "Gain Conversion Waiting Time 1", "", "0", "0", "1000", "ms"},
    {0x211D, ObjAccess::RW, ObjType::UINT, "Gain Conversion Waiting Time 2", "", "0", "0", "1000", "ms"},
    {0x211E, ObjAccess::RW, ObjType::UINT, "Dead Band for Position Control", "", "0", "0", "1000", "UU"},
    {0x211F, ObjAccess::RW, ObjType::UINT, "Drive Control Input 1", "", "0", "0", "0xFFFF", "-"},
    {0x2120, ObjAccess::RW, ObjType::UINT, "Drive Control Input 2", "", "0", "0", "0xFFFF", "-"},
    {0x2121, ObjAccess::RO, ObjType::UINT, "Drive Status Output 1", "", "0", "0", "0xFFFF", "-"},
    {0x2122, ObjAccess::RO, ObjType::UINT, "Drive Status Output 2", "", "0", "0", "0xFFFF", "-"},
    {0x2200, ObjAccess::RW, ObjType::UINT, "Digital Input Signal 1 Selection", "", "0x000F", "0", "0xFFFF", "-"},
    {0x2201, ObjAccess::RW, ObjType::UINT, "Digital Input Signal 2 Selection", "", "0x0001", "0", "0xFFFF", "-"},
    {0x2202, ObjAccess::RW, ObjType::UINT, "Digital Input Signal 3 Selection", "", "0x0002", "0", "0xFFFF", "-"},
    {0x2203, ObjAccess::RW, ObjType::UINT, "Digital Input Signal 4 Selection", "", "0x000C", "0", "0xFFFF", "-"},
    {0x2204, ObjAccess::RW, ObjType::UINT, "Digital Input Signal 5 Selection", "", "0x0010", "0", "0xFFFF", "-"},
    {0x2205, ObjAccess::RW, ObjType::UINT, "Digital Input Signal 6 Selection", "", "0x0004", "0", "0xFFFF", "-"},
    {0x2206, ObjAccess::RW, ObjType::UINT, "Digital Input Signal 7 Selection", "", "0x0012", "0", "0xFFFF", "-"},
    {0x2207, ObjAccess::RW, ObjType::UINT, "Digital Input Signal 8 Selection", "", "0x000B", "0", "0xFFFF", "-"},
    {0x2208, ObjAccess::RW, ObjType::UINT, "Digital Input Signal 9 Selection", "", "0x0003", "0", "0xFFFF", "-"},
    {0x2209, ObjAccess::RW, ObjType::UINT, "Digital Input Signal 10 Selection", "", "0x0013", "0", "0xFFFF", "-"},
    {0x220A, ObjAccess::RW, ObjType::UINT, "Digital Input Signal 11 Selection", "", "0x0014", "0", "0xFFFF", "-"},
    {0x220B, ObjAccess::RW, ObjType::UINT, "Digital Input Signal 12 Selection", "", "0x0015", "0", "0xFFFF", "-"},
    {0x220C, ObjAccess::RW, ObjType::UINT, "Digital Input Signal 13 Selection", "", "0x0016", "0", "0xFFFF", "-"},
    {0x220D, ObjAccess::RW, ObjType::UINT, "Digital Input Signal 14 Selection", "", "0x0017", "0", "0xFFFF", "-"},
    {0x220E, ObjAccess::RW, ObjType::UINT, "Digital Input Signal 15 Selection", "", "0x0018", "0", "0xFFFF", "-"},
    {0x220F, ObjAccess::RW, ObjType::UINT, "Digital Input Signal 16 Selection", "", "0x0019", "0", "0xFFFF", "-"},
    {0x2210, ObjAccess::RW, ObjType::UINT, "Digital Output Signal 1 Selection", "", "0x8002", "0", "0xFFFF", "-"},
    {0x2211, ObjAccess::RW, ObjType::UINT, "Digital Output Signal 2 Selection", "", "0x0003", "0", "0xFFFF", "-"},
    {0x2212, ObjAccess::RW, ObjType::UINT, "Digital Output Signal 3 Selection", "", "0x8001", "0", "0xFFFF", "-"},
    {0x2213, ObjAccess::RW, ObjType::UINT, "Digital Output Signal 4 Selection", "", "0x0005", "0", "0xFFFF", "-"},
    {0x2214, ObjAccess::RW, ObjType::UINT, "Digital Output Signal 5 Selection", "", "0x0010", "0", "0xFFFF", "-"},
    {0x2215, ObjAccess::RW, ObjType::UINT, "Digital Output Signal 6 Selection", "", "0x0011", "0", "0xFFFF", "-"},
    {0x2216, ObjAccess::RW, ObjType::UINT, "Digital Output Signal 7 Selection", "", "0x000A", "0", "0xFFFF", "-"},
    {0x2217, ObjAccess::RW, ObjType::UINT, "Digital Output Signal 8 Selection", "", "0x0006", "0", "0xFFFF", "-"},
    {0x2218, ObjAccess::RW, ObjType::UINT, "Analog Torque Input(command/limit) Scale", "", "100", "-1000", "1000", "0.1%/V"},
    {0x2219, ObjAccess::RW, ObjType::INT, "Analog Torque Input(command/limit) Offset", "", "0", "-1000", "1000", "mV"},
    {0x221A, ObjAccess::RW, ObjType::UINT, "Analog Velociity Override Mode", "", "0", "0", "1", "-"},
    {0x221B, ObjAccess::RW, ObjType::INT, "Analog Velocity Input(command/override) Offset", "", "0", "-1000", "1000", "mV"},
    {0x221C, ObjAccess::RW, ObjType::UINT, "Analog Monitor Output Mode", "", "0", "0", "1", "-"},
    {0x221D, ObjAccess::RW, ObjType::UINT, "Analog Monitor Channel 1 Select", "", "0", "0", "65535", "-"},
    {0x221E, ObjAccess::RW, ObjType::UINT, "Analog Monitor Channel 2 Select", "", "1", "0", "65535", "-"},
    {0x2220, ObjAccess::RW, ObjType::DINT, "Analog Monitor Channel 1 Offset", "", "0", "0", "0x40000000", "-"},
    {0x2222, ObjAccess::RW, ObjType::DINT, "Analog Monitor Channel 2 Offset", "", "0", "0", "0x40000000", "-"},
    {0x2224, ObjAccess::RW, ObjType::UDINT, "Analog Monitor Channel 1 Scale", "", "500", "0", "0x40000000", "-"},
    {0x2226, ObjAccess::RW, ObjType::UDINT, "Analog Monitor Channel 2 Scale", "", "500", "0", "0x40000000", "-"},
    {0x2228, ObjAccess::RW, ObjType::UINT, "Analog Velocity Command Filter Time Constant", "", "2", "0", "1000", "-"},
    {0x2229, ObjAccess::RW, ObjType::UINT, "Analog Torque Command Filter Time Constant", "", "2", "0", "1000", "-"},
    {0x222A, ObjAccess::RW, ObjType::INT, "Analog Velocity Command Scale", "", "100", "-1000", "1000", "-"},
    {0x222B, ObjAccess::RW, ObjType::UINT, "Analog Velocity Command Clamp Level", "", "0", "0", "1000", "-"},
    {0x2300, ObjAccess::RW, ObjType::INT, "Jog Operation Speed", "(Manual Jog)1.Speed", "500", "-6000", "6000", "rpm"},
    {0x2301, ObjAccess::RW, ObjType::UINT, "Speed Command Acceleration Time", "(Manual Jog)2.Accel Time", "200", "0", "10000", "ms"},
    {0x2302, ObjAccess::RW, ObjType::UINT, "Speed Command Deceleration Time", "(Manual Jog)3.decel Time", "200", "0", "10000", "ms"},
    {0x2303, ObjAccess::RW, ObjType::UINT, "Speed Command S-curve Time", "(Manual Jog)4.S-cruve Time", "0", "0", "1000", "ms"},
    {0x2304, ObjAccess::RW, ObjType::INT, "Program Jog Operation Speed 1", "", "0", "-6000", "6000", "rpm"},
    {0x2305, ObjAccess::RW, ObjType::INT, "Program Jog Operation Speed 2", "", "500", "-6000", "6000", "rpm"},
    {0x2306, ObjAccess::RW, ObjType::INT, "Program Jog Operation Speed 3", "", "0", "-6000", "6000", "rpm"},
    {0x2307, ObjAccess::RW, ObjType::INT, "Program Jog Operation Speed 4", "", "-500", "-6000", "6000", "rpm"},
    {0x2308, ObjAccess::RW, ObjType::UINT, "Program Jog Operation Time 1", "", "500", "0", "10000", "ms"},
    {0x2309, ObjAccess::RW, ObjType::UINT, "Program Jog Operation Time 2", "", "5000", "0", "10000", "ms"},
    {0x230A, ObjAccess::RW, ObjType::UINT, "Program Jog Operation Time 3", "", "500", "0", "10000", "ms"},
    {0x230B, ObjAccess::RW, ObjType::UINT, "Program Jog Operation Time 4", "", "5000", "0", "10000", "ms"},
    {0x230C, ObjAccess::RW, ObjType::INT, "Index Pulse Search Speed", "", "20", "-1000", "1000", "rpm"},
    {0x230D, ObjAccess::RW, ObjType::UINT, "Speed Limit Function Select", "", "0", "0", "3", "-"},
    {0x230E, ObjAccess::RW, ObjType::UINT, "Speed Limit Value at Torque Control Mode", "", "1000", "0", "6000", "rpm"},
    {0x230F, ObjAccess::RW, ObjType::UINT, "Over Speed Dection Level", "", "6000", "0", "10000", "rpm"},
    {0x2310, ObjAccess::RW, ObjType::UINT, "Excessive Speed Error Detection Level", "", "5000", "0", "10000", "rpm"},
    {0x2311, ObjAccess::RW, ObjType::UINT, "Servo-Lock Function Select", "(Manual Jog)5.Servo-Lock Function Select", "0", "0", "1", "-"},
    {0x2312, ObjAccess::RW, ObjType::INT, "Multi-Step Operation Speed 1", "", "0", "-32768", "32767", "rpm"},
    {0x2313, ObjAccess::RW, ObjType::INT, "Multi-Step Operation Speed 2", "", "10", "-32768", "32767", "rpm"},
    {0x2314, ObjAccess::RW, ObjType::INT, "Multi-Step Operation Speed 3", "", "50", "-32768", "32767", "rpm"},
    {0x2315, ObjAccess::RW, ObjType::INT, "Multi-Step Operation Speed 4", "", "100", "-32768", "32767", "rpm"},
    {0x2316, ObjAccess::RW, ObjType::INT, "Multi-Step Operation Speed 5", "", "200", "-32768", "32767", "rpm"},
    {0x2317, ObjAccess::RW, ObjType::INT, "Multi-Step Operation Speed 6", "", "500", "-32768", "32767", "rpm"},
    {0x2318, ObjAccess::RW, ObjType::INT, "Multi-Step Operation Speed 7", "", "1000", "-32768", "32767", "rpm"},
    {0x2319, ObjAccess::RW, ObjType::INT, "Multi-Step Operation Speed 8", "", "1500", "-32768", "32767", "rpm"},
    {0x231A, ObjAccess::RW, ObjType::UINT, "Velocity Command Switch Select", "", "0", "0", "3", "-"},
    {0x2400, ObjAccess::RW, ObjType::UINT, "Software Position Limit Function Select", "", "0", "0", "3", "-"},
    {0x2401, ObjAccess::RW, ObjType::UINT, "INPOS1 Output Range", "", "100", "0", "60000", "UU"},
    {0x2402, ObjAccess::RW, ObjType::UINT, "INPOS1 Output Time", "", "0", "0", "1000", "ms"},
    {0x2403, ObjAccess::RW, ObjType::UINT, "INPOS2 Output Range", "", "100", "0", "60000", "UU"},
    {0x2404, ObjAccess::RW, ObjType::UINT, "ZSPD Output Range", "", "10", "0", "6000", "rpm"},
    {0x2405, ObjAccess::RW, ObjType::UINT, "TGON Output Range", "", "100", "0", "6000", "rpm"},
    {0x2406, ObjAccess::RW, ObjType::UINT, "INSPD Output Range", "", "100", "0", "6000", "rpm"},
    {0x2407, ObjAccess::RW, ObjType::UINT, "BRAKE Output Speed", "", "100", "0", "6000", "rpm"},
    {0x2408, ObjAccess::RW, ObjType::UINT, "BRAKE Output Delay Time", "", "100", "0", "1000", "ms"},
    {0x2409, ObjAccess::RW, ObjType::UINT, "Torque Limit at Homing Using Stopper", "(Homing)8.Torque Limit at Homing Using Stopper", "250", "0", "2000", "0.10%"},
    {0x240A, ObjAccess::RW, ObjType::UINT, "Duration Time at Homing Using Stopper", "(Homing)9.Duration Time at Homing Using Stopper", "50", "0", "1000", "ms"},
    {0x240B, ObjAccess::RW, ObjType::UINT, "Modulo Mode", "", "0", "0", "5", "-"},
    {0x240C, ObjAccess::RW, ObjType::DINT, "Modulo Factor", "", "3600", "1", "0x40000000", "UU"},
    {0x240E, ObjAccess::RW, ObjType::STRING, "User Drive Name", "", "Drive", "-", "-", "-"},
    {0x2416, ObjAccess::RW, ObjType::UINT, "Individual Parameter Save", "", "0", "0", "1", "-"},
    {0x2500, ObjAccess::RW, ObjType::UINT, "Adaptive Filter Function Select", "", "0", "0", "5", "-"},
    {0x2501, ObjAccess::RW, ObjType::UINT, "Notch Filter 1 Frequency", "", "5000", "50", "5000", "Hz"},
    {0x2502, ObjAccess::RW, ObjType::UINT, "Notch Filter 1 Width", "", "1", "1", "100", "-"},
    {0x2503, ObjAccess::RW, ObjType::UINT, "Notch Filter 1 Depth", "", "1", "1", "5", "-"},
    {0x2504, ObjAccess::RW, ObjType::UINT, "Notch Filter 2 Frequency", "", "5000", "50", "5000", "Hz"},
    {0x2505, ObjAccess::RW, ObjType::UINT, "Notch Filter 2 Width", "", "1", "1", "100", "-"},
    {0x2506, ObjAccess::RW, ObjType::UINT, "Notch Filter 2 Depth", "", "1", "1", "5", "-"},
    {0x2507, ObjAccess::RW, ObjType::UINT, "Notch Filter 3 Frequency", "", "5000", "50", "5000", "Hz"},
    {0x2508, ObjAccess::RW, ObjType::UINT, "Notch Filter 3 Width", "", "1", "1", "100", "-"},
    {0x2509, ObjAccess::RW, ObjType::UINT, "Notch Filter 3 Depth", "", "1", "1", "5", "-"},
    {0x250A, ObjAccess::RW, ObjType::UINT, "Notch Filter 4 Frequency", "", "5000", "50", "5000", "Hz"},
    {0x250B, ObjAccess::RW, ObjType::UINT, "Notch Filter 4 Width", "", "1", "1", "100", "-"},
    {0x250C, ObjAccess::RW, ObjType::UINT, "Notch Filter 4 Depth", "", "1", "1", "5", "-"},
    {0x250D, ObjAccess::RW, ObjType::UINT, "On-line Gain Tuning Mode", "", "0", "0", "1", "-"},
    {0x250E, ObjAccess::RW, ObjType::UINT, "System Rigidity for Gain Tuning", "", "5", "1", "20", "-"},
    {0x250F, ObjAccess::RW, ObjType::UINT, "On-line Gain Tuning Adaptation Speed", "", "1", "1", "5", "-"},
    {0x2510, ObjAccess::RW, ObjType::UINT, "Off-line Gain Tuning Direction", "", "0", "0", "1", "-"},
    {0x2511, ObjAccess::RW, ObjType::UINT, "Off-line Gain Tuning Distance", "", "5", "1", "10", "-"},
    {0x2512, ObjAccess::RW, ObjType::UINT, "Disturbance Observer Gain", "", "0", "0", "100", "%"},
    {0x2513, ObjAccess::RW, ObjType::UINT, "Disturbance Observer Filter Time Constant", "", "10", "0", "1000", "0.1ms"},
    {0x2514, ObjAccess::RW, ObjType::UINT, "Current Controller Gain", "", "100", "1", "150", "%"},
    {0x2515, ObjAccess::RW, ObjType::UINT, "Vibration Supression Filter Configuration", "", "0", "0", "5", "-"},
    {0x2516, ObjAccess::RW, ObjType::UINT, "Vibration Supression Filter 1 Frequency", "", "0", "0", "2000", "0.1Hz"},
    {0x2517, ObjAccess::RW, ObjType::UINT, "Vibration Supression Filter 1 Damping", "", "0", "0", "5", "-"},
    {0x2518, ObjAccess::RW, ObjType::UINT, "Vibration Supression Filter 2 Frequency", "", "0", "0", "2000", "0.1Hz"},
    {0x2519, ObjAccess::RW, ObjType::UINT, "Vibration Supression Filter 2 Damping", "", "0", "0", "5", "-"},
    {0x2600, ObjAccess::RO, ObjType::INT, "Feedback Speed", "(Indexing)Feedback Speed", "-", "-", "-", "rpm"},
    {0x2601, ObjAccess::RO, ObjType::INT, "Command Speed", "(Indexing)Command Speed", "-", "-", "-", "rpm"},
    {0x2602, ObjAccess::RO, ObjType::DINT, "Following Error", "", "-", "-", "-", "pulse"},
    {0x2604, ObjAccess::RO, ObjType::INT, "Accumulated Operation Overload", "", "-", "-", "-", "0.10%"},
    {0x2605, ObjAccess::RO, ObjType::INT, "Instantaneous Maximum Operation Overload", "", "-", "-", "-", "0.10%"},
    {0x2606, ObjAccess::RO, ObjType::UINT, "DC-Link Voltage", "", "-", "-", "-", "Volt"},
    {0x2607, ObjAccess::RO, ObjType::INT, "Accumulated Regeneration Overload", "", "-", "-", "-", "0.10%"},
    {0x2608, ObjAccess::RO, ObjType::UDINT, "SingleTurn Data", "", "-", "-", "-", "pulse"},
    {0x260A, ObjAccess::RO, ObjType::UINT, "Mechanical Angle", "", "-", "-", "-", "0.1deg"},
    {0x260B, ObjAccess::RO, ObjType::INT, "Electrical Angle", "", "-", "-", "-", "0.1deg"},
    {0x260C, ObjAccess::RO, ObjType::DINT, "MultiTurn Data", "", "-", "-", "-", "rev"},
    {0x260E, ObjAccess::RO, ObjType::INT, "Drive Temperature 1", "", "-", "-", "-", "-"},
    {0x260F, ObjAccess::RO, ObjType::INT, "Drive Temperature 2", "", "-", "-", "-", "-"},
    {0x2610, ObjAccess::RO, ObjType::INT, "Encoder Temperature", "", "-", "-", "-", "-"},
    {0x2611, ObjAccess::RO, ObjType::UINT, "Motor Rated Speed", "", "-", "-", "-", "rpm"},
    {0x2612, ObjAccess::RO, ObjType::UINT, "Motor Maximum Speed", "", "-", "-", "-", "rpm"},
    {0x2613, ObjAccess::RO, ObjType::UINT, "Drive Rated Current", "", "-", "-", "-", "0.1A"},
    {0x2614, ObjAccess::RO, ObjType::STRING, "FPGA Version", "", "-", "-", "-", "-"},
    {0x2617, ObjAccess::RO, ObjType::UINT, "Hall Signal Display", "", "-", "-", "-", "-"},
    {0x2618, ObjAccess::RO, ObjType::STRING, "Bootloader Version", "", "-", "-", "-", "-"},
    {0x261B, ObjAccess::RO, ObjType::UINT, "Warning Code", "", "-", "-", "-", "-"},
    {0x261C, ObjAccess::RO, ObjType::INT, "Analog Input 1 Value", "", "-", "-", "-", "mV"},
    {0x261D, ObjAccess::RO, ObjType::INT, "Analog Input 2 Value", "", "-", "-", "-", "mV"},
    {0x2623, ObjAccess::RO, ObjType::INT, "RMS Operation Overload", "", "-", "-", "-", "0.1%"},
    {0x2700, ObjAccess::RW, ObjType::UINT, "Procedure Command Code", "", "0", "0", "0xFFFF", "-"},
    {0x2701, ObjAccess::RW, ObjType::UINT, "Procedure Command Argument", "", "0", "0", "0xFFFF", "-"},
    {0x2800, ObjAccess::RW, ObjType::UINT, "[Third Party Motor] Type", "", "0", "0", "1", "-"},
    {0x2801, ObjAccess::RW, ObjType::UINT, "[Third Party Motor] Number of Poles", "", "8", "2", "1000", "-"},
    {0x2802, ObjAccess::RW, ObjType::FP32, "[Third Party Motor] Rated Current", "", "2.89", "-", "-", "Arms"},
    {0x2804, ObjAccess::RW, ObjType::FP32, "[Third Party Motor] Maximum Current", "", "8.67", "-", "-", "Arms"},
    {0x2806, ObjAccess::RW, ObjType::UINT, "[Third Party Motor] Rated Speed", "", "3000", "1", "60000", "rpm"},
    {0x2807, ObjAccess::RW, ObjType::UINT, "[Third Party Motor] Maximum Speed", "", "5000", "1", "60000", "rpm"},
    {0x2808, ObjAccess::RW, ObjType::FP32, "[Third Party Motor] Inertia", "", "0.321", "-", "-", "Kg"},
    {0x280A, ObjAccess::RW, ObjType::FP32, "[Third Party Motor] Torque Constant", "", "0.46", "-", "-", "Kg.m2.10-4"},
    {0x280C, ObjAccess::RW, ObjType::FP32, "[Third Party Motor] Phase Resistance", "", "0.82", "-", "-", "ohm"},
    {0x280E, ObjAccess::RW, ObjType::FP32, "[Third Party Motor] Phase Inductance", "", "3.66", "-", "-", "mH"},
    {0x2810, ObjAccess::RW, ObjType::UINT, "[Third Party Motor] TN Curve Data 1", "", "3000", "1", "60000", "rpm"},
    {0x2812, ObjAccess::RW, ObjType::FP32, "[Third Party Motor] TN Curve Data 2", "", "100", "-", "-", "%"},
    {0x2814, ObjAccess::RW, ObjType::UINT, "[Third Party Motor] Hall Offset", "", "0", "0", "360", "deg"},
    {0x6000, ObjAccess::RO, ObjType::UINT, "Reserved", "", "0", "-", "-", "-"},
    {0x6001, ObjAccess::RO, ObjType::UINT, "Reserved", "", "-", "-", "-", "-"},
    {0x6002, ObjAccess::RO, ObjType::UINT, "Reserved", "", "-", "-", "-", "-"},
    {0x6003, ObjAccess::RW, ObjType::INT, "Quick Stop Option Code", "", "2", "0", "4", "-"},
    {0x6004, ObjAccess::RW, ObjType::INT, "Shutdown Option Code", "", "0", "1", "1", "-"},
    {0x6005, ObjAccess::RW, ObjType::INT, "Disable Operation Option Code", "", "1", "0", "1", "-"},
    {0x6006, ObjAccess::RW, ObjType::INT, "Halt Option Code", "", "0", "0", "4", "-"},
    {0x6007, ObjAccess::RW, ObjType::INT, "Fault Reaction Option Coed", "", "0", "0", "0", "-"},
    {0x6008, ObjAccess::RW, ObjType::SINT, "Modes of Operation", "", "-1", "-1", "10", "-"},
    {0x6009, ObjAccess::RO, ObjType::SINT, "Modes of Operation Display", "", "-", "-", "-", "-"},
    {0x600A, ObjAccess::RO, ObjType::DINT, "Position Demand Valude", "(Indexing)Position Demand Value", "-", "-", "-", "UU"},
    {0x600C, ObjAccess::RO, ObjType::DINT, "Position Actual Internal Value", "", "-", "-", "-", "Pulse"},
    {0x600E, ObjAccess::RO, ObjType::DINT, "Position Actual Value", "(Indexing)Position Actual Value", "-", "-", "-", "UU"},
    {0x6010, ObjAccess::RW, ObjType::UDINT, "Following Error Window", "", "600000", "0", "1073741823", "UU"},
    {0x6012, ObjAccess::RW, ObjType::UINT, "Following Error Timeout", "", "0", "0", "65535", "ms"},
    {0x6013, ObjAccess::RW, ObjType::UDINT, "Position Window", "", "100", "0", "1073741823", "UU"},
    {0x6015, ObjAccess::RW, ObjType::UINT, "Position Window Tim e", "", "0", "0-", "65535", "ms"},
    {0x6016, ObjAccess::RO, ObjType::DINT, "Velocity Demand Value", "(Indexing)Velocity Demand Value", "-", "-", "-", "UU/s"},
    {0x6018, ObjAccess::RO, ObjType::DINT, "Velocity Actual Value", "(Indexing)Velocity Actual Value", "-", "-", "-", "UU/s"},
    {0x601A, ObjAccess::RW, ObjType::UINT, "Velocity Window", "", "20000", "0", "65535", "UU/s"},
    {0x601B, ObjAccess::RW, ObjType::UINT, "Velocity Window Time", "", "0", "0", "65535", "ms"},
    {0x601C, ObjAccess::RW, ObjType::INT, "Target Torque", "", "0", "-5000", "5000", "0.1%"},
    {0x601D, ObjAccess::RW, ObjType::UINT, "Maximum Torque", "", "3000", "0", "5000", "0.1%"},
    {0x601E, ObjAccess::RO, ObjType::INT, "Torque Demand Value", "", "-", "-", "-", "0.1%"},
    {0x601F, ObjAccess::RO, ObjType::UDINT, "Motor Rated Torque", "", "-", "-", "-", "mNm"},
    {0x6021, ObjAccess::RO, ObjType::INT, "Torque Actual Value", "", "-", "-", "-", "0.1%"},
    {0x6022, ObjAccess::RO, ObjType::DINT, "Reserved", "", "-", "-", "-", "-"},
    {0x6024, ObjAccess::RW, ObjType::DINT, "Home Offset", "(Homing)5.Home Offset", "0", "-536870912", "536870911", "UU"},
    {0x6028, ObjAccess::RW, ObjType::DINT, "Software Position Limit (Min)", "", "-10000000000", "-1073741824", "1073741824", "UU"},
    {0x602A, ObjAccess::RW, ObjType::DINT, "Software Position Limit (Max)", "", "10000000000", "-1073741824", "1073741824", "UU"},
    {0x602C, ObjAccess::RO, ObjType::DINT, "Reserved", "", "-", "-", "-", "-"},
    {0x602E, ObjAccess::RO, ObjType::DINT, "Reserved", "", "-", "-", "-", "-"},
    {0x6030, ObjAccess::RO, ObjType::DINT, "Reserved", "", "-", "-", "-", "-"},
    {0x6032, ObjAccess::RO, ObjType::DINT, "Reserved", "", "-", "-", "-", "-"},
    {0x6034, ObjAccess::RW, ObjType::DINT, "Quick Stop Deceleration", "(Homing)6.Deceleration", "200000", "0", "0x7FFFFFFF", "UU/s2"},
    {0x6036, ObjAccess::RO, ObjType::DINT, "Reserved", "", "-", "-", "-", "-"},
    {0x603A, ObjAccess::RW, ObjType::UDINT, "Gear Ratio (Motor revolutions)", "", "1", "0", "0x40000000", "-"},
    {0x603C, ObjAccess::RW, ObjType::UDINT, "Gear Ratio (Shaft revolutions)", "", "1", "0", "0x40000000", "-"},
    {0x603E, ObjAccess::RW, ObjType::INT, "Homing Method", "(Homing)1.Homing Method", "34", "-128", "127", "-"},
    {0x6041, ObjAccess::RW, ObjType::DINT, "Homing Speed (switch)", "(Homing)2.Switch Search Velocity", "500000", "0", "0x40000000", "UU/s"},
    {0x6043, ObjAccess::RW, ObjType::DINT, "Homing Speed (zero)", "(Homing)3.Marker Search Velocity", "100000", "0", "0x40000000", "UU/s"},
    {0x6045, ObjAccess::RW, ObjType::UDINT, "Homing Acceleration", "(Homing)4.Acceleration", "200000", "0", "0x40000000", "UU/s2"},
    {0x6047, ObjAccess::RO, ObjType::DINT, "Reserved", "", "-", "-", "-", "-"},
    {0x6049, ObjAccess::RW, ObjType::DINT, "Velocity Offset", "", "0", "-2147483648", "2147483648", "UU/s"},
    {0x604B, ObjAccess::RW, ObjType::INT, "Torque Offset", "", "0", "-5000", "5000", "0.1%"},
    {0x604C, ObjAccess::RW, ObjType::UINT, "Touch Probe Function", "", "0x0033", "0", "0Xffff", "-"},
    {0x604D, ObjAccess::RO, ObjType::UINT, "Touch Probe Status", "", "-", "-", "-", "-"},
    {0x604E, ObjAccess::RO, ObjType::DINT, "Touch Prove 1 Positive Edge Position Value", "", "-", "-", "-", "UU"},
    {0x6050, ObjAccess::RO, ObjType::DINT, "Touch Prove 1 Negative Edge Position Value", "", "-", "-", "-", "UU"},
    {0x6052, ObjAccess::RO, ObjType::DINT, "Touch Prove 2 Positive Edge Position Value", "", "-", "-", "-", "UU"},
    {0x6054, ObjAccess::RO, ObjType::DINT, "Touch Prove 2 Negative Edge Position Value", "", "-", "-", "-", "UU"},
    {0x605C, ObjAccess::RW, ObjType::UINT, "Positive Torque Limit Value", "", "1000", "0", "5000", "0.1%"},
    {0x605D, ObjAccess::RW, ObjType::UINT, "Negative Torque Limit Value", "", "1000", "0", "5000", "0.1%"},
    {0x605E, ObjAccess::RO, ObjType::DINT, "Following Error Actual Value", "", "-", "-", "-", "UU"},
    {0x6060, ObjAccess::RO, ObjType::DINT, "Position Demand Internal Value", "", "-", "-", "-", "Pulse"},
    {0x6062, ObjAccess::RO, ObjType::UDINT, "Digital Inputs", "", "-", "-", "-", "-"},
    {0x6066, ObjAccess::RW, ObjType::DINT, "Disital Outputs (Physical)", "", "0", "0", "0xFFFFFFFF", "-"},
    {0x6068, ObjAccess::RW, ObjType::DINT, "Disital Outputs (Bit mask)", "", "0", "0", "0xFFFFFFFF", "-"},
    {0x606A, ObjAccess::RW, ObjType::DINT, "Target Velocity", "", "0", "-2147483648", "2147483648", "UU/s"},
    {0x606C, ObjAccess::RO, ObjType::UDINT, "Supported Drive Modes", "", "0x000003AD", "-", "-", "-"},
    {0x3000, ObjAccess::RW, ObjType::UINT, "Control Mode", "", "1", "0", "9", "-"},
    {0x3001, ObjAccess::RW, ObjType::UINT, "Coordinate Select", "", "0", "0", "1", "-"},
    {0x3002, ObjAccess::RW, ObjType::UINT, "Baud Rate Select", "", "3", "0", "3", "-"},
    {0x3003, ObjAccess::RW, ObjType::UINT, "Pulse Input Logic Select", "", "0", "0", "5", "-"},
    {0x3004, ObjAccess::RW, ObjType::UINT, "Pulse Input Filter Select", "", "0", "0", "4", "-"},
    {0x3005, ObjAccess::RW, ObjType::UINT, "PCLEAR Mode Select", "", "0", "0", "2", "-"},
    {0x3006, ObjAccess::RW, ObjType::UDINT, "Encoder Ouptput Pulse", "", "10000", "0", "2147483647", "-"},
    {0x3008, ObjAccess::RW, ObjType::UINT, "Encoder Output Mode", "", "0", "0", "1", "-"},
    {0x3009, ObjAccess::RW, ObjType::UINT, "Start Index Number(0~63)", "", "0", "0", "64", "-"},
    {0x300A, ObjAccess::RW, ObjType::UINT, "Index Buffer Mode", "", "0", "0", "1", "-"},
    {0x300B, ObjAccess::RW, ObjType::UINT, "IOUT Configuration", "", "0", "0", "5", "-"},
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
    {0x3111, ObjAccess::RW, ObjType::UINT16, "Action", "", "-", "0", "2", "-"}
};