#pragma once

#define ACT_POS 0x600E
#define ACT_SPD 0x6018

enum class ObjType {
    Unknown = 0, FP32 = 0, STRING = 0,
    DINT = 2, INT32 = 2, UDINT = 2,
    INT = 1, SINT = 1, UINT = 1, UINT16 = 1, USINT = 1,
};

enum class ObjAccess {
    RW, RO
};

int32_t getObjType(int32_t address);