#pragma once

#include <ros/ros.h>

#include "ObjectDictionary.h"

struct CoilParameter {
    int32_t address;
    ObjAccess access;
    const char* subject;
    const char* description;
};

#define COIL_NUMBER 64
extern const CoilParameter coilParameters[] = {
    {0x0000, ObjAccess::RW, "POT", "정방향(CCW) 회전금지"},
    {0x0001, ObjAccess::RW, "NOT", "역방향(CCW) 회전금지"},
    {0x0002, ObjAccess::RW, "HOME", "원점 센서"},
    {0x0003, ObjAccess::RW, "STOP", "서보 정지"},
    {0x0004, ObjAccess::RW, "PCON", "P 제어 동작(PI->P)"},
    {0x0005, ObjAccess::RW, "GAIN2", "게인 1, 2 전환(속도제어 게인1->2)"},
    {0x0006, ObjAccess::RW, "P_CL", "정방향 토크제한"},
    {0x0007, ObjAccess::RW, "N_CL", "역방향 토크제한"},
    {0x0008, ObjAccess::RW, "MODE", "운전모드 전환"},
    {0x0009, ObjAccess::RW, "Reserved", ""},
    {0x000A, ObjAccess::RW, "EMG", "비상정지"},
    {0x000B, ObjAccess::RW, "A_RST", "알람 리셋"},
    {0x000C, ObjAccess::RW, "SV_ON", "서보 온"},
    {0x000D, ObjAccess::RW, "SPD1/LVSF1", "다단속도1/진동제어필터1"},
    {0x000E, ObjAccess::RW, "SPD2/LVSF2", "다단속도2/진동제어필터2"},
    {0x000F, ObjAccess::RW, "SPD3", "다단속도3"},
    {0x0010, ObjAccess::RW, "START", "운전 개시"},
    {0x0011, ObjAccess::RW, "PAUSE", "일시 정지"},
    {0x0012, ObjAccess::RW, "REGT", "센서후 운전"},
    {0x0013, ObjAccess::RW, "HSTART", "원점 운전 개시"},
    {0x0014, ObjAccess::RW, "ISEL0", "인덱스 선택 0비트"},
    {0x0015, ObjAccess::RW, "ISEL1", "인덱스 선택 1비트"},
    {0x0016, ObjAccess::RW, "ISEL2", "인덱스 선택 2비트"},
    {0x0017, ObjAccess::RW, "ISEL3", "인덱스 선택 3비트"},
    {0x0018, ObjAccess::RW, "ISEL4", "인덱스 선택 4비트"},
    {0x0019, ObjAccess::RW, "ISEL5", "인덱스 선택 5비트"},
    {0x001A, ObjAccess::RW, "ABSRQ", "절대위치 데이터 요구"},
    {0x001B, ObjAccess::RW, "JSTART", "조그 운전"},
    {0x001C, ObjAccess::RW, "JDIR", "조그 회전방향 선택"},
    {0x001D, ObjAccess::RW, "PCLEAR", "입력펄스 클리어"},
    {0x001E, ObjAccess::RW, "AOVR", "속도 오버라이드 선택"},
    {0x001F, ObjAccess::RW, "Reserved", ""},
    {0x0020, ObjAccess::RO, "BRAKE", "브레이크"},
    {0x0021, ObjAccess::RO, "ALARM", "서보 알람"},
    {0x0022, ObjAccess::RO, "READY", "서보레디"},
    {0x0023, ObjAccess::RO, "ZSPD", "영 속도 도달 완료"},
    {0x0024, ObjAccess::RO, "INPOS1", "위치 도달 완료 1"},
    {0x0025, ObjAccess::RO, "TLMT", "토크 제한"},
    {0x0026, ObjAccess::RO, "VLMT", "속도 제한"},
    {0x0027, ObjAccess::RO, "INSPD", "속도 도달 완료"},
    {0x0028, ObjAccess::RO, "WARN", "서보 경고"},
    {0x0029, ObjAccess::RO, "TGON", "회전 검출"},
    {0x002A, ObjAccess::RO, "Reserved", ""},
    {0x002B, ObjAccess::RO, "Reserved", ""},
    {0x002C, ObjAccess::RO, "Reserved", ""},
    {0x002D, ObjAccess::RO, "Reserved", ""},
    {0x002E, ObjAccess::RO, "Reserved", ""},
    {0x002F, ObjAccess::RO, "Reserved", ""},
    {0x0030, ObjAccess::RO, "ORG", "원점 운전 완료"},
    {0x0031, ObjAccess::RO, "EOS", "운전 완료"},
    {0x0032, ObjAccess::RO, "IOUT0", "인덱스 출력 0비트"},
    {0x0033, ObjAccess::RO, "IOUT1", "인덱스 출력 1비트"},
    {0x0034, ObjAccess::RO, "IOUT2", "인덱스 출력 2비트"},
    {0x0035, ObjAccess::RO, "IOUT3", "인덱스 출력 3비트"},
    {0x0036, ObjAccess::RO, "IOUT4", "인덱스 출력 4비트"},
    {0x0037, ObjAccess::RO, "IOUT5", "인덱스 출력 5비트"},
    {0x0038, ObjAccess::RO, "Reserved", ""},
    {0x0039, ObjAccess::RO, "Reserved", ""},
    {0x003A, ObjAccess::RO, "Reserved", ""},
    {0x003B, ObjAccess::RO, "Reserved", ""},
    {0x003C, ObjAccess::RO, "Reserved", ""},
    {0x003D, ObjAccess::RO, "Reserved", ""},
    {0x003E, ObjAccess::RO, "Reserved", ""},
    {0x003F, ObjAccess::RO, "Reserved", ""}
};