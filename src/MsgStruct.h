#pragma once

#include <ros/ros.h>

class StatusInputStruct {
    uint8_t p_ot; uint8_t n_ot; uint8_t home; uint8_t stop; uint8_t pcon; uint8_t gain2; uint8_t p_cl; uint8_t n_cl;
    uint8_t mode; uint8_t reserved1; uint8_t emg; uint8_t a_rst; uint8_t sv_on; uint8_t spd1_lvsf1; uint8_t spd2_lvsf2; uint8_t spd3;
    uint8_t start; uint8_t pause; uint8_t regt; uint8_t hstart; uint8_t isel0; uint8_t isel1; uint8_t isel2; uint8_t isel3;
    uint8_t isel4; uint8_t isel5; uint8_t absrq; uint8_t jstart; uint8_t jdir; uint8_t pclear; uint8_t aovr; uint8_t reserved2;
};

class StatusOutputStruct {
    uint8_t brake; uint8_t alarm; uint8_t ready; uint8_t zspd; uint8_t inpos1; uint8_t tlmt; uint8_t vlmt; uint8_t inspd;
    uint8_t warn; uint8_t tgon; uint8_t reserved1; uint8_t reserved2; uint8_t reserved3; uint8_t reserved4; uint8_t reserved5; uint8_t reserved6;
    uint8_t org; uint8_t eos; uint8_t iout0; uint8_t iout1; uint8_t iout2; uint8_t iout3; uint8_t iout4; uint8_t iout5;
    uint8_t reserved7; uint8_t reserved8; uint8_t reserved9; uint8_t reserved10; uint8_t reserved11; uint8_t reserved12; uint8_t reserved13; uint8_t reserved14;
};

class StatusStruct {
    StatusInputStruct input;
    StatusOutputStruct output;
};

class AxisStruct {
    StatusStruct status;
    int32_t position;
    int32_t speed;
};