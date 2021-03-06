#include "CoilParameters.h"
#include "RegisterParameters.h"

#include "ObjectDictionary.h"

#include <stdio.h>
#ifndef SIM_MODBUS
#include <modbus.h>
#else
#include "sim_modbus.h"
#endif

#include "L7P.h"
#include "main.h"

int32_t getObjType(int32_t address) {
    for (int i=0; i<REGISTER_NUMBER; i++) {
        if (registerParameters[i].address == address) {
            return (int32_t)registerParameters[i].type;
        }
    }

    return -1;
}

int32_t setAxisParameter(modbus_t* ctx, int32_t id, int32_t index, int32_t value, OnOff read_check) {
    static int32_t objType = 0;
    static uint16_t write_data[MAX_DATA_SIZE] = {'\0', };
    static int32_t ret = 0;
    static int32_t getValue = 0;

    objType = getObjType(index);
    
    *((int32_t*)write_data) = value;

    if (modbus_set_slave(ctx, id) == -1) {
        reprintf(ScreenOutput::ERROR, "[%s{%s}(%d)] : slave: #%d, index: 0x%04x, error msg: %s \n",
            __FILENAME__, __FUNCTION__, __LINE__, id, index, modbus_strerror(errno));

        return -1;
    } else {
        ret = modbus_write_registers(ctx, index, objType, write_data);

        if (ret < 0) {
            reprintf(ScreenOutput::ERROR, "[%s{%s}(%d)] : modbus_write_registers[0x%04x] error: %s \n",
                __FILENAME__, __FUNCTION__, __LINE__, index, modbus_strerror(errno));

            return -1;
        }
        if ((int32_t)read_check) {
            ret = getAxisParameter(ctx, id, index, &getValue);

            if (value!=getValue && ret != -1) {
                reprintf(ScreenOutput::ERROR, "[%s{%s}(%d)] : getAxisParameter[0x%04x] error: %s \n",
                    __FILENAME__, __FUNCTION__, __LINE__, index, modbus_strerror(errno));

                return -1;
            }
        }
    }

    // modbus_write_register is success : 1, fail : -1 and set errno
    return 1;
}

int32_t getAxisParameter(modbus_t* ctx, int32_t id, int32_t index, int32_t* value) {
    static int32_t objType = 0;
    static uint16_t read_data[MAX_DATA_SIZE] = {'\0', };
    static int32_t ret = 0;

    objType = getObjType(index);

    *((int32_t*)read_data) = 0x00000000;

    if (modbus_set_slave(ctx, id) == -1) {
        reprintf(ScreenOutput::ERROR, "[%s{%s}(%d)] : slave: #%d, index: 0x%04x, error msg: %s \n",
            __FILENAME__, __FUNCTION__, __LINE__, id, index, modbus_strerror(errno));

        return -1;
    } else {
        ret = modbus_read_registers(ctx, index, objType, read_data);

        if (ret >= 0 && ret == objType) {
            *value = *((int32_t*)read_data);
        } else {
            reprintf(ScreenOutput::ERROR, "[%s{%s}(%d)] : modbus_read_registers[0x%04x] error: %s \n",
                __FILENAME__, __FUNCTION__, __LINE__, index, modbus_strerror(errno));

            return -1;
        }
    }

    // modbus_read_registers is success : >=0, fail : -1 and set errno
    return ret;
}