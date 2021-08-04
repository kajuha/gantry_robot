#include "CoilParameters.h"
#include "RegisterParameters.h"

#include "ObjectDictionary.h"

#include <stdio.h>

#include "L7P.h"

int32_t getObjType(int32_t address) {
    for (int i=0; i<REGISTER_NUMBER; i++) {
        if (registerParameters[i].address == address) {
            return (int32_t)registerParameters[i].type;
        }
    }

    return 0;
}