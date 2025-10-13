#ifndef FS_I6X_H
#define FS_I6X_H
#include <stdint.h>
#include "sbus.h"

#define REMOTER_DEADLINE 10  //摇杆死区
#define RC_CH_VALUE_MIN ((uint16_t)364)
#define RC_CH_VALUE_OFFSET ((uint16_t)1024)
#define RC_CH_VALUE_MAX ((uint16_t)1684)

typedef struct{
    SBUS_Data_s rc;
    struct{
        int16_t right_x;
        int16_t right_y;
        int16_t left_y;
        int16_t left_x;
    }joy;
    struct{
        int16_t a;
        int16_t b;
    }var;
    struct{
        uint8_t a;
        uint8_t b;
        uint8_t c;
        uint8_t d;
    }sw;
}Sbus_Data_s;

#endif //FS_I6X_H