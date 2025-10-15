#ifndef FS_I6X_H
#define FS_I6X_H
#include <stdint.h>
#include "sbus.h"
#include "bsp_usart.h"
#include "module_typedef.h"
#include "watch_dog.h"
#include "basic_math.h"
#define REMOTER_DEADLINE 10  //摇杆死区
#define RC_CH_VALUE_MIN ((uint16_t)364)
#define RC_CH_VALUE_OFFSET ((uint16_t)1024)
#define RC_CH_VALUE_MAX ((uint16_t)1684)

#define RC_JOY_TO_VAL_COFE 1.0f//0.006377f  //5/784
#define RC_JOY_TO_PI_COFE  0.0040071f  //PI/784
typedef struct{
    RampInstance_s x_ramp;
    RampInstance_s y_ramp;
    RampInstance_s pitch_ramp;
    RampInstance_s yaw_ramp;
    RampInstance_s fine_pitch;
    RampInstance_s fine_yaw;
}RcData_s;
typedef struct{
    SbusData_s rc;
    Frequency_s rx_freq;
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
    RcData_s rc_data;
}I6xData_s;

typedef struct{
    I6xData_s data;
    UsartInstance_s *usart_instance;
    WatchDogInstance_s *watchdog_instance;
}I6xInstance_s;

/**
 * @brief 注册FS-I6X遥控器实例
 * @param huart USART句柄
 * @return I6xInstance_s* 遥控器实例指针,失败返回NULL
 */
I6xInstance_s* I6x_Register(UART_HandleTypeDef *huart);
#endif //FS_I6X_H