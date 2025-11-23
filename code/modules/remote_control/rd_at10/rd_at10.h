#ifndef RD_AT10_H
#define RD_AT10_H
#include <stdint.h>
#include "sbus.h"
#include "bsp_usart.h"
#include "module_typedef.h"
#include "watch_dog.h"
#include "basic_math.h"

#define RD_REMOTER_DEADLINE 10  //摇杆死区
#define RD_RC_CH_VALUE_MIN ((uint16_t)0x0132)
#define RD_RC_CH_VALUE_OFFSET ((uint16_t)0x03E8)
#define RD_RC_CH_VALUE_MAX ((uint16_t)0x069E)

#define RD_RC_JOY_TO_VAL_COFE 1.0f//0.006377f  //5/784
#define RD_RC_JOY_TO_PI_COFE  0.0040071f  //PI/784

typedef struct
{
    struct
    {
        uint8_t a;
        uint8_t b;
        uint8_t c;
        uint8_t d;
        uint8_t e;
        uint8_t f;
        uint8_t g;
        uint8_t h;
    } sw;
    Frequency_s rx_freq;
    SbusData_s rc;
    struct
    {
        int16_t right_x;
        int16_t right_y;
        int16_t left_y;
        int16_t left_x;
    } joy;
} At10Data_s;
typedef struct{
    At10Data_s data;
    UsartInstance_s *usart_instance;
    WatchDogInstance_s *watchdog_instance;
}At10Instance_s;
/**
 * @brief 注册RD-AT10遥控器实例
 * @param huart USART句柄
 * @return At10Instance_s* 遥控器实例指针,失败返回NULL
 */
At10Instance_s* At10_Register(UART_HandleTypeDef *huart);
#endif //RD_AT10_H
