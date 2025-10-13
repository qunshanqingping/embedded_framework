#ifndef OLD_SBUS_H
#define OLD_SBUS_H
#include <stdint.h>
#include "bsp_usart.h"
#include "stdbool.h"
#define SBUS_SIZE 25
#define size_temp 18
#define SBUS_SWIRCH_MIDLE_VAL0 0x0400

typedef struct{
    bool online;
     struct
    {
        int16_t ch[10];
    } rc;
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

typedef struct{
    Sbus_Data_s remote_control;
    UsartInstance_s *usart_instance;
}I6xInstance_s;
I6xInstance_s* Sbus_Register(UART_HandleTypeDef *huart);
#endif //OLD_SBUS_H
