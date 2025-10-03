#ifndef EMBEDDED_FRAMEWORK_MOTOR_DJI_H
#define EMBEDDED_FRAMEWORK_MOTOR_DJI_H
#include "motor_dji.h"

#include <string.h>
#include <stdbool.h>

#include "bsp_log.h"
#include "bsp_can.h"
#include "bsp_fdcan.h"

/* 私有类型定义 -----------------------------------------------------------------*/
/**
 * @brief 大疆电机类型枚举
 */
typedef enum{
    GM6020 = 0,
    M3508 = 1,
    M2006 = 2,
}DJI_MotorType_e;

/**
 * @brief 大疆电机控制帧标识符枚举
 */
tpedef enum{
    GM_CTRL_1_TO_4_ID = 0X1FE,
    GM_CTRL_5_TO_7_ID = 0X2FE,
    M_CTRL_1_TO_4_ID = 0X200,
    M_CTRL_5_TO_8_ID = 0X1FF,
}DJI_MotorControlID_e;

/**
 * @brief 大疆电机状态枚举
 */
typedef enum{
    DJI_MOTOR_MISSING = 0,  //电机失联
    DJI_MOTOR_ONLINE = 1,   //电机在线
	DJI_
    DJI_MOTOR_OVERHEAT = 2, //电机过热
}Dji_Motor_State_e;

typedef struct{
    /* 原始反馈数据 */
    int16_t rotor_angle; //转子角度
    int16_t rotor_speed; //转子速度
    int16_t torque_current; //转矩电流
    int8_t  rotor_temp; //电机温度
    /* 计算得到的数据 */

    /* 其他数据 */
    uint16_t rx_cnt; //can接收计数
    uint16_t rx_freq; //can接受频率
	uint16_t tx_cnt; //can发送计数
	uint16_t tx_freq; //can发送频率
    uint16_t temp_overheat_cnt; //过热计数
}DJI_Motor_message_s;

#endif //EMBEDDED_FRAMEWORK_MOTOR_DJI_H