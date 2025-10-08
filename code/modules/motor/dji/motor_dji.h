#ifndef MOTOR_DJI_H
#define MOTOR_DJI_H

#include <string.h>
#include <stdbool.h>

#include "plf_log.h"
#include "bsp_can.h"
#include "bsp_fdcan.h"

/* 私有类型定义 -----------------------------------------------------------------*/
/**
 * @brief 大疆电机类型枚举
 */
typedef enum {
    GM6020 = 0,
    M3508 = 1,
    M2006 = 2,
} DJI_MotorType_e;

typedef enum {
    GM_RX_ID = 0X204,
    M_RX_ID = 0X200,
};
/**
 * @brief 大疆电机控制帧标识符枚举
 */
typedef enum {
    GM_CTRL_1_TO_4_ID = 0X1FE, //GM系列1-4号电机控制帧标识符
    GM_CTRL_5_TO_7_ID = 0X2FE, //GM系列5-7号电机控制帧标识符
    M_CTRL_1_TO_4_ID = 0X200,  //M系列1-4号电机控制帧标识符
    M_CTRL_5_TO_8_ID = 0X1FF,  //M系列5-8号电机控制帧标识符
} DJI_MotorControlID_e;

/**
 * @brief 大疆电机状态枚举
 */
typedef enum {
    DJI_MOTOR_MISSING = 0,    //电机失联
    DJI_MOTOR_ONLINE = 1,     //电机在线
    DJI_MOTOR_RX_ERROR = 2,   //电机接收异常
    DJI_MOTOR_TX_ERROR = 3,   //电机发送异常
    DJI_MOTOR_OVERHEAT = 4,   //电机过热
} Dji_Motor_State_e;

typedef struct {
    /* 原始反馈数据 */
    uint16_t rotor_angle;     //转子角度
    int16_t rotor_speed;      //转子速度
    int16_t torque_current;   //转矩电流
    uint8_t rotor_temp;       //电机温度
    /* 计算得到的数据 */
    float out_position;                 // 电机输出轴角度(-PI~PI)
    float out_velocity;                 // 电机输出轴速度(rpm)
    int16_t out_current;        //
    int8_t rotor_rounds;        //转子圈数
    int16_t out_rounds;         //电机输出轴圈数
    /* 状态数据 */
    uint16_t rx_cnt;            //can接收计数
    uint16_t rx_freq;           //can接受频率
    uint16_t tx_cnt;            //can发送计数
    uint16_t tx_freq;           //can发送频率
    uint16_t temp_overheat_cnt; //过热计数
} DJI_Motor_message_s;

typedef struct {
    char *topic_name;
    DJI_MotorType_e type;                  // 电机类型
    uint8_t id;                           // 电机ID(0~8)
    uint8_t can_number;
    uint8_t reduction_ratio;              // 减速比
    CanInitConfig_s can_config;
} DjiMotorInitConfig_s;

typedef struct DjiMotorInstance_s {
    char *topic_name;
    uint8_t id;

    Dji_Motor_State_e state;
    DJI_Motor_message_s message;

    CanInstance_s *can_instance;
} DjiMotorInstance_s;

#endif //MOTOR_DJI_H
