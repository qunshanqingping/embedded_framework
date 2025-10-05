#ifndef CHASSIS__CALC_H
#define CHASSIS__CALC_H

#include "dev_motor_dji.h"
#include "bsp_can.h"
#include "alg_pid.h"

/**
 * @brief 底盘电机类型枚举
 */
typedef enum {
    Omni_Wheel = 0,     ///< 全向轮
    Mecanum_Wheel = 1,  ///< 麦克纳姆轮
    Steering_Wheel = 2, ///< 舵轮
}CarMotorType;

/**
 * @brief 底盘工作模式枚举
 */
typedef enum {
    CHASSIS_NORMAL = 0,             ///< 底盘独立行走模式
    CHASSIS_FOLLOW_GIMBAL = 1,      ///< 底盘跟随云台行走模式
    CHASSIS_GYROSCOPE = 2,          ///< 小陀螺模式(原地旋转)
    CHASSIS_SLOW = 3,               ///< 静止/低速模式
    //CHASSIS_ROSHAN = 4,           ///< 打符模式(保留)
    //CHASSIS_SZUPUP = 5,           ///< 爬坡模式(保留)
    //CHASSIS_FLY=6,                ///< 飞坡模式(保留)
    //CHASSIS_GYROSCOPE_BIANSU=7    ///< 变速小陀螺模式
} ChassisAction;

/**
 * @brief 底盘速度结构体
 */
typedef struct {
    float Vx;                     ///< 底盘X轴速度(m/s)
    float Vy;                     ///< 底盘Y轴速度(m/s)
    float Vw;                     ///< 底盘旋转角速度(rad/s)
}Chassis_Speed;

/**
 * @brief 绝对坐标系底盘速度结构体
 */
typedef struct {
    float absolute_chassis_Vx;    ///< 绝对坐标系下X轴速度(m/s)
    float absolute_chassis_Vy;    ///< 绝对坐标系下Y轴速度(m/s)
    float absolute_chassis_Vw;    ///< 绝对坐标系下旋转角速度(rad/s)
}absolute_chassis_speed;

/**
 * @brief 底盘初始化配置结构体
 */
typedef struct {
    CarMotorType type;                      ///< 底盘类型
    DjiMotorInitConfig_s motor_config;      ///< 电机初始化配置,配置信息为id=1的电机信息
    PidInitConfig_s gimbal_follow_pid_config; ///< 云台跟随PID配置
    uint8_t motor_id[4];                ///< 电机ID数组(1号为主电机)
    float wheel_radius;                     ///< 轮子半径(m)
    float length_a;                         ///< 底盘前后半长度(m)
    float length_b;                         ///< 底盘左右半长度(m) 
    float chassis_radius;                   ///< 底盘旋转半径(m)
    float gimbal_yaw_zero;                  ///< 云台偏航零点角度(rad)
    float gimbal_yaw_half;                  ///< 云台俯仰零点角度(rad)
}ChassisInitConfig_s;

/**
 * @brief 底盘实例结构体
 */
typedef struct {
    CarMotorType type;                      ///< 底盘类型
    float wheel_radius;                     ///< 轮子半径(m)
    float length_a;                         ///< 底盘前后半长度(m)
    float length_b;                         ///< 底盘左右半长度(m) 
    float chassis_radius;                   ///< 底盘旋转半径(m)
    float gimbal_yaw_angle;                 ///< 云台当前偏航角度(rad)
    float gimbal_yaw_zero;                  ///< 云台偏航零点角度(rad)
    float gimbal_yaw_half;                  ///< 云台俯仰零点角度(rad) 
    ChassisAction actChassis;               ///< 当前底盘工作模式
    Chassis_Speed Chassis_speed;            ///< 底盘速度控制量
    absolute_chassis_speed absolute_chassis_speed; ///< 绝对坐标系底盘速度
    PidInstance_s *gimbal_follow_pid;       ///< 云台跟随PID实例指针
    DjiMotorInstance_s *chassis_motor[4];   ///< 底盘驱动电机实例数组(1号为主电机)
    DjiMotorInstance_s *chassis_Steering_motor[4]; ///< 舵轮转向电机数组
    float out_speed[4];                     ///< 电机输出速度数组(rpm)
    float out_angle[4];                     ///< 电机输出角度数组(rad)
}ChassisInstance_s;

/**
 * @brief 注册并初始化底盘实例
 * @param Chassis_config 底盘初始化配置结构体指针
 * @return 成功返回底盘实例指针，失败返回NULL
 * @note 需要先初始化CAN总线
 * @date 2025-07-03
 */
ChassisInstance_s *Chassis_Register(ChassisInitConfig_s *Chassis_config);

/**
 * @brief 底盘运动控制函数
 * @param chassis 底盘实例指针
 * @return 控制成功返回true，失败返回false
 * @note 会根据当前模式进行相应控制
 * @date 2025-07-09
 */
bool Chassis_Control(ChassisInstance_s *Chassis);

/**
 * @brief 底盘计算函数，根据传入底盘实例的Chassis_speed计算实际电机速度
 * @param Chassis 底盘实例指针
 * @return 计算是否成功
 */
bool Chassis_Calc(ChassisInstance_s *Chassis);

/**
 * @brief 底盘模式选择函数
 * @param ChassisAction 底盘实例指针
 * @note 根据当前模式调整实际控制参数
 * @date 2025-07-09
 */
void Chassis_Mode_Choose(ChassisInstance_s* Chassis);
#endif
