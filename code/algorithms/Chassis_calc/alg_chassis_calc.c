#include "alg_chassis_calc.h"
#include "math.h"
#include "main.h"
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "FreeRTOS.h"

/**
 * @brief 检查底盘配置参数合法性
 * @param config 底盘配置结构体指针
 * @return 配置合法返回true，否则返回false
 * @note 根据底盘类型检查关键参数是否有效
 * @date 2025-07-03
 */
static bool Chassis_judgment(ChassisInitConfig_s *config)
{
    // 全向轮底盘必须设置旋转半径
    if(config->type == Omni_Wheel && config->chassis_radius == 0) 
    {
        return false;
    }
    // 麦轮/舵轮底盘必须设置几何尺寸
    if((config->type == Mecanum_Wheel || config->type == Steering_Wheel) && 
       (config->length_a == 0 || config->length_b == 0)) 
    {
        return false;
    }
    return true;
}

/**
 * @brief 注册并初始化底盘实例
 * @param Chassis_config 底盘初始化配置结构体指针
 * @return 成功返回底盘实例指针，失败返回NULL
 * @note 根据底盘类型初始化不同数量的电机
 * @date 2025-07-03
 */
ChassisInstance_s *Chassis_Register(ChassisInitConfig_s *Chassis_config){
    // 分配内存并初始化
    ChassisInstance_s *Chassis_Instance = (ChassisInstance_s *)pvPortMalloc(sizeof(ChassisInstance_s));
    memset(Chassis_Instance, 0, sizeof(ChassisInstance_s));
	Chassis_Instance->type=Chassis_config->type;
	Chassis_Instance->gimbal_follow_pid = Pid_Register(&Chassis_config->gimbal_follow_pid_config);
	Chassis_Instance->gimbal_yaw_half = Chassis_config->gimbal_yaw_half;
	Chassis_Instance->gimbal_yaw_zero = Chassis_config->gimbal_yaw_zero;
	Chassis_Instance->chassis_radius=Chassis_config->chassis_radius;
	Chassis_Instance->wheel_radius=Chassis_config->wheel_radius;
	Chassis_Instance->length_a=Chassis_config->length_a;
	Chassis_Instance->length_b=Chassis_config->length_b;
    
    // 全向轮/麦轮底盘初始化
    if(Chassis_Instance->type == Omni_Wheel || Chassis_Instance->type == Mecanum_Wheel)
    {   
        uint32_t rx_id_temp = Chassis_config->motor_config.can_config.rx_id - 1;
        uint32_t id_temp = Chassis_config->motor_config.id - 1;
        for(int i = 0; i < 4; i++)
        {
            Chassis_config->motor_config.id = id_temp + Chassis_config->motor_id[i];
            Chassis_config->motor_config.can_config.rx_id = rx_id_temp + Chassis_config->motor_id[i];
            Chassis_Instance->chassis_motor[i] = Motor_Dji_Register(&Chassis_config->motor_config);
        }
    }
    // 舵轮底盘初始化
    else if(Chassis_Instance->type == Steering_Wheel)
    {   
        uint32_t rx_id_temp = Chassis_config->motor_config.can_config.rx_id;
        uint32_t id_temp = Chassis_config->motor_config.id;
        
        // 初始化驱动电机 (CAN1)
        for(int i = 0; i < 4; i++)
        {
            Chassis_config->motor_config.id = id_temp + i;
            Chassis_config->motor_config.can_config.can_number = 1;
            Chassis_config->motor_config.can_config.rx_id = rx_id_temp + i;
            Chassis_Instance->chassis_motor[i] = Motor_Dji_Register(&Chassis_config->motor_config);
        }
        
        // 初始化转向电机 (CAN2)
        for(int i = 0; i < 4; i++)
        {   
            Chassis_config->motor_config.id = id_temp + i;
            Chassis_config->motor_config.can_config.can_number = 2;
            Chassis_config->motor_config.can_config.rx_id = rx_id_temp + i;
            Chassis_Instance->chassis_Steering_motor[i] = Motor_Dji_Register(&Chassis_config->motor_config);
        }
    }
    
    // 检查初始化是否成功
    if (Chassis_Instance == NULL && Chassis_judgment(Chassis_config)) {
        vPortFree(Chassis_Instance);
        return NULL;
    }
    return Chassis_Instance;
}

/**
 * @brief 底盘运动学逆解计算
 * @param chassis 底盘实例指针
 * @note 根据底盘类型计算各轮速度和角度
 * @date 2025-07-09
 */
static void Chassis_IK_Calc(ChassisInstance_s *Chassis)
{
    switch (Chassis->type)
    {
    case Omni_Wheel:  // 全向轮逆解
        Chassis->out_speed[0] = (  0.707f * Chassis->absolute_chassis_speed.absolute_chassis_Vx  - 0.707f * Chassis->absolute_chassis_speed.absolute_chassis_Vy + Chassis->absolute_chassis_speed.absolute_chassis_Vw * Chassis->chassis_radius) * 30.0f/(3.14f * Chassis->wheel_radius);
        Chassis->out_speed[1] = (  0.707f * Chassis->absolute_chassis_speed.absolute_chassis_Vx + 0.707f * Chassis->absolute_chassis_speed.absolute_chassis_Vy + Chassis->absolute_chassis_speed.absolute_chassis_Vw * Chassis->chassis_radius) * 30.0f/(3.14f * Chassis->wheel_radius);
        Chassis->out_speed[2] = ( -0.707f * Chassis->absolute_chassis_speed.absolute_chassis_Vx + 0.707f * Chassis->absolute_chassis_speed.absolute_chassis_Vy + Chassis->absolute_chassis_speed.absolute_chassis_Vw * Chassis->chassis_radius) * 30.0f/(3.14f * Chassis->wheel_radius);
        Chassis->out_speed[3] = ( -0.707f * Chassis->absolute_chassis_speed.absolute_chassis_Vx - 0.707f * Chassis->absolute_chassis_speed.absolute_chassis_Vy + Chassis->absolute_chassis_speed.absolute_chassis_Vw * Chassis->chassis_radius) * 30.0f /(3.14f * Chassis->wheel_radius);
        break;
    
    case Mecanum_Wheel:  // 麦轮逆解
        Chassis->out_speed[0] = (-Chassis->absolute_chassis_speed.absolute_chassis_Vy + Chassis->absolute_chassis_speed.absolute_chassis_Vx - Chassis->absolute_chassis_speed.absolute_chassis_Vw * (Chassis->length_a + Chassis->length_b)) * 60.0f / (3.14f * Chassis->wheel_radius);
        Chassis->out_speed[1] = ( Chassis->absolute_chassis_speed.absolute_chassis_Vy + Chassis->absolute_chassis_speed.absolute_chassis_Vx - Chassis->absolute_chassis_speed.absolute_chassis_Vw * (Chassis->length_a + Chassis->length_b)) * 60.0f / (3.14f * Chassis->wheel_radius) ;
        Chassis->out_speed[2] = ( Chassis->absolute_chassis_speed.absolute_chassis_Vy - Chassis->absolute_chassis_speed.absolute_chassis_Vx - Chassis->absolute_chassis_speed.absolute_chassis_Vw * (Chassis->length_a + Chassis->length_b)) * 60.0f / (3.14f * Chassis->wheel_radius);
        Chassis->out_speed[3] = (-Chassis->absolute_chassis_speed.absolute_chassis_Vy - Chassis->absolute_chassis_speed.absolute_chassis_Vx - Chassis->absolute_chassis_speed.absolute_chassis_Vw * (Chassis->length_a + Chassis->length_b)) * 60.0f / (3.14f * Chassis->wheel_radius);
        break;
    
//    case Steering_Wheel:  // 舵轮逆解（保留实现）
//        // 计算各轮速度 (欧几里得范数)
//        Chassis->out_speed[0] = pow(Chassis->absolute_chassis_speed.absolute_chassis_Vx * Chassis->absolute_chassis_speed.absolute_chassis_Vx + Chassis->absolute_chassis_speed.absolute_chassis_Vy * Chassis->absolute_chassis_speed.absolute_chassis_Vy + 
//                                   Chassis->absolute_chassis_speed.absolute_chassis_Vw * Chassis->wheel_radius * 
//                                   (Chassis->absolute_chassis_speed.absolute_chassis_Vw * Chassis->wheel_radius + 
//                                    Chassis->absolute_chassis_speed.absolute_chassis_Vy * Chassis->length_b / Chassis->wheel_radius - 
//                                    Chassis->absolute_chassis_speed.absolute_chassis_Vx * Chassis->length_a / Chassis->wheel_radius), 0.5);
//        Chassis->out_speed[1] = pow(Chassis->absolute_chassis_speed.absolute_chassis_Vx * Chassis->absolute_chassis_speed.absolute_chassis_Vx + Chassis->absolute_chassis_speed.absolute_chassis_Vy * Chassis->absolute_chassis_speed.absolute_chassis_Vy + 
//                                   Chassis->absolute_chassis_speed.absolute_chassis_Vw * Chassis->wheel_radius * 
//                                   (Chassis->absolute_chassis_speed.absolute_chassis_Vw * Chassis->wheel_radius - 
//                                    Chassis->absolute_chassis_speed.absolute_chassis_Vy * Chassis->length_a / Chassis->wheel_radius - 
//                                    Chassis->absolute_chassis_speed.absolute_chassis_Vx * Chassis->length_b / Chassis->wheel_radius), 0.5);
//        Chassis->out_speed[2] = pow(Chassis->absolute_chassis_speed.absolute_chassis_Vx * Chassis->absolute_chassis_speed.absolute_chassis_Vx + Chassis->absolute_chassis_speed.absolute_chassis_Vy * Chassis->absolute_chassis_speed.absolute_chassis_Vy + 
//                                   Chassis->absolute_chassis_speed.absolute_chassis_Vw * Chassis->wheel_radius * 
//                                   (Chassis->absolute_chassis_speed.absolute_chassis_Vw * Chassis->wheel_radius - 
//                                    Chassis->absolute_chassis_speed.absolute_chassis_Vy * Chassis->length_b / Chassis->wheel_radius + 
//                                    Chassis->absolute_chassis_speed.absolute_chassis_Vx * Chassis->length_a / Chassis->wheel_radius), 0.5);
//        Chassis->out_speed[3] = pow(Chassis->absolute_chassis_speed.absolute_chassis_Vx * Chassis->absolute_chassis_speed.absolute_chassis_Vx + Chassis->absolute_chassis_speed.absolute_chassis_Vy * Chassis->absolute_chassis_speed.absolute_chassis_Vy + 
//                                   Chassis->absolute_chassis_speed.absolute_chassis_Vw * Chassis->wheel_radius * 
//                                   (Chassis->absolute_chassis_speed.absolute_chassis_Vw * Chassis->wheel_radius + 
//                                    Chassis->absolute_chassis_speed.absolute_chassis_Vy * Chassis->length_a / Chassis->wheel_radius + 
//                                    Chassis->absolute_chassis_speed.absolute_chassis_Vx * Chassis->length_b / Chassis->wheel_radius), 0.5);
//        
//        // 计算各轮转向角度 (atan2函数)
//        Chassis->out_angle[0] = atan2(Chassis->absolute_chassis_speed.absolute_chassis_Vx - Chassis->absolute_chassis_speed.absolute_chassis_Vw * Chassis->length_a / 2.0f, 
//                                     Chassis->absolute_chassis_speed.absolute_chassis_Vy + Chassis->absolute_chassis_speed.absolute_chassis_Vw * Chassis->length_b / 2.0f);
//        Chassis->out_angle[1] = atan2(Chassis->absolute_chassis_speed.absolute_chassis_Vx - Chassis->absolute_chassis_speed.absolute_chassis_Vw * Chassis->length_b / 2.0f, 
//                                     Chassis->absolute_chassis_speed.absolute_chassis_Vy - Chassis->absolute_chassis_speed.absolute_chassis_Vw * Chassis->length_a / 2.0f);
//        Chassis->out_angle[2] = atan2(Chassis->absolute_chassis_speed.absolute_chassis_Vx + Chassis->absolute_chassis_speed.absolute_chassis_Vw * Chassis->length_a / 2.0f, 
//                                     Chassis->absolute_chassis_speed.absolute_chassis_Vy - Chassis->absolute_chassis_speed.absolute_chassis_Vw * Chassis->length_b / 2.0f);
//        Chassis->out_angle[3] = atan2(Chassis->absolute_chassis_speed.absolute_chassis_Vx + Chassis->absolute_chassis_speed.absolute_chassis_Vw * Chassis->length_b / 2.0f, 
//                                     Chassis->absolute_chassis_speed.absolute_chassis_Vy + Chassis->absolute_chassis_speed.absolute_chassis_Vw * Chassis->length_a / 2.0f);
//        break;
//    
    default:
        break;
    }
}

/**
 * @brief 计算云台角度误差（相对于零点）
 * @param Chassis 底盘实例指针
 * @return 角度误差值（弧度）
 * @note 将角度误差限制在[-π, π]范围内
 * @date 2025-07-09
 */
static float Find_Angle(ChassisInstance_s *Chassis)
{
	float err = Chassis->gimbal_yaw_angle-Chassis->gimbal_yaw_zero;
    if(err > 3.141593f)
        err -= 2 *3.141593f;
    else if(err < -3.141593f)
        err += 2 * 3.141593f;
    return err;
}

/**
 * @brief 计算底盘绝对坐标系速度
 * @param Chassis 底盘实例指针
 * @note 将速度从云台坐标系转换到世界坐标系
 * @date 2025-07-09
 */
static void Absolute_Calc(ChassisInstance_s *Chassis)
{
    // 坐标系旋转变换
    float angle = Find_Angle(Chassis);
    Chassis->absolute_chassis_speed.absolute_chassis_Vx = Chassis->Chassis_speed.Vx * cos(angle) + Chassis->Chassis_speed.Vy * sin(angle);
    Chassis->absolute_chassis_speed.absolute_chassis_Vy = -Chassis->Chassis_speed.Vx * sin(angle) + Chassis->Chassis_speed.Vy * cos(angle);
    Chassis->absolute_chassis_speed.absolute_chassis_Vw = Chassis->Chassis_speed.Vw;
}

/**
 * @brief 底盘运动控制主函数
 * @param chassis 底盘实例指针
 * @return 控制成功返回true，失败返回false
 * @note 执行逆解计算并控制电机
 * @date 2025-07-09
 */
bool Chassis_Control(ChassisInstance_s *chassis)
{
    // 1. 坐标系转换
    Absolute_Calc(chassis);
    
    // 2. 执行运动学逆解
    Chassis_IK_Calc(chassis);

    // 3. 控制四个驱动电机
    for(uint8_t i = 0; i < 4; i++)
    {
        Motor_Dji_Control(chassis->chassis_motor[i], chassis->out_speed[i]);
    }
    
    // 4. 发送CAN命令 (通过第一个电机实例)
    Motor_Dji_Transmit(chassis->chassis_motor[0]);
    
    return true;
}

bool Chassis_Calc(ChassisInstance_s *chassis) {
    // 1. 坐标系转换
    Absolute_Calc(chassis);

    // 2. 执行运动学逆解
    Chassis_IK_Calc(chassis);
    return true;
}

/**
 * @brief 底盘工作模式选择函数
 * @param ChassisAction 底盘实例指针
 * @note 根据当前模式修改控制参数
 * @date 2025-07-09
 */
void Chassis_Mode_Choose(ChassisInstance_s* ChassisAction)
{
    // 计算云台跟随PID输出
	float follow_Vx = Pid_Calculate(ChassisAction->gimbal_follow_pid, 0, Find_Angle(ChassisAction));
    
    switch(ChassisAction->actChassis) 
    {
    case CHASSIS_FOLLOW_GIMBAL:  // 跟随云台模式
        ChassisAction->Chassis_speed.Vw = follow_Vx;
        break;
    
    case CHASSIS_NORMAL:  // 独立运动模式
        ChassisAction->Chassis_speed.Vw = 0;
        break;
    
    case CHASSIS_GYROSCOPE:  // 小陀螺模式
        ChassisAction->Chassis_speed.Vw = 9.42f; 
        break;
    
    case CHASSIS_SLOW:  // 静止/低速模式
        ChassisAction->Chassis_speed.Vx = 0;
        ChassisAction->Chassis_speed.Vy = 0;
        ChassisAction->Chassis_speed.Vw = 0;
        break;
    
    default:  // 未知模式
        break;
    }
}

/* 
 * @brief 底盘功率限制函数 (未实现)
 * @note 预留功能，用于防止底盘超功率
 * @date 2025-07-03
 */
/*
static void Chassis_Power_Limit(void)
{
    // 待实现功能
}
*/