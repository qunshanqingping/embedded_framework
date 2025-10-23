#include "motor_dji.h"
#include "basic_math.h"
#include "bsp_can.h"
#include "bsp_fdcan.h"
/**
 * @file motor_dji.c
 * @brief 大疆电机温度保护
 * @param motor_instance 电机实例指针
 * @date 2025-08-18
 */
static void Motor_Temp_Protect(DjiMotorInstance_s* motor_instance){
    if (motor_instance->message.raw.rotor_temp >= DJI_MOTOR_TEMPERATURE_LIMIT){
        motor_instance->message.status.temp_overheat_cnt++;
    }
}
/**
 * @brief 大疆M3508电机数据解码
 * @param can_instance CAN实例指针
 */
// ReSharper disable once CppParameterMayBeConstPtrOrRef
static void M3508_Decode(CanInstance_s* can_instance){
    // 检查CAN实例指针是否为空
    if (can_instance == NULL){
        return;
    }
    // 获取电机实例指针
    DjiMotorInstance_s* motor_instance = can_instance->parent_ptr;
    // 电机机械角度(0~8191)
    // motor_instance->message.raw.rotor_angle
    //     = (uint16_t)(can_instance->rx_buff[0] << 8 | can_instance->rx_buff[1]);
    // 电机转速 (rpm)
    motor_instance->message.raw.rotor_speed
        = (uint16_t)(can_instance->rx_buff[2] << 8 | can_instance->rx_buff[3]);
    // 转矩电流 ( -16384 ~ 16384 )映射到实际电流值(-20A ~ 20A)
    motor_instance->message.raw.torque_current
        = (uint16_t)(can_instance->rx_buff[4] << 8 | can_instance->rx_buff[5]);
    // 电机温度 (℃)
    motor_instance->message.raw.rotor_temp
        = can_instance->rx_buff[6];

    // 增加接收计数
    motor_instance->message.status.rx_cnt++;
    // 电机温度保护
    Motor_Temp_Protect(motor_instance);
    // 计算输出轴速度(rad/s)
    motor_instance->message.calc.velocity = motor_instance->message.raw.rotor_speed / motor_instance->reduction_ratio *
        RPM_TO_RADS;
    // 计算输出轴转矩(mN·m)
    motor_instance->message.calc.torque = map(motor_instance->message.raw.torque_current,DJI_RAW_TORQUE_CURRENT_MIN,
                                              DJI_RAW_TORQUE_CURRENT_MAX,
                                              DJI_M3508_ACTUAL_TORQUE_CURRENT_MIN,
                                              DJI_M3508_ACTUAL_TORQUE_CURRENT_MAX) * motor_instance->torque_constant;
}

/**
 * @brief 大疆GM6020电机数据解码
 * @param can_instance CAN实例指针
 */
// ReSharper disable once CppParameterMayBeConstPtrOrRef
static void GM6020_Decode(CanInstance_s* can_instance){
    // 检查CAN实例指针是否为空
    if (can_instance == NULL){
        return;
    }
    // 获取电机实例指针
    DjiMotorInstance_s* motor_instance = can_instance->parent_ptr;
    // 电机机械角度(0~8191)
    motor_instance->message.raw.rotor_angle
        = (uint16_t)(can_instance->rx_buff[0] << 8 | can_instance->rx_buff[1]);
    // 电机转速 (rpm)
    motor_instance->message.raw.rotor_speed
        = (uint16_t)(can_instance->rx_buff[2] << 8 | can_instance->rx_buff[3]);
    // 转矩电流 ( -16384 ~ 16384 )映射到实际电流值(-3A ~ 3A)
    motor_instance->message.raw.torque_current
        = (uint16_t)(can_instance->rx_buff[4] << 8 | can_instance->rx_buff[5]);
    // 电机温度 (℃)
    motor_instance->message.raw.rotor_temp
        = can_instance->rx_buff[6];
    // 增加接收计数
    motor_instance->message.status.rx_cnt++;
    // 电机温度保护
    Motor_Temp_Protect(motor_instance);
    // 计算输出轴速度(rad/s)
    motor_instance->message.calc.velocity = motor_instance->message.raw.rotor_speed / motor_instance->reduction_ratio *
        RPM_TO_RADS;
    // 计算输出轴转矩(mN·m)
    motor_instance->message.calc.torque = map(motor_instance->message.raw.torque_current,DJI_RAW_TORQUE_CURRENT_MIN,
                                              DJI_RAW_TORQUE_CURRENT_MAX,
                                              DJI_GM6020_ACTUAL_TORQUE_CURRENT_MIN,
                                              DJI_GM6020_ACTUAL_TORQUE_CURRENT_MAX) * motor_instance->torque_constant;
}

/**
 * @brief 选择大疆电机解码函数
 * @param config 大疆电机初始化配置结构体指针
 */
static void Motor_Dji_SLect_Decode(DjiMotorInitConfig_s* config){
    if (config == NULL){
        Log_Error("Motor_Dji_SLect_Decode : Config is NULL");
        return;
    }
    switch (config->type){
    case M3508:{
        config->can_config.can_module_callback = M3508_Decode;
        break;
    }
        case GM6020:{
        config->can_config.can_module_callback = GM6020_Decode;
        break;
    }
        default:
        break;
    }
}

/**
 * @brief 配置CAN模块
 * @param config
 */
static void MOTOR_Dji_Can_Set(DjiMotorInitConfig_s* config){
    config->can_config.topic_name = config->topic_name;
    config->can_config.can_number = config->can_number;
    if (config->type == GM6020){
        config->can_config.rx_id = config->id + GM_RX_ID;
        if (config->id < 5){
            config->can_config.tx_id = GM_CTRL_1_TO_4_ID;
        }
        else{
            config->can_config.tx_id = GM_CTRL_5_TO_7_ID;
        }
    }
    else{
        config->can_config.rx_id = config->id + M_RX_ID;
        if (config->id < 5){
            config->can_config.tx_id = M_CTRL_1_TO_4_ID;
        }
        else{
            config->can_config.tx_id = M_CTRL_5_TO_8_ID;
        }
    }
    Motor_Dji_SLect_Decode(config);
}
