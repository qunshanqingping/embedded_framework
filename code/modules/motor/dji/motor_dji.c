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
static void Motor_Temp_Protect(DjiMotorInstance_s *motor_instance) {
if (motor_instance->message.raw.rotor_temp >= DJI_MOTOR_TEMPERATURE_LIMIT){
    motor_instance->message.status.temp_overheat_cnt++;
}
}
// ReSharper disable once CppParameterMayBeConstPtrOrRef
static void MOTOR_Dji_M3508_Decode( CanInstance_s *can_instance) {
    if (can_instance == NULL) {
        return;
    }
    DjiMotorInstance_s *motor_instance = can_instance->parent_ptr;
    motor_instance->message.raw.rotor_angle          // 转子机械角度
            = (uint16_t) (can_instance->rx_buff[0] << 8 | can_instance->rx_buff[1]);
    motor_instance->message.raw.rotor_speed    // 转子转速
            = (uint16_t) (can_instance->rx_buff[2] << 8 | can_instance->rx_buff[3]);
    motor_instance->message.raw.torque_current // 实际转矩电流
            = (uint16_t) (can_instance->rx_buff[4] << 8 | can_instance->rx_buff[5]);
    motor_instance->message.raw.rotor_temp   // 电机温度
            = can_instance->rx_buff[6];
    motor_instance->message.status.rx_cnt++;
    Motor_Temp_Protect(motor_instance);
    motor_instance->message.calc.velocity = motor_instance->message.raw.rotor_speed /motor_instance->reduction_ratio * RPM_TO_RADS; // 计算输出轴角度
    motor_instance->message.calc.torque = map(motor_instance->message.raw.torque_current,DJI_RAW_TORQUE_CURRENT_MIN, DJI_RAW_TORQUE_CURRENT_MAX,
                                                DJI_M3508_ACTUAL_TORQUE_CURRENT_MIN, DJI_M3508_ACTUAL_TORQUE_CURRENT_MAX) * motor_instance->torque_constant; // 计算输出轴转矩
}
static void DJI_GM6020_Decode( CanInstance_s *can_instance) {
    if (can_instance == NULL) {
        return;
    }
    DjiMotorInstance_s *motor_instance = can_instance->parent_ptr;
    motor_instance->message.raw.rotor_angle          // 转子机械角度
            = (uint16_t) (can_instance->rx_buff[0] << 8 | can_instance->rx_buff[1]);
    motor_instance->message.raw.rotor_speed    // 转子转速
            = (uint16_t) (can_instance->rx_buff[2] << 8 | can_instance->rx_buff[3]);
    motor_instance->message.raw.torque_current // 实际转矩电流
            = (uint16_t) (can_instance->rx_buff[4] << 8 | can_instance->rx_buff[5]);
    motor_instance->message.raw.rotor_temp   // 电机温度
            = can_instance->rx_buff[6];
    motor_instance->message.status.rx_cnt++;
    Motor_Temp_Protect(motor_instance);
    motor_instance->message.calc.velocity = motor_instance->message.raw.rotor_speed /motor_instance->reduction_ratio * RPM_TO_RADS; // 计算输出轴角度
    motor_instance->message.calc.torque = map(motor_instance->message.raw.torque_current,DJI_RAW_TORQUE_CURRENT_MIN, DJI_RAW_TORQUE_CURRENT_MAX,
                                                DJI_GM6020_ACTUAL_TORQUE_CURRENT_MIN, DJI_GM6020_ACTUAL_TORQUE_CURRENT_MAX) * motor_instance->torque_constant; // 计算输出轴转矩
}
/**
 * @brief 选择大疆电机解码函数
 * @param config 大疆电机初始化配置结构体指针
 */
static void Motor_Dji_SLect_Decode(DjiMotorInitConfig_s *config){
    if (config == NULL) {
        Log_Error("Motor_Dji_SLect_Decode : Config is NULL");
        return;
    }
    if (config->type == M3508){
        config->can_config.can_module_callback = MOTOR_Dji_M3508_Decode;
    }
}
/**
 * @brief 配置CAN模块
 * @param config
 */
static void MOTOR_Dji_Can_Set(DjiMotorInitConfig_s *config) {
    config->can_config.topic_name = config->topic_name;
    config->can_config.can_number = config->can_number;
    if (config->type == GM6020) {
        config->can_config.rx_id = config->id + GM_RX_ID;
        if (config->id <5) {
            config->can_config.tx_id = GM_CTRL_1_TO_4_ID;
        }
        else {
            config->can_config.tx_id = GM_CTRL_5_TO_7_ID;
        }
    }
    else {
        config->can_config.rx_id = config->id + M_RX_ID;
        if (config->id<5) {
            config->can_config.tx_id = M_CTRL_1_TO_4_ID;
        }
        else {
            config->can_config.tx_id = M_CTRL_5_TO_8_ID;
        }
    }
    Motor_Dji_SLect_Decode(config);
}

