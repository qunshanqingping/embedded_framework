#include "motor_dji.h"
#include "bsp_can.h"
#include "bsp_fdcan.h"

// ReSharper disable once CppParameterMayBeConstPtrOrRef
static void MOTOR_Dji_Decode( CanInstance_s *can_instance) {
    if (can_instance == NULL) {
        return;
    }
    DjiMotorInstance_s *motor_instance = can_instance->parent_ptr;
    motor_instance->message.rotor_angle          // 转子机械角度
            = (uint16_t) (can_instance->rx_buff[0] << 8 | can_instance->rx_buff[1]);
    motor_instance->message.rotor_speed    // 转子转速
            = (uint16_t) (can_instance->rx_buff[2] << 8 | can_instance->rx_buff[3]);
    motor_instance->message.torque_current // 实际转矩电流
            = (uint16_t) (can_instance->rx_buff[4] << 8 | can_instance->rx_buff[5]);
    motor_instance->message.rotor_temp   // 电机温度
            = can_instance->rx_buff[6];

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
    config->can_config.can_module_callback = MOTOR_Dji_Decode;
}

