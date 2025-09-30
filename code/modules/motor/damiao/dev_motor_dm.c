#include "dev_motor_dm.h"


static int float_to_uint(const float x_float, const float x_min, const float x_max, const int bits)
{
    const float span = x_max - x_min;
    const float offset = x_min;
    return (int)((x_float - offset) * ((float)((1 << bits) - 1)) / span);
}
static float uint_to_float(const int x_int, const float x_min, const float x_max, const int bits)
{
    const float span = x_max - x_min;
    const float offset = x_min;
    return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}
static float  Angle_Normalize (float angle)
{
    while (angle > 3.141593f)
        angle -=  2*3.141593f;
    while (angle < -3.141593f)
        angle +=  2*3.141593f;
    return angle;
}
// ReSharper disable once CppParameterMayBeConstPtrOrRef
static void Motor_Dm_Decode( CanInstance_s *can_instance)
{
    if(can_instance == NULL){
        return;
    }

    const uint8_t *rx_buff = can_instance->rx_buff;
    DmMotorInstance_s *motor = can_instance->id;

    motor->motor_state = rx_buff[0] >> 4;
    motor->p_int = rx_buff[1] << 8 | rx_buff[2];
    motor->v_int = rx_buff[3] << 4 | rx_buff[4] >> 4;
    motor->t_int = (rx_buff[4] & 0xF) << 8 | rx_buff[5];
    motor->last_position = motor->position;
    motor->last_out_position = motor->out_position;
    motor->position = uint_to_float(motor->p_int, -motor->p_max, motor->p_max, 16); // (-P_MAX,P_MAX)
    motor->out_position = Angle_Normalize(motor->position);  // (-PI,PI)
    motor->out_velocity = uint_to_float(motor->v_int, -motor->v_max, motor->v_max, 12); // (-V_MAX,V_MAX)
    motor->torque = uint_to_float(motor->t_int, -motor->t_max, motor->t_max, 12);   // (-T_MAX,T_MAX)
    motor->T_MOS = (float)(rx_buff[6]);
    motor->T_Rotor = (float)(rx_buff[7]);
}



DmMotorInstance_s *Motor_DM_Register(DMMotorInitConfig_s *config)
{
    if (config == NULL)
    {
        return NULL;
    }
    DmMotorInstance_s *motor_instance = (DmMotorInstance_s *)user_malloc(sizeof(DmMotorInstance_s));
    if (motor_instance == NULL)
    {
        return NULL; // 内存分配失败
    }
    motor_instance->topic_name = config->topic_name;
    config->can_config.topic_name = config->topic_name;
    motor_instance->control_mode = config->control_mode;
    motor_instance->work_mode = config->work_mode;
    motor_instance->p_max = config->p_max;
    motor_instance->v_max = config->v_max;
    motor_instance->t_max = config->t_max;
    motor_instance->can_id = config->can_id;
    motor_instance->master_id = config->master_id;

    config->can_config.id = motor_instance;
    config->can_config.can_number = config->can_number;
    config->can_config.rx_id = config->master_id;
    config->can_config.tx_id = config->can_id+config->work_mode;
    config->can_config.can_module_callback = Motor_Dm_Decode;
    motor_instance->can_instance = Can_Register(&config->can_config);
    motor_instance->angle_pid = Pid_Register(&config->angle_pid_config);
    motor_instance->velocity_pid = Pid_Register(&config->velocity_pid_config);
    return motor_instance;
}

/**
 * @brief DM电机命令帧
 */
static uint8_t dm_cmd_frame[4][8] ={
    {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFC}, // 使能
    {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFD}, // 失能
    {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFE}, // 保存位置零点
    {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFB}  // 清除电机错误
} ;


bool Motor_Dm_Cmd(const DmMotorInstance_s *motor, const DmMotor_Mode_e cmd)
{
    if (motor == NULL || motor->can_instance == NULL || cmd >= 4)
    {
        return false;
    }
    memcpy(motor->can_instance->tx_buff, dm_cmd_frame[cmd], 8);  // 复制8个字节
    return true;
}
bool Motor_Dm_Mit_Control(const DmMotorInstance_s *motor, const float pos, const float vel, const float kp, const float kd, const float tor)
{
    if (motor == NULL || motor->can_instance == NULL || motor->work_mode != MIT || motor->motor_state != DM_ENABLE){
        return false;
    }
        const uint16_t pos_tmp = float_to_uint(pos, -motor->p_max, motor->p_max, 16);
        const uint16_t vel_tmp = float_to_uint(vel, -motor->v_max, motor->v_max, 12);
        const uint16_t kp_tmp = float_to_uint(kp, KP_MIN,KP_MAX, 12);
        const uint16_t kd_tmp = float_to_uint(kd, KD_MIN,KD_MAX, 12);
        const uint16_t tor_tmp = float_to_uint(tor, -motor->t_max, motor->t_max, 12);

        motor->can_instance->tx_buff[0] = (pos_tmp >> 8);
        motor->can_instance->tx_buff[1] = pos_tmp;
        motor->can_instance->tx_buff[2] = (vel_tmp >> 4);
        motor->can_instance->tx_buff[3] = ((vel_tmp & 0xF) << 4) | (kp_tmp >> 8);
        motor->can_instance->tx_buff[4] = kp_tmp;
        motor->can_instance->tx_buff[5] = (kd_tmp >> 4);
        motor->can_instance->tx_buff[6] = ((kd_tmp & 0xF) << 4) | (tor_tmp >> 8);
        motor->can_instance->tx_buff[7] = tor_tmp;
        return true;
}

bool Motor_DM_Control(DmMotorInstance_s *motor, const float target) {
    if(motor == NULL || motor->can_instance == NULL || motor->work_mode != MIT || motor->motor_state != DM_ENABLE) {
        return false;
    }
    memset(motor->can_instance->tx_buff, 0 , 8);
    if (motor->control_mode == POSITION) {
        motor->target_position = target;
        motor->target_velocity = Pid_Calculate(motor->angle_pid, motor->target_position, motor->position);
        motor->output= Pid_Calculate(motor->velocity_pid, motor->target_velocity, motor->out_velocity);
    }
    else
        if (motor->control_mode == VELOCITY) {
            motor->target_velocity = target;
            motor->output = Pid_Calculate(motor->velocity_pid, motor->target_velocity, motor->out_velocity);
        }else {
        return false;
    }
    Motor_Dm_Mit_Control(motor, 0,0,0,0,motor->output);
    return true;
}

bool Motor_Dm_Pos_Vel_Control(const DmMotorInstance_s *motor, float pos, float vel)
{
    if (motor == NULL || motor->can_instance == NULL || motor->work_mode != POS_VEL || motor->motor_state != DM_ENABLE)
    {
        return false;
    }
    const uint8_t* pos_buf = (uint8_t*)&pos;
    const uint8_t* vel_buf = (uint8_t*)&vel;

    motor->can_instance->tx_buff[0] = *pos_buf;
    motor->can_instance->tx_buff[1] = *(pos_buf+1);
    motor->can_instance->tx_buff[2] = *(pos_buf+2);
    motor->can_instance->tx_buff[3] = *(pos_buf+3);
    motor->can_instance->tx_buff[4] = *vel_buf;
    motor->can_instance->tx_buff[5] = *(vel_buf+1);
    motor->can_instance->tx_buff[6] = *(vel_buf+2);
    motor->can_instance->tx_buff[7] = *(vel_buf+3);
    return true;
}


bool Motor_Dm_Transmit(const DmMotorInstance_s *motor)
{
    if (motor == NULL || motor->can_instance == NULL)
    {
        return false;
    }
    return Can_Transmit(motor->can_instance);
}
