//
// Created by 29568 on 2025/11/2.
//

#include "pid.h"
#include <math.h>
#include "plf_log.h"
#include "memory_management.h"
#include "string.h"

// ==================== 积分计算函数 ====================

/**
 * @brief 前向欧拉积分计算
 * @param instance PID实例指针
 * @param error 当前误差
 * @return 积分增量
 */
float integral_forward_euler(PidInstance_s* instance, float error)
{
    return instance->Ki * instance->Ts * error;
}

/**
 * @brief 后向欧拉积分计算
 * @param instance PID实例指针
 * @param error 当前误差
 * @return 积分增量
 */
float integral_backward_euler(PidInstance_s* instance, float error)
{
    return instance->Ki * instance->Ts * error;
}

/**
 * @brief 梯形积分计算
 * @param instance PID实例指针
 * @param error 当前误差
 * @return 积分增量
 */
float integral_trapezoidal(PidInstance_s* instance, float error)
{
    float increment = instance->Ki * instance->Ts * 0.5f * (error + instance->prev_error);
    return increment;
}

// ==================== 微分计算函数 ====================

/**
 * @brief 前向欧拉微分计算
 * @param instance PID实例指针
 * @param error 当前误差
 * @return 微分值
 */
float derivative_forward_euler(PidInstance_s* instance, float error)
{
    if (instance->Tf <= 0.0f) {
        // 无滤波器的纯微分
        float derivative = (error - instance->prev_error) / instance->Ts;
        return derivative;
    } else {
        // 带滤波器的微分
        float error_derivative = (error - instance->prev_error) / instance->Ts;
        float alpha = instance->Ts / (instance->Tf + instance->Ts);
        instance->derivative_state = (1.0f - alpha) * instance->derivative_state + alpha * error_derivative;
        return instance->derivative_state;
    }
}

/**
 * @brief 后向欧拉微分计算
 * @param instance PID实例指针
 * @param error 当前误差
 * @return 微分值
 */
float derivative_backward_euler(PidInstance_s* instance, float error)
{
    if (instance->Tf <= 0.0f) {
        // 无滤波器的纯微分
        float derivative = (error - instance->prev_error) / instance->Ts;
        return derivative;
    } else {
        // 带滤波器的微分 (后向欧拉)
        float error_derivative = (error - instance->prev_error) / instance->Ts;
        float alpha = instance->Ts / (instance->Tf + instance->Ts);
        instance->derivative_state = (1.0f - alpha) * instance->derivative_state + alpha * error_derivative;
        return instance->derivative_state;
    }
}

/**
 * @brief 梯形微分计算
 * @param instance PID实例指针
 * @param error 当前误差
 * @return 微分值
 */
float derivative_trapezoidal(PidInstance_s* instance, float error)
{
    if (instance->Tf <= 0.0f) {
        // 无滤波器时回退到后向欧拉
        return derivative_backward_euler(instance, error);
    } else {
        // 梯形公式微分滤波器
        float error_derivative = (error - instance->prev_error) / instance->Ts;
        float alpha = 2.0f * instance->Tf / (2.0f * instance->Tf + instance->Ts);
        float beta = instance->Ts / (2.0f * instance->Tf + instance->Ts);
        
        float new_derivative = alpha * instance->prev_derivative + beta * (error_derivative + instance->prev_derivative);
        instance->derivative_state = new_derivative;
        return new_derivative;
    }
}

// ==================== 主要PID函数 ====================

/**
 * @brief 注册PID控制器实例
 * @param config 初始化配置
 * @return PID实例指针，失败返回NULL
 */
PidInstance_s* Pid_Register(PidInitConfig_s* config)
{
    if (config == NULL || config->topic_name == NULL) {
        Log_Error("Pid_Register: config_ptr is NULL");
        return NULL;
    }
    
    // 验证输出限制参数
    if (config->output_min >= config->output_max) {
        Log_Error("Pid_Register: %s Invalid output limits (min=%.2f >= max=%.2f)", 
                  config->topic_name, config->output_min, config->output_max);
        return NULL;
    }
    
    if (config->Ts <= 0.0f) {
        Log_Error("Pid_Register: %s Invalid sampling time Ts=%.6f", 
                  config->topic_name, config->Ts);
        return NULL;
    }
    
    PidInstance_s* instance = (PidInstance_s*)user_malloc(sizeof(PidInstance_s));
    if (instance == NULL) {
        Log_Error("Pid_Register: %s Memory_Alloc failed", config->topic_name);
        return NULL;
    }
    
    // 清零内存
    memset(instance, 0, sizeof(PidInstance_s));
    
    // 复制配置参数
    instance->topic_name = config->topic_name;
    instance->Kp = config->Kp;
    instance->Ki = config->Ki;
    instance->Kd = config->Kd;
    instance->Tf = config->Tf;
    instance->Ts = config->Ts;
    instance->d_formula = config->d_formula;
    instance->i_formula = config->i_formula;
    
    // 默认启用抗积分饱和，设置输出限制
    instance->output_min = config->output_min;
    instance->output_max = config->output_max;
    instance->enable_limits = true;  // 默认启用输出限制和抗积分饱和
    
    // 设置函数指针
    switch (config->i_formula) {
        case IF_FORWARD_EULER:
            instance->integral_func = integral_forward_euler;
            break;
        case IF_BACKWARD_EULER:
            instance->integral_func = integral_backward_euler;
            break;
        case IF_TRAPEZOIDAL:
            instance->integral_func = integral_trapezoidal;
            break;
        default:
            instance->integral_func = integral_backward_euler;
            break;
    }
    
    switch (config->d_formula) {
        case DF_FORWARD_EULER:
            instance->derivative_func = derivative_forward_euler;
            break;
        case DF_BACKWARD_EULER:
            instance->derivative_func = derivative_backward_euler;
            break;
        case DF_TRAPEZOIDAL:
            instance->derivative_func = derivative_trapezoidal;
            break;
        default:
            instance->derivative_func = derivative_backward_euler;
            break;
    }
    
    // 初始化状态
    Pid_Reset(instance);
    
    Log_Information("Pid_Register: %s registered successfully (Limits: [%.2f, %.2f], Anti-windup: ON)", 
                    config->topic_name, config->output_min, config->output_max);
    return instance;
}

/**
 * @brief 注销PID控制器实例
 * @param instance PID实例指针
 */
void Pid_Unregister(PidInstance_s* instance)
{
    if (instance != NULL) {
        Log_Information("Pid_Unregister: %s unregistered", instance->topic_name);
        user_free(instance);
    }
}

/**
 * @brief 设置PID输出限制
 * @param instance PID实例指针
 * @param min_output 最小输出值
 * @param max_output 最大输出值
 */
void Pid_SetLimits(PidInstance_s* instance, float min_output, float max_output)
{
    if (instance == NULL) return;
    
    instance->output_min = min_output;
    instance->output_max = max_output;
    instance->enable_limits = true;
}

/**
 * @brief 重置PID控制器状态
 * @param instance PID实例指针
 */
void Pid_Reset(PidInstance_s* instance)
{
    if (instance == NULL) return;
    
    instance->integral_state = 0.0f;
    instance->derivative_state = 0.0f;
    instance->prev_error = 0.0f;
    instance->prev_derivative = 0.0f;
    instance->output = 0.0f;
}

/**
 * @brief 设置PID增益参数
 * @param instance PID实例指针
 * @param Kp 比例增益
 * @param Ki 积分增益
 * @param Kd 微分增益
 */
void Pid_SetGains(PidInstance_s* instance, float Kp, float Ki, float Kd)
{
    if (instance == NULL) return;
    
    instance->Kp = Kp;
    instance->Ki = Ki;
    instance->Kd = Kd;
}

/**
 * @brief PID控制器更新计算
 * @param instance PID实例指针
 * @param setpoint 设定值
 * @param measurement 测量值
 * @return 控制器输出
 */
float Pid_Update(PidInstance_s* instance, float setpoint, float measurement)
{
    if (instance == NULL) return 0.0f;
    
    // 计算误差
    float error = setpoint - measurement;
    
    // 比例项
    float proportional = instance->Kp * error;
    
    // 积分项
    float integral_increment = 0.0f;
    if (instance->Ki != 0.0f && instance->integral_func != NULL) {
        integral_increment = instance->integral_func(instance, error);
        instance->integral_state += integral_increment;
    }
    
    // 微分项
    float derivative = 0.0f;
    if (instance->Kd != 0.0f && instance->derivative_func != NULL) {
        derivative = instance->Kd * instance->derivative_func(instance, error);
    }
    
    // 计算总输出
    float output = proportional + instance->integral_state + derivative;
    
    // 应用输出限制和抗积分饱和
    if (instance->enable_limits) {
        if (output > instance->output_max) {
            output = instance->output_max;
            // 抗积分饱和：如果输出饱和且积分项会使饱和更严重，则不累积积分
            if (integral_increment > 0.0f) {
                instance->integral_state -= integral_increment;
            }
        } else if (output < instance->output_min) {
            output = instance->output_min;
            // 抗积分饱和：如果输出饱和且积分项会使饱和更严重，则不累积积分
            if (integral_increment < 0.0f) {
                instance->integral_state -= integral_increment;
            }
        }
    }
    
    // 更新历史值
    instance->prev_error = error;
    if (instance->Kd != 0.0f) {
        instance->prev_derivative = derivative / instance->Kd; // 存储未缩放的微分值
    }
    instance->output = output;
    
    return output;
}
