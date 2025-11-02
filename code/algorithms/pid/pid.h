//
// Created by 29568 on 2025/11/2.
// 
// 离散时间PID控制器库
// 特性:
// - 支持多种积分和微分数值方法 (前向欧拉、后向欧拉、梯形公式)
// - 函数指针优化，避免运行时switch判断
// - 默认启用抗积分饱和机制
// - 动态内存管理，支持多实例
// - 微分滤波器支持
//

#ifndef PID_H
#define PID_H

#include <stdint.h>
#include <stdbool.h>

// 离散时间PID控制器导数公式类型
typedef enum {
    DF_FORWARD_EULER,     // 前向欧拉: DF(z) = Ts/(z-1)
    DF_BACKWARD_EULER,    // 后向欧拉: DF(z) = Ts*z/(z-1)
    DF_TRAPEZOIDAL        // 梯形公式: DF(z) = Ts/2*(z+1)/(z-1)
} DERIVATIVE_FORMULA_e;

// 离散时间PID控制器积分公式类型
typedef enum {
    IF_FORWARD_EULER,     // 前向欧拉: IF(z) = Ts/(z-1)
    IF_BACKWARD_EULER,    // 后向欧拉: IF(z) = Ts*z/(z-1)
    IF_TRAPEZOIDAL        // 梯形公式: IF(z) = Ts/2*(z+1)/(z-1)
} INTEGRAL_FORMULA_e;

// 前向声明
struct PidInstance_s;

// 函数指针类型定义
typedef float (*integral_func_t)(struct PidInstance_s* instance, float error);
typedef float (*derivative_func_t)(struct PidInstance_s* instance, float error);

// PID控制器实例结构体
typedef struct PidInstance_s
{
    char* topic_name;           // 实例名称
    
    // PID增益参数
    float Kp;                   // 比例增益
    float Ki;                   // 积分增益
    float Kd;                   // 微分增益

    // 滤波器参数
    float Tf;                   // 微分滤波器时间常数
    float Ts;                   // 采样时间

    // 公式选择
    DERIVATIVE_FORMULA_e d_formula;  // 微分公式选择
    INTEGRAL_FORMULA_e i_formula;    // 积分公式选择
    
    // 函数指针 - 核心计算函数
    integral_func_t integral_func;   // 积分计算函数指针
    derivative_func_t derivative_func; // 微分计算函数指针

    // 内部状态变量
    float integral_state;       // 积分累加器
    float derivative_state;     // 微分滤波器状态
    float prev_error;          // 前一次误差值
    float prev_derivative;     // 前一次微分值

    // 输出限制
    float output_min;          // 输出最小值
    float output_max;          // 输出最大值
    bool enable_limits;        // 是否启用输出限制
    float output;              // 当前输出值
} PidInstance_s;

// PID初始化配置结构体
typedef struct
{
    char* topic_name;           // 实例名称
    float Kp;                   // 比例增益
    float Ki;                   // 积分增益
    float Kd;                   // 微分增益

    // 滤波器参数
    float Tf;                   // 微分滤波器时间常数
    float Ts;                   // 采样时间

    // 公式选择
    DERIVATIVE_FORMULA_e d_formula;  // 微分公式选择
    INTEGRAL_FORMULA_e i_formula;    // 积分公式选择
    
    // 输出限制 (必须设置，用于抗积分饱和)
    float output_min;           // 输出最小值 (必须 < 0)
    float output_max;           // 输出最大值 (必须 > 0)
} PidInitConfig_s;

// 函数声明
PidInstance_s* Pid_Register(PidInitConfig_s* config);    // 注册PID实例，默认启用抗积分饱和
void Pid_Unregister(PidInstance_s* instance);           // 注销PID实例
void Pid_SetLimits(PidInstance_s* instance, float min_output, float max_output);  // 动态设置输出限制
void Pid_Reset(PidInstance_s* instance);                // 重置PID状态
float Pid_Update(PidInstance_s* instance, float setpoint, float measurement);    // PID计算更新
void Pid_SetGains(PidInstance_s* instance, float Kp, float Ki, float Kd);       // 动态调整增益

// 积分计算函数
float integral_forward_euler(PidInstance_s* instance, float error);
float integral_backward_euler(PidInstance_s* instance, float error);
float integral_trapezoidal(PidInstance_s* instance, float error);

// 微分计算函数
float derivative_forward_euler(PidInstance_s* instance, float error);
float derivative_backward_euler(PidInstance_s* instance, float error);
float derivative_trapezoidal(PidInstance_s* instance, float error);

#endif //PID_H