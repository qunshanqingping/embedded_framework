// ReSharper disable CppParameterMayBeConst
#include "basic_math.h"

/**
 * @brief float死区
 * @param Value: 输入值
 * @param minValue: 死区最小值
 * @param maxValue: 死区最大值
 * @return 死区处理后的值
 */
float float_deadline(float Value, float minValue, float maxValue){
    if (Value < maxValue && Value > minValue){
        Value = 0.0f;
    }
    return Value;
}

/**
 * @brief int16_t死区
 * @param Value: 输入值
 * @param minValue: 死区最小值
 * @param maxValue: 死区最大值
 * @return 死区处理后的值
 */
int16_t int16_deadline(int16_t Value, int16_t minValue, int16_t maxValue){
    if (Value < maxValue && Value > minValue){
        Value = 0;
    }
    return Value;
}

/**
 * @brief 线性映射函数
 * @param x: 输入值
 * @param in_min: 输入值最小值
 * @param in_max: 输入值最大值
 * @param out_min: 输出值最小值
 * @param out_max: 输出值最大值
 * @return 映射后的值
 */
long map(long x, long in_min, long in_max, long out_min, long out_max){
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float floatEqual_0(const float num){
    //用于处理判断float变量是否为0
    if (num < (float)FLOAT_ZERO){
        return num;
    }
    return 0;
}

/**
 * @brief float转byte数组
 * @param target: 输入float指针
 * @param buf: 输出byte数组
 * @param beg: buf的起始位置
 */
void float2byte(float* target, unsigned char* buf, unsigned char beg){
    // ReSharper disable once CppLocalVariableMayBeConst
    unsigned char* point = (unsigned char*)target; //得到float的地址
    buf[beg] = point[0];
    buf[beg + 1] = point[1];
    buf[beg + 2] = point[2];
    buf[beg + 3] = point[3];
}

/**
 * @brief 斜坡函数初始化
 * @param instance: 斜坡结构体
 * @param frame_period: 两帧之间的间隔时间（单位：毫秒）
 */
void Ramp_init(RampInstance_s* instance,uint8_t frame_period){
    instance->frame_period = frame_period;
}

/**
 * @brief 斜坡函数更新
 * @param instance: 斜坡结构体
 * @param update_value: 更新的目标值
 */
void Ramp_Update(RampInstance_s* instance, float update_value){
    instance->start_value = instance->end_value;
    instance->end_value = update_value;
    instance->step_value = (instance->end_value - instance->start_value) / (float)instance->frame_period;
    instance->loop_count = 0;
}
/**
 * @brief 斜坡函数读取
 * @param instance: 斜坡结构体
 * @return 当前输出值
 */
float Ramp_Read(RampInstance_s* instance){
    if (instance->loop_count < instance->frame_period){
        instance->output_value += instance->step_value;
        instance->loop_count++;
    }
    else{
        instance->output_value = instance->end_value;
    }
    return instance->output_value;
}
