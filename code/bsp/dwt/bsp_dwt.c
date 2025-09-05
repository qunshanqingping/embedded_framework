/**
 * @file bsp_dwt.c
 * @brief 更精确的延时函数和系统时间功能实现
 * @author Adonis Jin
 * @date 2025-7-27
 * @version 1.0
 * @note  实现基本功能
 * @date 2025-8-26
 * @version 1.1
 * @note 规范命名；修改Dwt_Init函数，无需传入频率参数；修改延时函数，不使用float进行时间计算
 */
#include "bsp_dwt.h"
#include "cmsis_os.h"
#include "main.h"

/* 私有变量 ---------------------------------------------------------*/

/**
 * @brief 系统时间相关变量
 * @param sys_time : 用于存储系统当前时间的结构体，包含秒(s)、毫秒(ms)和微秒(us)三个成员
 */
static DwtTime_s sys_time;
/**
 * @brief 系统时钟频率相关变量
 * @param cpu_freq_hz_s : 每秒的时钟计数
 * @param cpu_freq_hz_ms : 每毫秒的时钟计数
 * @param cpu_freq_hz_us : 每微秒的时钟计数
 */
static uint32_t cpu_freq_hz_s, cpu_freq_hz_ms, cpu_freq_hz_us;
/**
 * @brief DWT计数器相关变量
 * @param cyclist_overflow_count : 记录计数器溢出次数
 * @param cyclist_last : 上次读取的计数值
 */
static uint32_t cyclist_overflow_count, cyclist_last;
/**
 * @brief 64位循环计数器，用于存储从系统启动到当前时刻的总周期数
 * @details 此变量结合了DWT->CYCCNT寄存器的当前值和溢出次数来构造一个64位无符号整数，表示总的处理器时钟周期数。
 *          它主要用于精确的时间测量和性能分析。
 */
static uint64_t cyclist64;

/* 私有函数 ---------------------------------------------------------*/

/**
 * @brief 更新DWT计数器值
 * @note 该函数用于更新CYCLIST计数器的状态，检测计数器是否溢出，
 *       并更新相应的计数器变量。使用bit_blocker防止函数重入。
 */
static void Dwt_Cnt_Update(void)
{
    // 使用静态变量作为互斥标志，防止函数重入
    static volatile uint8_t bit_blocker = 0;
    if (!bit_blocker)
    {
        bit_blocker = 1;
        // 读取当前CYCLIST寄存器的值
        const volatile uint32_t cnt_now = DWT->CYCCNT;
        // 如果当前值小于上次记录的值，说明计数器已溢出，增加溢出计数
        if (cnt_now < cyclist_last)
        {
            cyclist_overflow_count++;
        }
        // 更新上次记录的计数值
        cyclist_last = DWT->CYCCNT;
        bit_blocker = 0;
    }
}

/* 公有函数 ---------------------------------------------------------*/

/**
 * @brief 初始化DWT（Data Watchpoint and Trace）模块
 * @note 使能DWT CYCLIST计数器，并根据CPU频率设置相关变量
 */
void Dwt_Init(void)
{
    const uint32_t cpu_freq_hz = HAL_RCC_GetHCLKFreq();
    // 使能调试跟踪功能
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    // 清零CYCLIST计数器
    DWT->CYCCNT = 0;
    // 使能CYCLIST计数器
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    // 根据获取的系统时钟频率计算不同单位下的频率值
    cpu_freq_hz_s = cpu_freq_hz;                     // 每秒计数
    cpu_freq_hz_ms = cpu_freq_hz / 1000;             // 每毫秒计数
    cpu_freq_hz_us = cpu_freq_hz/1000000;            // 每微秒计数

    // 初始化计数器溢出计数为0
    cyclist_overflow_count = 0;
    // 更新计数器状态
    Dwt_Cnt_Update();
}

/**
 * @brief 获取两个时间点之间的时间差(单精度浮点数)
 * @note 计算从上次记录的时间点到当前时间点经过的时间，单位为秒
 * @param cnt_last 指向记录上次计数值的指针
 * @return float 时间差，单位为秒
 */
float Get_Time_Delta(uint32_t *cnt_last)
{
    const volatile uint32_t cnt_now = DWT->CYCCNT;
    // 处理溢出情况
    uint32_t delta = cnt_now - *cnt_last;
    if (delta > UINT32_MAX / 2)
    {
        delta = UINT32_MAX - *cnt_last + cnt_now;
    }
    // 计算时间差，单位为秒
    const float dt = (float)delta / (float)cpu_freq_hz_s;
    *cnt_last = cnt_now;
    Dwt_Cnt_Update();
    return dt;

}

/**
 * @brief 获取两个时间点之间的时间差(双精度浮点数)
 * @note 计算从上次记录的时间点到当前时间点经过的时间，单位为秒
 * @param cnt_last 指向记录上次计数值的指针
 * @return double 时间差，单位为秒
 */
double Get_Time_Delta64(uint32_t *cnt_last)
{
    const volatile uint32_t cnt_now = DWT->CYCCNT;
    const double dt = (cnt_now-*cnt_last)/(double)cpu_freq_hz_s;
    *cnt_last = cnt_now;
    Dwt_Cnt_Update();
    return dt;
}

/**
 * @brief 更新系统时间
 * @note  根据DWT计数器的值更新系统时间结构体，包括秒、毫秒和微秒
 */
void Dwt_Sys_Time_Update(void)
{
    const volatile uint32_t cnt_now = DWT->CYCCNT;
    static uint64_t cnt_temp1, cnt_temp2, cnt_temp3;
    Dwt_Cnt_Update();
    // 构造64位计数值：溢出次数*最大值 + 当前计数值
    cyclist64 = (uint64_t)cyclist_overflow_count * (uint64_t)UINT32_MAX + (uint64_t)cnt_now;
    // 计算秒数
    cnt_temp1 = cyclist64 / cpu_freq_hz_s;
    cnt_temp2 = cyclist64 - cnt_temp1 * cpu_freq_hz_s;
    sys_time.s = cnt_temp1;
    // 计算毫秒数
    sys_time.ms = cnt_temp2 / cpu_freq_hz_ms;
    cnt_temp3 = cnt_temp2 - cnt_temp3 * cpu_freq_hz_ms;
    // 计算微秒数
    sys_time.us = cnt_temp3 / cpu_freq_hz_us;
}

/**
 * @brief 获取系统时间线(秒)
 * @note 返回系统运行的总时间，单位为秒，精度到微秒
 * @return float 系统运行时间，单位为秒
 */
float Dwt_Get_Time_Line_S(void)
{
    Dwt_Sys_Time_Update();
    const float dwt_time_line_f32 = (float)sys_time.s + (float)sys_time.ms / 1000.0f + (float)sys_time.us / 1000000.0f;
    return dwt_time_line_f32;
}

/**
 * @brief 获取系统时间线(毫秒)
 * @note 返回系统运行的总时间，单位为毫秒，精度到微秒
 * @return float 系统运行时间，单位为毫秒
 */
float Dwt_Get_Time_Line_Ms(void)
{
    Dwt_Sys_Time_Update();
    const float dwt_time_line_f32 = (float)sys_time.s * 1000.0f + (float)sys_time.ms + (float)sys_time.us / 1000.0f;
    return dwt_time_line_f32;
}

/**
 * @brief 获取系统时间线(微秒)
 * @note 返回系统运行的总时间，单位为微秒
 * @return uint32_t 系统运行时间，单位为微秒
 */
uint32_t Dwt_Get_Time_Line_Us(void)
{
    Dwt_Sys_Time_Update();
    const uint64_t dwt_time_line_f64 = sys_time.s * 1000000 + sys_time.ms * 1000 + sys_time.us;
    return dwt_time_line_f64;
}

/**
 * @brief 阻塞式延时函数(秒)
 * @note 使用DWT计数器实现精确延时，单位为秒,范围为1-255秒
 * @param delay_time 延时时间，单位为秒
 */
void Dwt_Delay_S(const uint8_t delay_time)
{
    const uint32_t tick_start = DWT->CYCCNT;
    const uint8_t wait_time = delay_time;
    while (DWT->CYCCNT - tick_start < wait_time * cpu_freq_hz_s);
}

/**
 * @brief 阻塞式延时函数(毫秒)
 * @note 使用DWT计数器实现精确延时，单位为毫秒，范围为1-65535毫秒
 * @param delay_time 延时时间，单位为毫秒
 */
void Dwt_Delay_Ms(const uint16_t delay_time)
{
    const uint32_t tick_start = DWT->CYCCNT;
    const uint16_t wait_time = delay_time ;
    while (DWT->CYCCNT - tick_start < wait_time * cpu_freq_hz_ms);
}

/**
 * @brief 阻塞延式时函数(微秒)
 * @note 使用DWT计数器实现精确延时，单位为微秒，范围为1-65535微秒
 * @param delay_time 延时时间，单位为微秒
 */
void Dwt_Delay_Us(const uint16_t delay_time)
{
    const uint32_t tick_start = DWT->CYCCNT;
    const uint16_t wait_time = delay_time ;
    while (DWT->CYCCNT - tick_start < wait_time * cpu_freq_hz_us);
}
