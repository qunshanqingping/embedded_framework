#include "bsp_dwt.h"
#include "main.h"
#include <stdint.h>

// HCLK 频率 Hz
static uint32_t hclk_freq = 0;
// 每微秒、每毫秒和每秒的计数值
static uint32_t per_us_count = 0;
static uint32_t per_ms_count = 0;
static uint32_t per_s_count = 0;

/**
 * @brief DWT延时初始化函数
 * @note 该函数用于初始化DWT计数器，必须在使用延时函数前调用
 */
void Dwt_Init(void){
    // 获取HCLK频率 Hz
    hclk_freq = HAL_RCC_GetHCLKFreq();
    // 计算每秒、每毫秒和每微秒的计数值
    per_s_count = hclk_freq;
    per_ms_count = hclk_freq / 1000u;
    per_us_count = hclk_freq / 1000000u;
    // 启用DWT计数器
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    // DWT计数器清零并启动
    DWT->CYCCNT = (uint32_t)0u;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

/**
 * @brief 微秒级延时函数
 * @param us 延时微秒数
 */
void Dwt_delay_us(const uint16_t us){
    // 获取当前DWT计数值
    const uint32_t start_tick = DWT->CYCCNT;
    // 计算延时所需的计数值
    const uint32_t delay_ticks = us * per_us_count;
    const uint32_t target_tick = start_tick + delay_ticks;
    // 处理计数器溢出情况
    if (start_tick > target_tick){
        while (DWT->CYCCNT <= UINT32_MAX);
    }
    // 等待直到达到目标计数值
    while (DWT->CYCCNT < target_tick);
}

/**
 * @brief 毫秒级延时函数
 * @param ms 延时毫秒数
 */
void Dwt_delay_ms(const uint16_t ms){
    // 获取当前DWT计数值
    const uint32_t start_tick = DWT->CYCCNT;
    // 计算延时所需的计数值
    const uint32_t delay_ticks = ms * per_ms_count;
    const uint32_t target_tick = start_tick + delay_ticks;
    // 处理计数器溢出情况
    if (start_tick > target_tick){
        while (DWT->CYCCNT <= UINT32_MAX);
    }
    // 等待直到达到目标计数值
    while (DWT->CYCCNT < target_tick);
}