/**
 ******************************************************************************
 * @file	bsp_dwt.c
 * @author  Wang Hongxi
 * @version V1.1.0
 * @date    2022/3/8
 * @brief
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#include "bsp_dwt.h"
#include "main.h"
#include "stdint.h"

#define  DWT_CR      *(__IO uint32_t *)0xE0001000
#define  DWT_CYCCNT  *(__IO uint32_t *)0xE0001004
#define  DEM_CR      *(__IO uint32_t *)0xE000EDFC

#define  DEM_CR_TRCENA                   (1 << 24)
#define  DWT_CR_CYCCNTENA                (1 <<  0)

static uint32_t  hclk_freq = 0 ;
static uint32_t  per_us_count = 0 ;
static uint32_t  per_ms_count = 0 ;
static uint32_t  per_s_count = 0 ;
void Dwt_Init(void){
    hclk_freq = HAL_RCC_GetHCLKFreq();
    per_s_count = hclk_freq;
    per_ms_count = hclk_freq / 1000u;
    per_us_count = hclk_freq / 1000000u;
    // DEM_CR |= (uint32_t)DEM_CR_TRCENA;
    // DWT_CYCCNT = (uint32_t)0u;
    // DWT_CR |= (uint32_t)DWT_CR_CYCCNTENA;

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    /* DWT CYCCNT寄存器计数清0 */
    DWT->CYCCNT = (uint32_t)0u;
    /* 使能Cortex-M DWT CYCCNT寄存器 */
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}
/**
 * @brief 微秒级延时函数
 * @param us 延时微秒数
 * @note 该函数基于DWT实现，确保SysTick定时器已初始化
 */
void Dwt_delay_us(uint16_t us)
{
    uint32_t start_tick = DWT->CYCCNT;
    uint32_t delay_ticks = us * per_us_count;
    while ((DWT->CYCCNT - start_tick) < delay_ticks);
}
// void Dwt_delay_us(uint16_t us)
// {
//     // 使能DWT
//     if (!(CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk))
//         CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
//     DWT->CYCCNT = 0;
//     DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
//     uint32_t ticks = us * (SystemCoreClock / 1000000);
//     uint32_t start = DWT->CYCCNT;
//     while ((DWT->CYCCNT - start) < ticks);
// }

// void Dwt_delay_us(uint16_t us)
// {
//
//     uint32_t ticks = 0;
//     uint32_t told = 0;
//     uint32_t tnow = 0;
//     uint32_t tcnt = 0;
//     uint32_t reload = 0;
//     reload = SysTick->LOAD;
//     ticks = us * 480;
//     told = SysTick->VAL;
//     while (1){
//
//     }
    //     tnow = SysTick->VAL;
    //     if (tnow != told)
    //     {
    //         if (tnow < told)
    //         {
    //             tcnt += told - tnow;
    //         }
    //         else
    //         {
    //             tcnt += reload - tnow + told;
    //         }
    //         told = tnow;
    //         if (tcnt >= ticks)
    //         {
    //             break;
    //         }
    //     }
    // }

/**
 * @brief 毫秒级延时函数
 * @param ms 延时毫秒数
 * @note 该函数调用Dwt_delay_us实现毫秒级延时
 */
void Dwt_delay_ms(uint16_t ms)
{
    while(ms--)
    {
        Dwt_delay_us(1000);
    }
}