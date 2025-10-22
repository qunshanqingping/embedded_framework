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
/**
 * @brief 微秒级延时函数
 * @param us 延时微秒数
 * @note 该函数基于DWT实现，确保SysTick定时器已初始化
 */
void Dwt_delay_us(uint16_t us)
{

    uint32_t ticks = 0;
    uint32_t told = 0;
    uint32_t tnow = 0;
    uint32_t tcnt = 0;
    uint32_t reload = 0;
    reload = SysTick->LOAD;
    ticks = us * 480;
    told = SysTick->VAL;
    while (1)
    {
        tnow = SysTick->VAL;
        if (tnow != told)
        {
            if (tnow < told)
            {
                tcnt += told - tnow;
            }
            else
            {
                tcnt += reload - tnow + told;
            }
            told = tnow;
            if (tcnt >= ticks)
            {
                break;
            }
        }
    }
}

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