#include "fs_i6x.h"
#include "basic_math.h"

/**
 * @brief 解析两位开关
 * @param ch 通道值
 * @param mid_val 中值
 * @return 开关状态 0,1
 */
inline static uint8_t remoter_2stage_switch_parse(int16_t ch, int16_t mid_val){
    if (ch < mid_val){
        return 0;
    }
    return 1;
}

/**
 * @brief 解析三位开关
 * @param ch 通道值
 * @param mid_val0 第一个中值
 * @param mid_val1 第二个中值
 * @return 开关状态 0,1,2
 */
inline static uint8_t remoter_3stage_switch_parse(int16_t ch, int16_t mid_val0, int16_t mid_val1){
    if (ch > 0 && ch < mid_val0){
        return 0;
    }
    if (ch >= mid_val0 && ch < mid_val1){
        return 1;
    }
    return 2;
}

