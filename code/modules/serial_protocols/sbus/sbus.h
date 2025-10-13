/**
 * @file sbus.h
 * @author Adonis Jin
 * @date 25-10-13
 * @brief sbus解码
 */
#ifndef SBUS_H
#define SBUS_H

#include <stdint.h>

#define SBUS_FRAME_SIZE      25      // SBUS帧长度
#define SBUS_HEADER          0x0F    // 帧头
#define SBUS_FOOTER          0x00    // 帧尾
#define SBUS_NO_ONLINE       0x0C    // 无线失联标志
#define SBUS_CHANNELS        16      // 通道数量
#define SBUS_MIN_VALUE       172     // 通道最小值
#define SBUS_MID_VALUE       992     // 通道中间值
#define SBUS_MAX_VALUE       1811    // 通道最大值

typedef enum{
    SBUS_LOST = 0, // 失联
    SBUS_OK, // 正常
} SBUS_Status_e;

typedef struct{
    SBUS_Status_e state;
    int16_t ch[16];
    uint16_t frequency;
    uint16_t rx_cnt;
} SbusData_s;
/**
 * @brief 解析SBUS数据帧
 * @param frame SBUS数据结构体指针
 * @param buffer SBUS数据帧缓冲区指针
 */
// ReSharper disable once CppParameterMayBeConstPtrOrRef
void Sbus_Frame_Parse(SbusData_s* frame, uint8_t* buffer);

#endif //SBUS_H
