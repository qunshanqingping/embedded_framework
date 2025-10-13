#include "sbus.h"
#include <stdbool.h>
#include <string.h>
/**
 * @brief 解析SBUS数据帧
 * @param frame SBUS数据结构体指针
 * @param buffer SBUS数据帧缓冲区指针
 */
// ReSharper disable once CppParameterMayBeConstPtrOrRef
void Sbus_Frame_Parse(SbusData_s* frame, uint8_t* buffer){
    frame->rx_cnt++;
    uint8_t header = 0;
    uint8_t buffer_temp[SBUS_FRAME_SIZE] = {0};
    if (buffer[0] != SBUS_HEADER || buffer[24] != SBUS_FOOTER){
        // frame->state = SBUS_FRAME_ERROR;
        while (buffer[header] != SBUS_HEADER && buffer[header-1] != SBUS_FRAME_SIZE){
            header++;
        }
    }
    if (header == 0){
        memcpy(buffer_temp, buffer, SBUS_FRAME_SIZE);
    }
    else{
        memcpy(&buffer_temp[0], &buffer[header], SBUS_FRAME_SIZE - header);
        memcpy(&buffer_temp[SBUS_FRAME_SIZE - header], &buffer[0], header);
    }
    if (buffer_temp[23] == SBUS_NO_ONLINE){
        frame->state = SBUS_LOST;
    }
    frame->state = SBUS_OK;
    frame->ch[0]  = (int16_t)(((uint16_t)buffer_temp[1]         | ((uint16_t)buffer_temp[2]  << 8)) & 0x07FF);
    frame->ch[1]  = (int16_t)((((uint16_t)buffer_temp[2]  >> 3) | ((uint16_t)buffer_temp[3]  << 5)) & 0x07FF);
    frame->ch[2]  = (int16_t)((((uint16_t)buffer_temp[3]  >> 6) | ((uint16_t)buffer_temp[4]  << 2)  | ((uint16_t)buffer_temp[5] << 10)) & 0x07FF);
    frame->ch[3]  = (int16_t)((((uint16_t)buffer_temp[5]  >> 1) | ((uint16_t)buffer_temp[6]  << 7)) & 0x07FF);
    frame->ch[4]  = (int16_t)((((uint16_t)buffer_temp[6]  >> 4) | ((uint16_t)buffer_temp[7]  << 4)) & 0x07FF);
    frame->ch[5]  = (int16_t)((((uint16_t)buffer_temp[7]  >> 7) | ((uint16_t)buffer_temp[8]  << 1)  | ((uint16_t)buffer_temp[9] << 9)) & 0x07FF);
    frame->ch[6]  = (int16_t)((((uint16_t)buffer_temp[9]  >> 2) | ((uint16_t)buffer_temp[10] << 6)) & 0x07FF);
    frame->ch[7]  = (int16_t)((((uint16_t)buffer_temp[10] >> 5) | ((uint16_t)buffer_temp[11] << 3)) & 0x07FF);
    frame->ch[8]  = (int16_t)(((uint16_t)buffer_temp[12]        | ((uint16_t)buffer_temp[13] << 8)) & 0x07FF);
    frame->ch[9]  = (int16_t)((((uint16_t)buffer_temp[13] >> 3) | ((uint16_t)buffer_temp[14] << 5)) & 0x07FF);
    frame->ch[10] = (int16_t)((((uint16_t)buffer_temp[14] >> 6) | ((uint16_t)buffer_temp[15] << 2)  | ((uint16_t)buffer_temp[16] << 10)) & 0x07FF);
    frame->ch[11] = (int16_t)((((uint16_t)buffer_temp[16] >> 1) | ((uint16_t)buffer_temp[17] << 7)) & 0x07FF);
    frame->ch[12] = (int16_t)((((uint16_t)buffer_temp[17] >> 4) | ((uint16_t)buffer_temp[18] << 4)) & 0x07FF);
    frame->ch[13] = (int16_t)((((uint16_t)buffer_temp[18] >> 7) | ((uint16_t)buffer_temp[19] << 1)  | ((uint16_t)buffer_temp[20] << 9)) & 0x07FF);
    frame->ch[14] = (int16_t)((((uint16_t)buffer_temp[20] >> 2) | ((uint16_t)buffer_temp[21] << 6)) & 0x07FF);
    frame->ch[15] = (int16_t)((((uint16_t)buffer_temp[21] >> 5) | ((uint16_t)buffer_temp[22] << 3)) & 0x07FF);
}


