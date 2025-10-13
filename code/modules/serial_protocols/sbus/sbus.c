#include "sbus.h"
#include <stdbool.h>
/**
 * @brief 解析SBUS数据帧
 * @param frame SBUS数据结构体指针
 * @param buffer SBUS数据帧缓冲区指针
 * @return 解析成功返回true，失败返回false
 */
// ReSharper disable once CppParameterMayBeConstPtrOrRef
bool Sbus_Frame_Parse(SBUS_Data_s* frame, uint8_t* buffer){
    frame->rx_cnt++;
    if (buffer[0] != SBUS_HEADER || buffer[24] != SBUS_FOOTER){
        frame->state = SBUS_FRAME_ERROR;
        return false;
    }
    if (buffer[23] == SBUS_NO_ONLINE){
        frame->state = SBUS_LOST;
        return false;
    }
    frame->state = SBUS_OK;
    frame->ch[0]  = ((buffer[1]       | buffer[2]  << 8) & 0x07FF);
    frame->ch[1]  = ((buffer[2]  >> 3 | buffer[3]  << 5) & 0x07FF);
    frame->ch[2]  = ((buffer[3]  >> 6 | buffer[4]  << 2  | buffer[5] << 10) & 0x07FF);
    frame->ch[3]  = ((buffer[5]  >> 1 | buffer[6]  << 7) & 0x07FF);
    frame->ch[4]  = ((buffer[6]  >> 4 | buffer[7]  << 4) & 0x07FF);
    frame->ch[5]  = ((buffer[7]  >> 7 | buffer[8]  << 1  | buffer[9] << 9) & 0x07FF);
    frame->ch[6]  = ((buffer[9]  >> 2 | buffer[10] << 6) & 0x07FF);
    frame->ch[7]  = ((buffer[10] >> 5 | buffer[11] << 3) & 0x07FF);
    frame->ch[8]  = ((buffer[12]      | buffer[13] << 8) & 0x07FF);
    frame->ch[9]  = ((buffer[13] >> 3 | buffer[14] << 5) & 0x07FF);
    frame->ch[10] = ((buffer[14] >> 6 | buffer[15] << 2  | buffer[16] << 10) & 0x07FF);
    frame->ch[11] = ((buffer[16] >> 1 | buffer[17] << 7) & 0x07FF);
    frame->ch[12] = ((buffer[17] >> 4 | buffer[18] << 4) & 0x07FF);
    frame->ch[13] = ((buffer[18] >> 7 | buffer[19] << 1  | buffer[20] << 9) & 0x07FF);
    frame->ch[14] = ((buffer[20] >> 2 | buffer[21] << 6) & 0x07FF);
    frame->ch[15] = ((buffer[21] >> 5 | buffer[22] << 3) & 0x07FF);
    return true;
}
