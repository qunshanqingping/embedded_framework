#ifndef CIRCULAR_QUEUE_H
#define CIRCULAR_QUEUE_H

#include <stdint.h>
#include <stdbool.h>

// 循环队列结构体
typedef struct{
    void *data;           /* 数据缓冲区 */
    uint16_t front;       /* 队头索引 */
    uint16_t rear;        /* 队尾索引 */
    uint16_t capacity;    /* 队列容量 */
    uint16_t element_size; /* 每个元素的大小 */
    uint16_t count;       /* 当前元素数量 */
}CircularQueue_s;


/* 创建循环队列 */
CircularQueue_s* CircularQueue_Create(uint16_t capacity, uint16_t element_size);

/* 销毁循环队列 */
void CircularQueue_Destroy(CircularQueue_s* queue);

/* 入队操作 */
bool CircularQueue_Enqueue(CircularQueue_s* queue, const void* data);

/* 出队操作 */
bool CircularQueue_Dequeue(CircularQueue_s* queue, void* data);


/* 获取队列中元素数量 */
uint16_t CircularQueue_Get_Size(const CircularQueue_s* queue);

/* 清空队列 */
void CircularQueue_Clear(CircularQueue_s* queue);

#endif //CIRCULAR_QUEUE_H