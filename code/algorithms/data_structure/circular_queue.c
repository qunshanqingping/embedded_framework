#include "circular_queue.h"
#include <stdlib.h>
#include <string.h>
#include "memory_management.h"
#include "robot_config.h"
#include "plf_log.h"
/* 检查队列是否为空 */
static bool CircularQueue_Is_Empty(const CircularQueue_s *queue) {
    return (queue->count == 0);
}

/* 检查队列是否已满 */
static bool CircularQueue_Is_Full(const CircularQueue_s *queue) {
    return (queue->count == queue->capacity);
}

/* 获取队列中元素数量 */
uint16_t CircularQueue_Get_Size(const CircularQueue_s *queue) {
    return queue->count;
}

/* 创建循环队列 */
CircularQueue_s *CircularQueue_Create(const uint16_t capacity, const uint16_t element_size) {
    if (capacity == 0 || element_size == 0) {
        Log_Error("CircularQueue_Create capacity or element_size is 0");
        return NULL;
    }

    CircularQueue_s *queue = (CircularQueue_s *) user_malloc(sizeof(CircularQueue_s));
    if (queue == NULL) {
        Log_Error("CircularQueue_Create queue malloc fail");
        return NULL;
    }

    // 分配数据缓冲区
    queue->data = user_malloc(capacity * element_size);
    if (queue->data == NULL) {
        Log_Error("CircularQueue_Create queue data malloc fail");
        user_free(queue);
        return NULL;
    }

    queue->front = 0;
    queue->rear = 0;
    queue->capacity = capacity;
    queue->element_size = element_size;
    queue->count = 0;

    return queue;
}

/* 销毁循环队列 */
void CircularQueue_Destroy(CircularQueue_s *queue) {
    if (queue != NULL) {
        if (queue->data != NULL) {
            user_free(queue->data);
        }
        user_free(queue);
    }
}

/* 入队操作 */
bool CircularQueue_Enqueue(CircularQueue_s *queue, const void *data) {
    if (queue == NULL || data == NULL) {
        Log_Error("CircularQueue_Enqueue queue or data is NULL");
        return false;
    }

    // 检查队列是否已满
    if (CircularQueue_Is_Full(queue)) {
        queue->front = (queue->front + 1) % queue->capacity;
        queue->count--;
    }

    // 计算插入位置的地址
    void *insert_pos = (char *) queue->data + (queue->rear * queue->element_size);

    // 拷贝数据到队列
    memcpy(insert_pos, data, queue->element_size);

    // 更新队尾指针和元素计数
    queue->rear = (queue->rear + 1) % queue->capacity;
    queue->count++;

    return true;
}

/* 出队操作 */
bool CircularQueue_Dequeue(CircularQueue_s *queue, void *data) {
    if (queue == NULL || data == NULL) {
        return false;
    }

    // 检查队列是否为空
    if (CircularQueue_Is_Empty(queue)) {
        return false;
    }

    // 计算队头元素的地址
    const void *front_pos = (char *) queue->data + (queue->front * queue->element_size);

    // 将队头数据拷贝到输出参数
    memcpy(data, front_pos, queue->element_size);

    // 更新队头指针和元素计数
    queue->front = (queue->front + 1) % queue->capacity;
    queue->count--;

    return true;
}


/* 清空队列 */
void CircularQueue_Clear(CircularQueue_s *queue) {
    if (queue != NULL) {
        queue->front = 0;
        queue->rear = 0;
        queue->count = 0;
    }
}
