

#ifndef BSP_FDCAN_H_
#define BSP_FDCAN_H_
#include "robot_config.h"
#ifdef USER_CAN_FD
#include "fdcan.h"

#define FDCAN_MAX_REGISTER_CNT 16

#pragma pack(1)
typedef struct CanInstance_s
{
    char* topic_name;
    FDCAN_HandleTypeDef *can_handle;                      // FDCAN 句柄
    FDCAN_TxHeaderTypeDef tx_conf;                        // FDCAN 报文发送配置
    uint16_t tx_id;                                       // 发送 id, 即发送的 FDCAN 报文 id
    uint8_t tx_buff[8];                                   // 发送缓存, 可以不用，但建议保留，方便调试
    uint8_t* tx_buff_ptr;
    uint16_t rx_id;                                       // 接收 id, 即接收的 FDCAN 报文 id
    uint8_t rx_buff[8];                                   // 接收缓存, 目前保留，增加了一次 内存拷贝 操作，方便调试
    void (*can_module_callback)(struct CanInstance_s *); // 接收的回调函数, 用于解析接收到的数据，如果增加了 uint8_t *rx_buff 成员，前面的rx_buff[8] 可以删去
    void *parent_ptr;                                 // 使用 can 外设的模块指针 (即 id 指向的模块拥有此 can 实例, 是父子关系)
}CanInstance_s;

typedef struct
{
    char* topic_name;                  //实例名称
    uint8_t can_number;               //can通道号 1,2,3 分别对应 FDCAN1, FDCAN2, FDCAN3，为了抽象接口向module层隐藏HAL库
    uint16_t tx_id;                    //发送id
    uint16_t rx_id;                    //接收id
    void (*can_module_callback)(CanInstance_s *);   //接收的回调函数, 用于解析接收到的数据
    void *parent_ptr;                                   //使用 can 外设的父指针 (即 id 指向的模块拥有此 can 实例, 是父子关系)
}CanInitConfig_s;
#pragma pack()
/**
 * @brief FDCAN接收帧结构体。
 * 该结构体用于存储从FDCAN接收的消息，包括消息头和数据缓冲区。
 */
typedef struct {
    FDCAN_RxHeaderTypeDef Header;
    uint8_t 			rx_buff[8];
}FDCAN_RxFrame_TypeDef;

CanInstance_s* Can_Register(const CanInitConfig_s* config);
bool Can_Transmit_External_Tx_Buff(const CanInstance_s *instance, const uint8_t *tx_buff);
#endif
#endif
