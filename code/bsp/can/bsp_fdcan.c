#include "robot_config.h"
#ifdef USER_CAN_FD
#include "bsp_fdcan.h"
#include "FreeRTOS.h"
#include "plf_log.h"
#include "memory_management.h"
#include <string.h>
#include <stdbool.h>
/* 私有类型定义 -----------------------------------------------------------------*/

/* FDCAN 实例声明 */
#if defined USER_CAN1
static uint8_t fdcan_idx1;                                     /*!< fdcan1实例索引 */
static CanInstance_s *fdcan1_instance[FDCAN_MAX_REGISTER_CNT]; /*!< fdcan1实例数组 */
#endif

#if defined USER_CAN2
static uint8_t fdcan_idx2;                                     /*!< fdcan2实例索引 */
static CanInstance_s *fdcan2_instance[FDCAN_MAX_REGISTER_CNT]; /*!< fdcan2实例数组 */
#endif

#if defined USER_CAN3
static uint8_t can_idx3;                                       /*!< fdcan3实例索引 */
static CanInstance_s *fdcan3_instance[FDCAN_MAX_REGISTER_CNT]; /*!< fdcan3实例数组 */
#endif

/**
 * @brief FDCAN 初始化标志
 */
static bool fdcan_init_flag = false;

#if defined (USER_CAN1_FIFO_0) || defined (USER_CAN2_FIFO_0) || defined (USER_CAN3_FIFO_0)
/**
 * @brief FIFO0的接收帧指针
 */
static FDCAN_RxFrame_TypeDef *FDCAN_RxFIFO0Frame;

/**
 * @brief FIFO0过滤器
 */
FDCAN_FilterTypeDef FDCAN_x_FIFO0_Filter = {
    .IdType = FDCAN_STANDARD_ID,            /*!< 标准 ID */
    .FilterIndex = 0 ,                       /*!< 过滤器编号 */
    .FilterType = FDCAN_FILTER_MASK,         /*!< 过滤器 Mask 模式 关乎到 ID1ID2 的配置 */
    .FilterConfig = FDCAN_FILTER_TO_RXFIFO0, /*!< 过滤器配置到 FIFO0 */
    .FilterID1 = 0x00000000,                 /*!< 过滤器 ID1，只要 ID2 配置为 0x00000000，就不会过滤任何 ID */
    .FilterID2 = 0x00000000,                 /*!< 过滤器 ID2, 只要 ID2 配置为 0x00000000，就不会过滤任何 ID */
};
#endif

#if defined (USER_CAN1_FIFO_1) || defined (USER_CAN2_FIFO_1) || defined (USER_CAN3_FIFO_1)
/**
 * @brief FIFO1的接收帧指针
 */
static FDCAN_RxFrame_TypeDef *FDCAN_RxFIFO1Frame;

/**
 * @brief FIFO1过滤器
 */
FDCAN_FilterTypeDef FDCAN_x_FIFO1_Filter = {
    .IdType = FDCAN_STANDARD_ID,            /*!< 标准 ID */
    .FilterIndex = 0 ,                       /*!< 过滤器编号 */
    .FilterType = FDCAN_FILTER_MASK,         /*!< 过滤器 Mask 模式 关乎到 ID1ID2 的配置 */
    .FilterConfig = FDCAN_FILTER_TO_RXFIFO1, /*!< 过滤器配置到 FIFO1 */
    .FilterID1 = 0x00000000,                 /*!< 过滤器 ID1，只要 ID2 配置为 0x00000000，就不会过滤任何 ID */
    .FilterID2 = 0x00000000,                 /*!< 过滤器 ID2, 只要 ID2 配置为 0x00000000，就不会过滤任何 ID */
};
#endif

/* 私有函数 ---------------------------------------------------------------------*/
/**
 * @brief 初始化 FDCAN 过滤器配置。
 *
 * 该函数根据用户定义的宏（USER_CAN1, USER_CAN2, USER_CAN3）初始化相应的 FDCAN 实例的过滤器。对于每个启用的 FDCAN 实例，它首先确定 ID 类型（标准或扩展），然后选择接收 FIFO（FIFO0 或 FIFO1）。如果两个 FIFO 都没有元素，则记录错误并返回 false。接着，配置过滤器参数，包括 ID 类型、过滤器索引、过滤器类型和过滤器配置。此外，还配置了全局过滤器以拒绝不匹配的标准 ID 和扩展 ID 以及远程帧，并设置了 FIFO 水印中断。如果在配置过程中出现任何错误，将记录错误日志并重试直到成功。
 * 注意：此函数依赖于 HAL 库提供的 FDCAN 相关 API。
 * @return 如果所有 FDCAN 实例的过滤器配置成功，返回 true；否则返回 false。
 */
static void FDCAN_Filter_Init(void){
#if defined USER_CAN1_FIFO_0
    /* 配置 CAN1 FIFO0 过滤器 */
    while (HAL_FDCAN_ConfigFilter(&hfdcan1, &FDCAN_x_FIFO0_Filter) != HAL_OK) {
        Log_Error("FDCAN1 Fifo0 configs filter failed");
    }
    /* 水印中断，接受 1 条消息触发中断 */
    while (HAL_FDCAN_ConfigFifoWatermark(&hfdcan1, FDCAN_CFG_RX_FIFO0, 1) != HAL_OK) {
        Log_Error("FDCAN1 Fifo0 configs fifo watermark failed");
    }
    /* 激活 FIFO0 新消息中断 */
    while (HAL_FDCAN_ActivateNotification(&hfdcan1,FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) {
        Log_Error("FDCAN1 Fifo0 configs interruption failed");
    }
#endif
#if defined USER_CAN1_FIFO_1
    /* 配置 CAN1 FIFO1 过滤器 */
    while (HAL_FDCAN_ConfigFilter(&hfdcan1, &FDCAN_x_FIFO1_Filter) != HAL_OK) {
        Log_Error("FDCAN1 Fifo1 configs filter failed");
    }
    /* 水印中断，接受 1 条消息触发中断 */
    while (HAL_FDCAN_ConfigFifoWatermark(&hfdcan1, FDCAN_CFG_RX_FIFO1, 1) != HAL_OK) {
        Log_Error("FDCAN1 Fifo1 configs fifo watermark failed");
    }
    /* 激活 FIFO1 新消息中断 */
    while (HAL_FDCAN_ActivateNotification(&hfdcan1,FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0) != HAL_OK) {
        Log_Error("FDCAN1 Fifo1 configs interruption failed");
    }
#endif
#if defined USER_CAN2_FIFO_0
    /* 配置 CAN2 FIFO0 过滤器 */
    while (HAL_FDCAN_ConfigFilter(&hfdcan2, &FDCAN_x_FIFO0_Filter) != HAL_OK) {
        Log_Error("FDCAN2 Fifo0 configs filter failed");
    }
    /* 水印中断，接受 1 条消息触发中断 */
    while (HAL_FDCAN_ConfigFifoWatermark(&hfdcan2, FDCAN_CFG_RX_FIFO0, 1) != HAL_OK) {
        Log_Error("FDCAN2 Fifo0 configs fifo watermark failed");
    }
    /* 激活 FIFO0 新消息中断 */
    while (HAL_FDCAN_ActivateNotification(&hfdcan2,FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) {
        Log_Error("FDCAN2 Fifo0 configs interruption failed");
    }
#endif
#if defined USER_CAN2_FIFO_1
    /* 配置 CAN2 FIFO1 过滤器 */
    while (HAL_FDCAN_ConfigFilter(&hfdcan2, &FDCAN_x_FIFO1_Filter) != HAL_OK) {
        Log_Error("FDCAN2 Fifo1 configs filter failed");
    }
    /* 水印中断，接受 1 条消息触发中断 */
    while (HAL_FDCAN_ConfigFifoWatermark(&hfdcan2, FDCAN_CFG_RX_FIFO1, 1) != HAL_OK) {
        Log_Error("FDCAN2 Fifo1 configs fifo watermark failed");
    }
    /* 激活 FIFO1 新消息中断 */
    while (HAL_FDCAN_ActivateNotification(&hfdcan2,FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0) != HAL_OK) {
        Log_Error("FDCAN2 Fifo1 configs interruption failed");
    }
#endif
#if defined USER_CAN3_FIFO_0
    /* 配置 CAN3 FIFO0 过滤器 */
    while (HAL_FDCAN_ConfigFilter(&hfdcan3, &FDCAN_x_FIFO0_Filter) != HAL_OK) {
        Log_Error("FDCAN3 Fifo0 configs filter failed");
    }
    /* 水印中断，接受 1 条消息触发中断 */
    while (HAL_FDCAN_ConfigFifoWatermark(&hfdcan3, FDCAN_CFG_RX_FIFO0, 1) != HAL_OK) {
        Log_Error("FDCAN3 Fifo0 configs fifo watermark failed");
    }
    /* 激活 FIFO0 新消息中断 */
    while (HAL_FDCAN_ActivateNotification(&hfdcan2,FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) {
        Log_Error("FDCAN3 Fifo1 configs interruption failed");
    }
#endif
#if defined USER_CAN3_FIFO_1
    /* 配置 CAN3 FIFO1 过滤器 */
    while (HAL_FDCAN_ConfigFilter(&hfdcan3, &FDCAN_x_FIFO1_Filter) != HAL_OK) {
        Log_Error("FDCAN3 Fifo1 configs filter failed");
    }
    /* 水印中断，接受 1 条消息触发中断 */
    while (HAL_FDCAN_ConfigFifoWatermark(&hfdcan3, FDCAN_CFG_RX_FIFO1, 1) != HAL_OK) {
        Log_Error("FDCAN3 Fifo1 configs fifo watermark failed");
    }
    /* 激活 FIFO1 新消息中断 */
    while (HAL_FDCAN_ActivateNotification(&hfdcan2,FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0) != HAL_OK) {
        Log_Error("FDCAN3 Fifo1 configs interruption failed");
    }
#endif
}

/**
 * @brief 初始化CAN服务。
 *
 * 该函数根据用户配置的宏定义（USER_CAN1, USER_CAN2, USER_CAN3）来初始化相应的FDCAN硬件实例。对于每个启用的FDCAN实例，首先尝试启动它。如果启动失败，则记录错误日志并继续重试直到成功。随后，根据USER_CANx_FIFOx的值选择激活Rx FIFO 0或Rx FIFO 1的消息接收中断通知。如果激活中断通知失败，同样会记录错误日志，并且持续尝试直到成功为止。
 * 注意：此函数依赖于HAL库提供的FDCAN相关API。
 * @return 无返回值（void）。
 */
static void FDCAN_Service_Init(void){
#if defined USER_CAN1
    /* 配置全局过滤器，拒绝所有不匹配的标准ID和扩展ID以及远程帧 */
    while (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) !=HAL_OK) {
        Log_Error("FDCAN1 config global filter failed");
    }
    /* 启动 FDCAN1 */
    while (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
        Log_Error("FDCAN1 Starts Failed");
    }
#endif
#if defined USER_CAN2
    /* 配置全局过滤器，拒绝所有不匹配的标准ID和扩展ID以及远程帧 */
    while (HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) !=HAL_OK) {
        Log_Error("FDCAN2 config global filter failed");
    }
    /* 启动 FDCAN2 */
    while (HAL_FDCAN_Start(&hfdcan2) != HAL_OK) {
        Log_Error("FDCAN2 Starts Failed");
    }
#endif
#if defined USER_CAN3
    /* 配置全局过滤器，拒绝所有不匹配的标准ID和扩展ID以及远程帧 */
    while (HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) !=HAL_OK) {
        Log_Error("FDCAN2 config global filter failed");
    }
    /* 启动 FDCAN3 */
    while (HAL_FDCAN_Start(&hfdcan3) != HAL_OK) {
        Log_Error("FDCAN3 Starts Failed");
    }
#endif
}


/**
 * @brief 初始化CAN模块。
 * 该函数首先尝试初始化CAN过滤器，如果成功，则继续初始化CAN服务，
 * 并记录一条通过日志。如果CAN过滤器初始化失败，则直接记录一条错误日志。
 * @return 无返回值（void）。
 */
static void FDCAN_Init(void) {
    if (fdcan_init_flag == false){
        FDCAN_Filter_Init();       /* !<初始化 FDCAN 过滤器 */
        FDCAN_Service_Init();      /* !<初始化 FDCAN 服务 */
        fdcan_init_flag = true;    /* !<更新初始化标志 */
        Log_Passing("Can Init");   /* !<记录初始化成功日志 */
    }
}

/**
 * @brief 根据CAN编号选择对应的FDCAN句柄。
 * 该函数根据传入的CAN编号返回对应的FDCAN句柄指针。如果传入的编号不匹配任何已定义的CAN实例，则返回NULL。
 * @param can_number 要选择的CAN编号（1、2或3）。
 * @return 指向对应FDCAN句柄的指针；如果编号无效则返回NULL。
 */
static FDCAN_HandleTypeDef* FDCAN_Select_Handle(const uint8_t can_number){
#if defined USER_CAN1
    if (can_number == 1){
        return &hfdcan1;    /* !<返回FDCAN1句柄指针 */
    }
#endif
#if defined USER_CAN2
    if (can_number == 2){
        return &hfdcan2;    /* !<返回FDCAN2句柄指针 */
    }
#endif
#if defined USER_CAN3
    if (can_number == 3){
        return &hfdcan3;    /* !<返回FDCAN3句柄指针 */
    }
#endif
    return NULL;            /* !<无效编号，返回NULL */
}

static bool FDCAN_Register_Check(const CanInitConfig_s *config) {
    if (config == NULL) {                                    /* !<检查配置是否为空 */
        Log_Error("Can Init Config Is Null");
        return false;
    }
    if (config->topic_name == NULL) {                        /* !<检查实例名称是否为空 */
        Log_Error("Can Init Topic Name Is Null");
        return false;
    }
    if (config->can_number == 0 || config->can_number > 3) { /* !<检查CAN编号是否合法 */
        Log_Error("CanX %s Number Error", config->can_number, config->topic_name);
        return false;
    }
    if (config->tx_id == 0) {                                /* !<检查发送ID是否为0 */
        Log_Error("Can%d    %s : Tx ID Is 0x000", config->can_number, config->topic_name);
        return false;
    }
    if (config->rx_id == 0) {                                /* !<检查接收ID是否为0 */
        Log_Error("Can%d    %s : ID Is 0x000", config->can_number, config->topic_name);
        return false;
    }
#if defined USER_CAN1
    if (config->can_number == 1) {
        /* 检查是否超过CAN1最大实例数 */
        if (fdcan_idx1 == FDCAN_MAX_REGISTER_CNT) {
            Log_Error("Can1    %s : Max Register Count Reached", config->topic_name);
            return false;
        }/* 检查RX ID是否冲突 */
        for (uint8_t i = 0; i < fdcan_idx1; i++) {
            if (fdcan1_instance[i]->rx_id == config->rx_id) {
                Log_Error("Can1    %s : Rx ID 0x%03X Already Exists", config->topic_name, config->rx_id);
                return false;
            }
        }
    }
#endif
#if defined USER_CAN2
    if (config->can_number == 2) {
        /* 检查是否超过CAN2最大实例数 */
        if (fdcan_idx2 == FDCAN_MAX_REGISTER_CNT) {
            Log_Error("Can2    %s : Max Register Count Reached", config->topic_name);
            return false;
        } /* 检查RX ID是否冲突 */
        for (uint8_t i = 0; i < fdcan_idx2; i++) {
            if (fdcan2_instance[i]->rx_id == config->rx_id) {
                Log_Error("Can2    %s : Rx ID 0x%03X Already Exists", config->topic_name, config->rx_id);
                return false;
            }
        }
    }
#if defined USER_CAN3
    if (config->can_number == 3) {
        /* 检查是否超过CAN3最大实例数 */
        if (can_idx3 == FDCAN_MAX_REGISTER_CNT) {
            Log_Error("Can3    %s : Max Register Count Reached", config->topic_name);
            return false;
        } /* 检查RX ID是否冲突 */
        for (uint8_t i = 0; i < can_idx3; i++) {
            if (fdcan3_instance[i]->rx_id == config->rx_id) {
                Log_Error("Can3    %s : Rx ID 0x%03X Already Exists", config->topic_name, config->rx_id);
                return false;
            }
        }
    }
#endif
#endif
    if (config->can_module_callback == NULL) {              /* !<检查回调函数是否为空 */
        Log_Warning("%s Can Callback Is Null", config->topic_name);
    }
    return true;                                            /* !<配置检查通过 */
}


static void FDCAN_Register_to_fdcan_x_instance(CanInstance_s *instance) {

#if defined USER_CAN1
    if (instance->can_handle == &hfdcan1) {
        fdcan1_instance[fdcan_idx1++] = instance;
    }
#endif
#if defined USER_CAN2
    if (instance->can_handle == &hfdcan2){
        fdcan2_instance[fdcan_idx2++] = instance;
    }
#endif
#if defined USER_CAN3
    if (instance->can_handle == &hfdcan3)
    {
        fdcan3_instance[can_idx3++] = instance;
    }
#endif
    Log_Passing("%s RxID : 0x%03X TxID : 0x%03X", instance->topic_name, instance->rx_id, instance->tx_id);
}

/**
 * @brief FDCAN 接收回调函数。
 * 该函数在接收到FDCAN消息时被调用。它根据接收到的消息的ID查找对应的CAN实例，并将消息数据复制到该实例的接收缓冲区中。如果找到匹配的实例且其回调函数不为空，则调用该回调函数。
 * @param FDCAN_RxFIFOxFrame 指向包含接收到的FDCAN消息的FDCAN_RxFrame_TypeDef结构体的指针。
 * @param idx 接收到的消息数量。
 * @param fdcan_instance 指向包含所有注册的CAN实例的CanInstance_s指针数组。
 */
static void FDCAN_RxFifoCallback(const FDCAN_RxFrame_TypeDef *FDCAN_RxFIFOxFrame, const uint8_t idx, CanInstance_s **fdcan_instance) {
    if (idx == 0) {
        return;
    }
    for (uint8_t i = 0; i < idx; i++) {
        if ( FDCAN_RxFIFOxFrame->Header.Identifier == fdcan_instance[i]->rx_id) {
            if (fdcan_instance[i]->can_module_callback != NULL)
            {
                memcpy(fdcan_instance[i]->rx_buff, FDCAN_RxFIFOxFrame->rx_buff,FDCAN_RxFIFOxFrame->Header.DataLength);
                fdcan_instance[i]->can_module_callback(fdcan_instance[i]);
            }
            break;
        }
    }
}

/* 公共函数 ------------------------------------------------------------------*/

/**
 * @brief 注册一个新的CAN实例。
 * 根据提供的配置信息创建并初始化一个新的CAN实例。如果配置信息为空或内存分配失败，则函数将返回NULL。成功注册后，新的CAN实例将被添加到相应的FDCAN实例列表中，并返回指向新实例的指针。
 * @param config 指向包含CAN初始化配置信息（如句柄、发送ID等）的CanInitConfig_s结构的指针。
 * @return 如果成功注册则返回指向新创建的CanInstance_s结构的指针；如果配置无效或内存分配失败则返回NULL。
 */
CanInstance_s *Can_Register(const CanInitConfig_s *config) {
    FDCAN_Init();
    if (FDCAN_Register_Check(config) == false) {
        return NULL;
    }
    CanInstance_s *instance = user_malloc(sizeof(CanInstance_s)); // 分配空间
    if (instance == NULL) {
        Log_Error("%s CanInstance Malloc Failed", config->topic_name);
        return NULL; // 内存分配失败
    }
    memset(instance, 0, sizeof(CanInstance_s));
    instance->topic_name = config->topic_name;
    instance->tx_id = config->tx_id;
    instance->rx_id = config->rx_id;
    instance->can_module_callback = config->can_module_callback;
    instance->parent_ptr = config->parent_ptr;
    instance->tx_conf.Identifier = config->tx_id;
    instance->can_handle = FDCAN_Select_Handle(config->can_number);
    instance->tx_conf.IdType = FDCAN_STANDARD_ID; // 标准 ID
    instance->tx_conf.TxFrameType = FDCAN_DATA_FRAME;
    instance->tx_conf.DataLength = FDCAN_DLC_BYTES_8;
    instance->tx_conf.ErrorStateIndicator = FDCAN_ESI_ACTIVE; // 传输节点 error active
    instance->tx_conf.BitRateSwitch = FDCAN_BRS_OFF; // FDCAN 帧发送 / 接收不带波特率可变
    instance->tx_conf.FDFormat = FDCAN_CLASSIC_CAN; // 设置为经典 CAN 帧格式
    instance->tx_conf.TxEventFifoControl = FDCAN_NO_TX_EVENTS; // 不存储 Tx events 事件
    instance->tx_conf.MessageMarker = 0;

    FDCAN_Register_to_fdcan_x_instance(instance);

    return instance;
}

/**
 * @brief 通过外部提供的发送缓冲区发送CAN消息。
 * 该函数尝试将外部提供的发送缓冲区中的数据通过指定的CAN实例发送出去。如果发送缓冲区为空或CAN实例无效，函数将返回false。函数会检查CAN的发送FIFO是否有空闲位置，如果没有空闲位置则会等待，直到超时。如果成功将消息添加到发送FIFO队列中，函数将返回true；否则返回false。
 * @param instance 指向要使用的CanInstance_s结构的指针，表示要通过哪个CAN实例发送消息。
 * @param tx_buff 指向包含要发送数据的缓冲区的指针。
 * @return 如果消息成功添加到发送队列则返回true；如果发送缓冲区为空、CAN实例无效或发送失败则返回false。
 */
bool Can_Transmit_External_Tx_Buff(const CanInstance_s *instance, const uint8_t *tx_buff) {
    uint8_t fdcan_tx_cnt = 0;
    while (HAL_FDCAN_GetTxFifoFreeLevel(instance->can_handle) == 0) {
        fdcan_tx_cnt++;
        if (fdcan_tx_cnt > 100) {
            Log_Error("%s Can Transmit Timeout", instance->topic_name);
            return false;
        }
    }
    if (HAL_FDCAN_AddMessageToTxFifoQ(instance->can_handle, &instance->tx_conf, tx_buff) == HAL_OK) {
        return true;
    }
    return false;
}

/**
 * @brief 通过CAN总线发送数据,为了避免大修MODULE而写的函数
 * 该函数将实例内部的发送缓冲区数据通过给定的CAN实例发送出去。如果发送成功，返回true；否则返回false。
 * @param instance 指向已注册的CanInstance_s结构体的指针，表示要使用的CAN实例
 * @return 如果数据发送成功则返回true，否则返回false
 */
bool Can_Transmit(const CanInstance_s *instance) {
    uint8_t fdcan_tx_cnt = 0;
    while (HAL_FDCAN_GetTxFifoFreeLevel(instance->can_handle) == 0) {
        fdcan_tx_cnt++;
        if (fdcan_tx_cnt > 100) {
            Log_Error("%s Can Transmit Timeout", instance->topic_name);
            return false;
        }
    }
    if (HAL_FDCAN_AddMessageToTxFifoQ(instance->can_handle, &instance->tx_conf, instance->tx_buff_ptr) == HAL_OK) {
        return true;
    }
    return false;
}

/**
 * @brief FDCAN接收FIFO0中断的回调函数。
 *
 * 该函数处理接收FIFO0中的消息。当检测到新的消息时，它会根据配置调用相应的用户定义的回调函数。
 *
 * @param hfdcan 指向包含指定FDCAN配置信息的FDCAN_HandleTypeDef结构的指针。
 * @param RxFifo0ITs 触发此回调的中断源。
 */
// ReSharper disable once CppParameterMayBeConst
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
    if (RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) {
        HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &FDCAN_RxFIFO0Frame->Header, FDCAN_RxFIFO0Frame->rx_buff);

#if defined USER_CAN1_FIFO_0
        if (hfdcan == &hfdcan1){
            FDCAN_RxFifoCallback(FDCAN_RxFIFO0Frame,fdcan_idx1,fdcan1_instance);
        }
#endif
#if defined USER_CAN2_FIFO_0
        if (hfdcan == &hfdcan2){
            FDCAN_RxFifoCallback(FDCAN_RxFIFO0Frame,fdcan_idx2,fdcan2_instance);
        }
#endif
#if defined USER_CAN3_FIFO_0
        if (hfdcan == &hfdcan3){
            FDCAN_RxFifoCallback(FDCAN_RxFIFO0Frame,can_idx3,fdcan3_instance);
        }
#endif
    }
}

/**
 * @brief FDCAN接收FIFO1中断的回调函数。
 *
 * 该函数处理来自FDCAN接收FIFO1中的消息。当接收到新消息时，它会根据配置调用相应的用户定义回调函数。
 *
 * @param hfdcan 指向包含指定FDCAN配置信息的FDCAN_HandleTypeDef结构的指针。
 * @param RxFifo1ITs 触发此回调的中断源。
 */
// ReSharper disable once CppParameterMayBeConst
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs) {
    if (RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) {
        HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &FDCAN_RxFIFO1Frame->Header, FDCAN_RxFIFO1Frame->rx_buff);
#if defined USER_CAN1_FIFO_1
        if (hfdcan == &hfdcan1){
            FDCAN_RxFifoCallback(FDCAN_RxFIFO1Frame,fdcan_idx1, fdcan1_instance);
        }
#endif
#if defined USER_CAN2_FIFO_1
        if (hfdcan == &hfdcan2){
            FDCAN_RxFifoCallback(FDCAN_RxFIFO1Frame,fdcan_idx2,fdcan2_instance);
        }
#endif
#if defined USER_CAN3_FIFO_1
        if (hfdcan == &hfdcan3){
            FDCAN_RxFifoCallback(FDCAN_RxFIFO1Frame,can_idx3,fdcan3_instance);
        }
#endif
    }
}

//对HAL_FDCAN_ErrorStatusCallback的重载
//@todo 待完善
// ReSharper disable once CppParameterMayBeConstPtrOrRef
// ReSharper disable once CppParameterMayBeConst
void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs){
    if (ErrorStatusITs & FDCAN_IR_BO) {
        CLEAR_BIT(hfdcan->Instance->CCCR, FDCAN_CCCR_INIT);
    }
}
#endif
