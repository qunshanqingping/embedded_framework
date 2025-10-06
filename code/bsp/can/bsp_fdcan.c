#include "bsp_fdcan.h"
#include <string.h>
#include <stdbool.h>
#include "FreeRTOS.h"
#include "plf_log.h"
#include "memory_management.h"
#ifdef USER_CAN_FD

/* FDCAN 实例声明 */
#if defined USER_CAN1
static uint8_t fdcan1_idx; /*!< fdcan1实例索引 */
static CanInstance_s *fdcan1_instance[FDCAN_MAX_REGISTER_CNT]; /*!< fdcan1实例数组 */
static uint8_t fdcan1_tx_idx; /*!< fdcan1发送缓存索引 */
uint8_t fdcan1_tx_buf[FDCAN_MAX_REGISTER_CNT][8] = {0}; /*!< fdcan1发送缓存 */
#endif

#if defined USER_CAN2
static uint8_t fdcan2_idx; /*!< fdcan2实例索引 */
static CanInstance_s *fdcan2_instance[FDCAN_MAX_REGISTER_CNT]; /*!< fdcan2实例数组 */
static uint8_t fdcan2_tx_idx; /*!< fdcan2发送缓存索引 */
uint8_t fdcan2_tx_buf[FDCAN_MAX_REGISTER_CNT][8] = {0}; /*!< fdcan2发送缓存 */
#endif

#if defined USER_CAN3
static uint8_t fdcan3_idx; /*!< fdcan3实例索引 */
static CanInstance_s *fdcan3_instance[FDCAN_MAX_REGISTER_CNT]; /*!< fdcan3实例数组 */
static uint8_t fdcan3_tx_idx; /*!< fdcan3发送缓存索引 */
uint8_t fdcan3_tx_buf[FDCAN_MAX_REGISTER_CNT][8] = {0}; /*!< fdcan3发送缓存 */
#endif

/* fdcan初始化标志 */
static bool fdcan_init_flag = false;



FDCAN_FilterTypeDef FDCAN_FIFO0_Filter = {
    .IdType = FDCAN_STANDARD_ID,
    .FilterIndex = 0, // 过滤器编号
    .FilterType = FDCAN_FILTER_MASK, // 过滤器 Mask 模式 关乎到 ID1ID2 的配置
    .FilterConfig = FDCAN_FILTER_TO_RXFIFO0,
    .FilterID1 = 0x00000000, // 过滤器 ID1，只要 ID2 配置为 0x00000000，就不会过滤任何 ID
    .FilterID2 = 0x00000000, // 过滤器 ID2
};
FDCAN_FilterTypeDef FDCAN_FIFO1_Filter = {
    .IdType = FDCAN_STANDARD_ID,
    .FilterIndex = 1, // 过滤器编号
    .FilterType = FDCAN_FILTER_MASK, // 过滤器 Mask 模式 关乎到 ID1ID2 的配置
    .FilterConfig = FDCAN_FILTER_TO_RXFIFO1,
    .FilterID1 = 0x00000000, // 过滤器 ID1，只要 ID2 配置为 0x00000000，就不会过滤任何 ID
    .FilterID2 = 0x00000000, // 过滤器 ID2
};

/* 私有函数 ---------------------------------------------------------------------*/
/**
 * @brief 初始化 FDCAN 过滤器配置。
 *
 * 该函数根据用户定义的宏（USER_CAN1, USER_CAN2, USER_CAN3）初始化相应的 FDCAN 实例的过滤器。对于每个启用的 FDCAN 实例，它首先确定 ID 类型（标准或扩展），然后选择接收 FIFO（FIFO0 或 FIFO1）。如果两个 FIFO 都没有元素，则记录错误并返回 false。接着，配置过滤器参数，包括 ID 类型、过滤器索引、过滤器类型和过滤器配置。此外，还配置了全局过滤器以拒绝不匹配的标准 ID 和扩展 ID 以及远程帧，并设置了 FIFO 水印中断。如果在配置过程中出现任何错误，将记录错误日志并重试直到成功。
 * 注意：此函数依赖于 HAL 库提供的 FDCAN 相关 API。
 * @return 如果所有 FDCAN 实例的过滤器配置成功，返回 true；否则返回 false。
 */
static void FDCAN_Filter_Init(void){
#ifdef USER_CAN1_FIFO_0

    while (HAL_FDCAN_ConfigFilter(&hfdcan1, &FDCAN_FIFO0_Filter) != HAL_OK) {
        Log_Error("FDCAN1 Fifo0 configs filter failed");
    } // 拒绝接收匹配不成功的标准 ID 和扩展 ID, 不接受远程帧
    while (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) !=
        HAL_OK) {
        Log_Error("FDCAN1 Fifo0 configs global filter failed");
        }
    // 水印中断，接受 1 条消息触发中断
    while (HAL_FDCAN_ConfigFifoWatermark(&hfdcan1, FDCAN_CFG_RX_FIFO0, 1) != HAL_OK) {
        Log_Error("FDCAN1 Fifo0 configs fifo watermark failed");
    }
#endif
#ifdef USER_CAN1_FIFO_1
    while (HAL_FDCAN_ConfigFilter(&hfdcan1, &FDCAN_FIFO1_Filter) != HAL_OK) {
        Log_Error("FDCAN1 Fifo1 configs filter failed");
    } // 拒绝接收匹配不成功的标准 ID 和扩展 ID, 不接受远程帧
    while (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) !=
        HAL_OK) {
        Log_Error("FDCAN1 Fifo1 configs global filter failed");
        }
    // 水印中断，接受 1 条消息触发中断
    while (HAL_FDCAN_ConfigFifoWatermark(&hfdcan1, FDCAN_CFG_RX_FIFO1, 1) != HAL_OK) {
        Log_Error("FDCAN1 Fifo1 configs fifo watermark failed");
    }
#endif
#ifdef USER_CAN2_FIFO_0
    while (HAL_FDCAN_ConfigFilter(&hfdcan2, &FDCAN_FIFO0_Filter) != HAL_OK) {
        Log_Error("FDCAN2 Fifo0 configs filter failed");
    } // 拒绝接收匹配不成功的标准 ID 和扩展 ID, 不接受远程帧
    while (HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) !=
        HAL_OK) {
        Log_Error("FDCAN2 Fifo0 configs global filter failed");
        }
    // 水印中断，接受 1 条消息触发中断
    while (HAL_FDCAN_ConfigFifoWatermark(&hfdcan2, FDCAN_CFG_RX_FIFO0, 1) != HAL_OK) {
        Log_Error("FDCAN2 Fifo0 configs fifo watermark failed");
    }
#endif
#ifdef USER_CAN2_FIFO_1
    while (HAL_FDCAN_ConfigFilter(&hfdcan2, &FDCAN_FIFO1_Filter) != HAL_OK) {
        Log_Error("FDCAN2 Fifo1 configs filter failed");
    } // 拒绝接收匹配不成功的标准 ID 和扩展 ID, 不接受远程帧
    while (HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) !=
        HAL_OK) {
        Log_Error("FDCAN2 Fifo1 configs global filter failed");
        }
    // 水印中断，接受 1 条消息触发中断
    while (HAL_FDCAN_ConfigFifoWatermark(&hfdcan2, FDCAN_CFG_RX_FIFO1, 1) != HAL_OK) {
        Log_Error("FDCAN2 Fifo1 configs fifo watermark failed");
    }
#endif
#ifdef USER_CAN3_FIFO_0
    while (HAL_FDCAN_ConfigFilter(&hfdcan3, &FDCAN_FIFO0_Filter) != HAL_OK) {
        Log_Error("FDCAN3 Fifo0 configs filter failed");
    } // 拒绝接收匹配不成功的标准 ID 和扩展 ID, 不接受远程帧
    while (HAL_FDCAN_ConfigGlobalFilter(&hfdcan3, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) !=
        HAL_OK) {
        Log_Error("FDCAN3 Fifo0 configs global filter failed");
        }
    // 水印中断，接受 1 条消息触发中断
    while (HAL_FDCAN_ConfigFifoWatermark(&hfdcan3, FDCAN_CFG_RX_FIFO0, 1) != HAL_OK) {
        Log_Error("FDCAN3 Fifo0 configs fifo watermark failed");
    }
#endif
#ifdef USER_CAN3_FIFO_1
    while (HAL_FDCAN_ConfigFilter(&hfdcan3, &FDCAN_FIFO1_Filter) != HAL_OK) {
        Log_Error("FDCAN3 Fifo1 configs filter failed");
    } // 拒绝接收匹配不成功的标准 ID 和扩展 ID, 不接受远程帧
    while (HAL_FDCAN_ConfigGlobalFilter(&hfdcan3, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) !=
        HAL_OK) {
        Log_Error("FDCAN3 Fifo1 configs global filter failed");
        }
    // 水印中断，接受 1 条消息触发中断
    while (HAL_FDCAN_ConfigFifoWatermark(&hfdcan3, FDCAN_CFG_RX_FIFO1, 1) != HAL_OK) {
        Log_Error("FDCAN3 Fifo1 configs fifo watermark failed");
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
#ifdef USER_CAN1
    while (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
        Log_Error("FDCAN1 Starts Failed");
    }
#endif
#ifdef USER_CAN2
    while (HAL_FDCAN_Start(&hfdcan2) != HAL_OK) {
        Log_Error("FDCAN2 Starts Failed");
    }
#endif
#ifdef USER_CAN3
    while (HAL_FDCAN_Start(&hfdcan3) != HAL_OK) {
        Log_Error("FDCAN3 Starts Failed");
    }
#endif
#ifdef USER_CAN1_FIFO_0
    while (HAL_FDCAN_ActivateNotification(&hfdcan1,FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) {
        Log_Error("FDCAN1 Fifo0 configs interruption failed");
    }
#endif

#ifdef USER_CAN1_FIFO_1
    while (HAL_FDCAN_ActivateNotification(&hfdcan1,FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0) != HAL_OK) {
        Log_Error("FDCAN1 Fifo1 configs interruption failed");
    }
#endif


#ifdef USER_CAN2_FIFO_0
    while (HAL_FDCAN_ActivateNotification(&hfdcan2,FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) {
        Log_Error("FDCAN2 Fifo0 configs interruption failed");
    }
#endif

#ifdef USER_CAN2_FIFO_1
    while (HAL_FDCAN_ActivateNotification(&hfdcan2,FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0) != HAL_OK) {
        Log_Error("FDCAN2 Fifo1 configs interruption failed");
    }
#endif

#ifdef USER_CAN3_FIFO_0
    while (HAL_FDCAN_ActivateNotification(&hfdcan3,FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) {
        Log_Error("FDCAN3 Fifo0 configs interruption failed");
    }
#endif
#ifdef USER_CAN3_FIFO_1
    while (HAL_FDCAN_ActivateNotification(&hfdcan3,FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0) != HAL_OK) {
        Log_Error("FDCAN3 Fifo1 configs interruption failed");
    }
#endif
}


/**
 * @brief 初始化CAN模块。
 * 该函数首先尝试初始化CAN过滤器，如果成功，则继续初始化CAN服务，并记录一条通过日志。如果CAN过滤器初始化失败，则直接记录一条错误日志。
 * @todo 需要添加超时警告机制，防止初始化过程中的死循环。
 * @return 无返回值（void）。
 */
static void FDCAN_Init(void) {
    if (!fdcan_init_flag) {
        FDCAN_Filter_Init();
        FDCAN_Service_Init();
        Log_Passing("Can Init successfully");
        fdcan_init_flag = true;
    }
}

// ReSharper disable once CppNotAllPathsReturnValue
static FDCAN_HandleTypeDef* Select_FDCAN_Handle(const uint8_t can_number) {
#ifdef USER_CAN1
    if (can_number == 1) {
        return &hfdcan1;
    }
#endif
#ifdef USER_CAN2
    if (can_number == 2) {
        return &hfdcan2;
    }
#endif
#ifdef USER_CAN3
    if (can_number == 3) {
        return &hfdcan3;
    }
#endif
    return NULL;
}

/**
 * @brief 检查FDCAN注册配置的有效性。
 *
 * 该函数对传入的FDCAN初始化配置进行一系列验证，以确保其合法性和一致性。具体检查包括：
 * 1. 配置指针是否为空。
 * 2. 实例名称是否为空。
 * 3. CAN编号是否在合法范围内（1-3）。
 * 4. 发送ID和接收ID是否为0。
 * 5. 检查每个CAN实例的最大注册数量是否已达到上限。
 * 6. 检查接收ID是否与已注册实例冲突。
 * 7. 回调函数是否为空（仅记录警告，不影响注册）。
 *
 * 如果任何检查失败，函数将记录相应的错误日志并返回false；如果所有检查通过，则返回true。
 *
 * @param config 指向CanInitConfig_s结构体的指针，包含FDCAN初始化配置参数。
 * @return 如果配置有效则返回true，否则返回false。
 */
static bool FDCAN_Register_Check(const CanInitConfig_s *config) {
    if (config == NULL) {
        /* !<检查配置是否为空 */
        Log_Error("Can Init Config Is Null");
        return false;
    }
    if (config->topic_name == NULL) {
        /* !<检查实例名称是否为空 */
        Log_Error("Can Init Topic Name Is Null");
        return false;
    }
    if (config->can_number == 0 || config->can_number > 3) {
        /* !<检查CAN编号是否合法 */
        Log_Error("CanX %s Number Error", config->can_number, config->topic_name);
        return false;
    }
    if (config->tx_id == 0) {
        /* !<检查发送ID是否为0 */
        Log_Error("Can%d    %s : Tx ID Is 0x000", config->can_number, config->topic_name);
        return false;
    }
    if (config->rx_id == 0) {
        /* !<检查接收ID是否为0 */
        Log_Error("Can%d    %s : ID Is 0x000", config->can_number, config->topic_name);
        return false;
    }
#if defined USER_CAN1
    if (config->can_number == 1) {
        /* 检查是否超过CAN1最大实例数 */
        if (fdcan1_idx == FDCAN_MAX_REGISTER_CNT) {
            Log_Error("Can1    %s : Max Register Count Reached", config->topic_name);
            return false;
        } /* 检查RX ID是否冲突 */
        for (uint8_t i = 0; i < fdcan1_idx; i++) {
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
        if (fdcan2_idx == FDCAN_MAX_REGISTER_CNT) {
            Log_Error("Can2    %s : Max Register Count Reached", config->topic_name);
            return false;
        } /* 检查RX ID是否冲突 */
        for (uint8_t i = 0; i < fdcan2_idx; i++) {
            if (fdcan2_instance[i]->rx_id == config->rx_id) {
                Log_Error("Can2    %s : Rx ID 0x%03X Already Exists", config->topic_name, config->rx_id);
                return false;
            }
        }
    }
#if defined USER_CAN3
    if (config->can_number == 3) {
        /* 检查是否超过CAN3最大实例数 */
        if (fdcan3_idx == FDCAN_MAX_REGISTER_CNT) {
            Log_Error("Can3    %s : Max Register Count Reached", config->topic_name);
            return false;
        } /* 检查RX ID是否冲突 */
        for (uint8_t i = 0; i < fdcan3_idx; i++) {
            if (fdcan3_instance[i]->rx_id == config->rx_id) {
                Log_Error("Can3    %s : Rx ID 0x%03X Already Exists", config->topic_name, config->rx_id);
                return false;
            }
        }
    }
#endif
#endif
    if (config->can_module_callback == NULL) {
        /* !<检查回调函数是否为空 */
        Log_Warning("%s Can Callback Is Null", config->topic_name);
    }
    return true; /* !<配置检查通过 */
}

/**
 * @brief 获取FDCAN发送缓冲区指针。
 *
 * 该函数根据提供的CAN初始化配置，从相应的FDCAN实例中获取与指定发送ID匹配的发送缓冲区指针。如果找不到匹配的发送ID，则返回一个新的发送缓冲区指针。如果配置无效或找不到对应的FDCAN实例，则记录错误日志并返回NULL。
 *
 * @param config 指向包含CAN初始化配置信息（如CAN编号、发送ID等）的CanInitConfig_s结构的指针。
 *
 * @return 成功时返回指向发送缓冲区的指针；如果配置无效或找不到对应的FDCAN实例，则返回NULL。
 */
static uint8_t* FDCAN_Get_Tx_Buff(const CanInitConfig_s *config) {
#if defined USER_CAN1
    if (config->can_number == 1) {
        for (uint8_t i = 0; i < fdcan1_idx; i++) {
            if (fdcan1_instance[i]->tx_id == config->tx_id) {
                return fdcan1_instance[i]->tx_buff_ptr;
            }
        }
        return fdcan1_tx_buf[fdcan1_tx_idx++];
    }
#endif
#if defined USER_CAN2
    if (config->can_number == 2) {
        for (uint8_t i = 0; i < fdcan2_idx; i++) {
            if (fdcan2_instance[i]->tx_id == config->tx_id) {
                return fdcan2_instance[i]->tx_buff_ptr;
            }
        }
        return fdcan2_tx_buf[fdcan2_tx_idx++];
    }
#endif
#if defined USER_CAN3
    if (config->can_number == 3) {
        for (uint8_t i = 0; i < fdcan3_idx; i++) {
            if (fdcan3_instance[i]->tx_id == config->tx_id) {
                return fdcan3_instance[i]->tx_buff_ptr;
            }
        }
        return fdcan3_tx_buf[fdcan3_tx_idx++];
    }
#endif
    Log_Error("Can Get Tx Buff Failed");
    return NULL;
}

/**
 * @brief 将CAN实例注册到对应的FDCAN实例数组中。
 *
 * 该函数根据CAN实例的句柄，将其添加到相应的FDCAN实例数组中（fdcan1_instance, fdcan2_instance, fdcan3_instance）。如果CAN实例的句柄与任何已定义的FDCAN实例匹配，则将其添加到对应的数组，并增加相应的索引计数器。最后，函数通过日志记录注册的CAN实例的信息，包括其接收ID和发送ID。
 *
 * @param instance 指向要注册的CanInstance_s结构体的指针。
 */
static void FDCAN_Register_to_fdcan_x_instance(CanInstance_s *instance) {
#if defined USER_CAN1
    if (instance->can_handle == &hfdcan1) {
        fdcan1_instance[fdcan1_idx++] = instance;
    }
#endif
#if defined USER_CAN2
    if (instance->can_handle == &hfdcan2) {
        fdcan2_instance[fdcan2_idx++] = instance;
    }
#endif
#if defined USER_CAN3
    if (instance->can_handle == &hfdcan3) {
        fdcan3_instance[fdcan3_idx++] = instance;
    }
#endif
    Log_Passing("%s RxID : 0x%03X TxID : 0x%03X", instance->topic_name, instance->rx_id, instance->tx_id);
}
/**
 * @brief FDCAN接收FIFO中断的回调函数。
 * 该函数处理接收FIFO中的消息。它检索消息，并在消息标识符与已注册的CAN实例之一匹配时调用相应的回调函数。
 *
 * @param FDCAN_RxFIFOxFrame 指向包含接收到的FDCAN消息的FDCAN_RxFrame_TypeDef结构的指针。
 * @param idx 表示fdcan_instance数组中CanInstance_s元素的数量的索引。
 * @param fdcan_instance CanInstance_s结构数组，每个结构包含特定CAN实例的配置和回调。
 */
static void FDCAN_RxFifoCallback(const FDCAN_RxFrame_TypeDef *FDCAN_RxFIFOxFrame, const uint8_t idx, CanInstance_s **fdcan_instance) {
    if (idx == 0)
        return;
    // ReSharper disable once CppDFAUnreachableCode
    for (uint8_t i = 0; i < idx; i++) {
        if ( FDCAN_RxFIFOxFrame->Header.Identifier == fdcan_instance[i]->rx_id) {
            if (fdcan_instance[i]->can_module_callback != NULL)
            {
                fdcan_instance[i]->rx_len = FDCAN_RxFIFOxFrame->Header.DataLength;
                memcpy(fdcan_instance[i]->rx_buff, FDCAN_RxFIFOxFrame->rx_buff,fdcan_instance[i]->rx_len);
                fdcan_instance[i]->can_module_callback(fdcan_instance[i]);
            }
            break;
        }
    }
}

/* 公共函数 ------------------------------------------------------------------*/

/**
 * @brief 注册一个新的CAN实例。
 *
 * 根据提供的配置信息创建并初始化一个新的CAN实例。如果配置信息为空或内存分配失败，则函数将返回NULL。成功注册后，新的CAN实例将被添加到相应的FDCAN实例列表中，并返回指向新实例的指针。
 *
 * @param config 指向包含CAN初始化配置信息（如句柄、发送ID等）的CanInitConfig_s结构的指针。
 *
 * @return 如果成功注册则返回指向新创建的CanInstance_s结构的指针；如果配置无效或内存分配失败则返回NULL。
 */
CanInstance_s *Can_Register(const CanInitConfig_s *config) {
    FDCAN_Init();
    if (!FDCAN_Register_Check(config)) {
        return NULL; // 配置检查失败
    }

    CanInstance_s *instance = user_malloc(sizeof(CanInstance_s)); // 分配空间
    if (instance == NULL) {
        Log_Error("%s CanInstance Malloc Failed", config->topic_name);
        return NULL; // 内存分配失败
    }
    memset(instance, 0, sizeof(CanInstance_s));

    instance->topic_name = config->topic_name;
    instance->can_handle = Select_FDCAN_Handle(config->can_number);
    instance->tx_id = config->tx_id;
    instance->rx_id = config->rx_id;
    instance->parent_ptr = FDCAN_Get_Tx_Buff(config);
    instance->can_module_callback = config->can_module_callback;

    instance->tx_conf.Identifier = config->tx_id;
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
 * 该函数检查CAN发送FIFO是否有空闲位置，如果有，则将外部提供的发送缓冲区中的消息添加到发送队列中并返回true。如果发送FIFO满，则函数会等待一段时间（最多100次检查），如果仍然没有空闲位置，则记录错误日志并返回false。
 * @param instance 指向要使用的CanInstance_s结构体的指针。
 * @param tx_buff 指向包含要发送的CAN消息数据的缓冲区的指针。
 * @return 如果消息成功添加到发送队列，返回true；如果发送FIFO满或发生错误，返回false。
 */
bool Can_Transmit_External_Tx_Buff( const CanInstance_s *instance, const uint8_t *tx_buff) {
    uint8_t can_tx_cnt=0;
    while (HAL_FDCAN_GetTxFifoFreeLevel(instance->can_handle) == 0) {
        can_tx_cnt++;
        if(can_tx_cnt>=100) {
            Log_Error("Can Transmit Timeout");
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
 * 该函数将实例内部的发送缓冲区数据通过给定的CAN实例发送出去。如果发送成功，返回true；否则返回false
 * @param instance 指向已注册的CanInstance_s结构体的指针，表示要使用的CAN实例
 * @return 如果数据发送成功则返回true，否则返回false
 */
bool Can_Transmit( const CanInstance_s *instance) {
    uint8_t can_tx_cnt=0;
    while (HAL_FDCAN_GetTxFifoFreeLevel(instance->can_handle) == 0) {
        can_tx_cnt++;
        if(can_tx_cnt>=100) {
            Log_Error("Can Transmit Timeout");
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
    FDCAN_RxFrame_TypeDef FDCAN_RxFIFO0Frame;
    if (RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) {
        HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &FDCAN_RxFIFO0Frame.Header, FDCAN_RxFIFO0Frame.rx_buff);

#ifdef USER_CAN1_FIFO_0
        if (hfdcan == &hfdcan1){
            FDCAN_RxFifoCallback(&FDCAN_RxFIFO0Frame,fdcan1_idx,fdcan1_instance);
        }
#endif
#ifdef USER_CAN2_FIFO_0
        if (hfdcan == &hfdcan2){
            FDCAN_RxFifoCallback(&FDCAN_RxFIFO0Frame,fdcan2_idx,fdcan2_instance);
        }
#endif
#ifdef USER_CAN3_FIFO_0
        if (hfdcan == &hfdcan3){
            FDCAN_RxFifoCallback(&FDCAN_RxFIFO0Frame,fdcan3_idx,fdcan3_instance);
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
    FDCAN_RxFrame_TypeDef FDCAN_RxFIFO1Frame;
    if (RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) {
        HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &FDCAN_RxFIFO1Frame.Header, FDCAN_RxFIFO1Frame.rx_buff);
#ifdef USER_CAN1_FIFO_1
        if (hfdcan == &hfdcan1){
            FDCAN_RxFifoCallback(&FDCAN_RxFIFO1Frame,fdcan1_idx, fdcan1_instance);
        }
#endif
#ifdef USER_CAN2_FIFO_1
        if (hfdcan == &hfdcan2){
            FDCAN_RxFifoCallback(&FDCAN_RxFIFO1Frame,fdcan2_idx,fdcan2_instance);
        }
#endif
#ifdef USER_CAN3_FIFO_1
        if (hfdcan == &hfdcan3){
            FDCAN_RxFifoCallback(&FDCAN_RxFIFO1Frame,fdcan3_idx,fdcan3_instance);
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
