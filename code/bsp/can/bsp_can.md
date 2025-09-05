# bsp_can

## 使用
### 1. 配置
   在 user_config.h 中关于配置 CAN 相关参数，按需要开启
   ```c
    /* CAN 初始化配置选项 */
    
    /* 选择 CAN 类型 */
    #define USER_CAN_STANDARD
    
    /* 选择 can1 or can2 */
    #define USER_CAN1
    #define USER_CAN2
    
    /* 选择 can1 fifo 0 or 1 */
    #define USER_CAN1_FIFO_0
    #define USER_CAN1_FIFO_1
    
    /* 选择 can2 fifo 0 or 1 */
    #define USER_CAN2_FIFO_0
    #define USER_CAN2_FIFO_1
    
    /* 选择过滤器模式 */
    #define USER_CAN_FILTER_MASK_MODE
    #define USER_CAN_FILTER_LIST_MODE
   ```
### 示例
#### 初始化
    ```c
     #include "bsp_can.h"
     void test_decode(CanInstance_s *instance)
    {
    Log_Passing("%x %x %x %x %x %x %x %x",
    instance->rx_buff[0],instance->rx_buff[1],instance->rx_buff[2],instance->rx_buff[3],instance->rx_buff[4],instance->rx_buff[5],instance->rx_buff[6],instance->rx_buff[7]);
    }
    CanInstance_s *test;
    CanInitConfig_s test_config = {
    .topic_name = "test",
    .can_number = 1,
    .tx_id = 0x200,
    .rx_id = 0x001,
    .can_module_callback = test_decode,
    .id = NULL
    };
    bool test_status;
    uint8_t tx_buf[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
         }
    ```
#### 使用
    ```c
    void Start_Cmd_Task(void const * argument)
    {
    /* USER CODE BEGIN CAN_Task */
    test_init();
    /* Infinite loop */
    for(;;)
    {
    test_status = Can_Transmit_External_Tx_Buff(test, tx_buf);
    // test_status = Can_Transmit(test);
    osDelay(2000);
    }
    /* USER CODE END CAN_Task */
    }
```