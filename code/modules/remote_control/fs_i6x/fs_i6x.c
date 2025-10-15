// ReSharper disable CppParameterMayBeConst
#include "fs_i6x.h"
#include "basic_math.h"
#include "bsp_usart.h"
#include "sbus.h"
#include "memory_management.h"
#include "plf_log.h"
#include "watch_dog.h"
#include <string.h>
/* 接收缓冲区 */
__attribute((section(".axid1"), aligned(32))) uint8_t sbus_rx_first_buff[SBUS_FRAME_SIZE];
__attribute((section(".axid1"), aligned(32))) uint8_t sbus_rx_second_buff[SBUS_FRAME_SIZE];

/**
 * @brief 解析两位开关
 * @param ch 通道值
 * @param mid_val 中值
 * @return 开关状态 0,1
 */
// ReSharper disable once CppDeclaratorNeverUsed
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
 * @return 开关状态 0,1,2
 */
// ReSharper disable once CppDeclaratorNeverUsed
inline static uint8_t remoter_3stage_switch_parse(int16_t ch, int16_t mid_val0){
    if (ch < mid_val0){
        return 0;
    }
    if (ch == mid_val0){
        return 1;
    }
    return 2;
}
/**
 * @brief 解析Sbus数据
 * @param buffer Sbus数据缓冲区指针
 * @param data I6x数据结构体指针
 */
static void Sbus_Decode(uint8_t *buffer, I6xData_s* data){
    Sbus_Frame_Parse(&data->rc, buffer);
    data->rx_freq.cnt_1s++;
    data->joy.right_x = int16_deadline((int16_t)(data->rc.ch[0] - RC_CH_VALUE_OFFSET),
        -REMOTER_DEADLINE,REMOTER_DEADLINE);
    data->joy.right_y = int16_deadline((int16_t)(data->rc.ch[1] - RC_CH_VALUE_OFFSET),
        -REMOTER_DEADLINE,REMOTER_DEADLINE);
    data->joy.left_y = int16_deadline((int16_t)(data->rc.ch[2] - RC_CH_VALUE_OFFSET),
        -REMOTER_DEADLINE,REMOTER_DEADLINE);
    data->joy.left_x = int16_deadline((int16_t)(data->rc.ch[3] - RC_CH_VALUE_OFFSET),
        -REMOTER_DEADLINE,REMOTER_DEADLINE);
    data->var.a = (int16_t)(data->rc.ch[4] - RC_CH_VALUE_OFFSET);
    data->var.b = (int16_t)(data->rc.ch[5] - RC_CH_VALUE_OFFSET);
    data->sw.a = remoter_3stage_switch_parse(data->rc.ch[6],RC_CH_VALUE_OFFSET);
    data->sw.b = remoter_3stage_switch_parse(data->rc.ch[7],RC_CH_VALUE_OFFSET);
    data->sw.c = remoter_3stage_switch_parse(data->rc.ch[8],RC_CH_VALUE_OFFSET);
    data->sw.d = remoter_3stage_switch_parse(data->rc.ch[9],RC_CH_VALUE_OFFSET);

    Ramp_Update(&data->rc_data.x_ramp, data->joy.right_x * RC_JOY_TO_VAL_COFE);
    Ramp_Update(&data->rc_data.y_ramp, data->joy.right_y * RC_JOY_TO_VAL_COFE);
    Ramp_Update(&data->rc_data.pitch_ramp, data->joy.left_y * RC_JOY_TO_PI_COFE);
    Ramp_Update(&data->rc_data.yaw_ramp, data->joy.left_x * RC_JOY_TO_PI_COFE);
    Ramp_Update(&data->rc_data.fine_pitch, data->var.a * RC_JOY_TO_PI_COFE);
    Ramp_Update(&data->rc_data.fine_yaw, data->var.b * RC_JOY_TO_PI_COFE);
}

/**
 * @brief Sbus接收回调函数
 * @param usart_instance USART实例指针
 * @param Size 接收数据长度
 */
// ReSharper disable once CppParameterMayBeConstPtrOrRef
static void Sbus_Callback(UsartInstance_s *usart_instance, uint16_t Size){
    if (usart_instance == NULL || usart_instance->parent_ptr == NULL || Size > SBUS_FRAME_SIZE)
    {
        return;
    }
    I6xInstance_s *i6x_instance = usart_instance->parent_ptr;
    if ((((DMA_Stream_TypeDef*)usart_instance->huart_handle->hdmarx->Instance)->CR & DMA_SxCR_CT) == RESET)
    {
        __HAL_DMA_DISABLE(usart_instance->huart_handle->hdmarx);
        ((DMA_Stream_TypeDef*)usart_instance->huart_handle->hdmarx->Instance)->CR |= DMA_SxCR_CT;
        __HAL_DMA_SET_COUNTER(usart_instance->huart_handle->hdmarx,SBUS_FRAME_SIZE * 2);
        if(Size == SBUS_FRAME_SIZE)
        {
            Sbus_Decode(usart_instance->first_rx_buf,&i6x_instance->data);
        }
    }
    else
    {
        __HAL_DMA_DISABLE(usart_instance->huart_handle->hdmarx);
        ((DMA_Stream_TypeDef  *)usart_instance->huart_handle->hdmarx->Instance)->CR &= ~(DMA_SxCR_CT);
        __HAL_DMA_SET_COUNTER(usart_instance->huart_handle->hdmarx,SBUS_FRAME_SIZE * 2);
        if(Size == SBUS_FRAME_SIZE)
        {
            Sbus_Decode(usart_instance->second_rx_buf,&i6x_instance->data);
            // sbus_frame_parse(usart_instance->second_rx_buf,&i6x_instance->remote_control);
        }
    }
    __HAL_DMA_ENABLE(usart_instance->huart_handle->hdmarx);
}

void Monitor_I6x(WatchDogInstance_s * WatchDogInstance_s){
    I6xInstance_s * instance = (I6xInstance_s *)WatchDogInstance_s->parent_ptr;
    instance->data.rx_freq.frequency = instance->data.rx_freq.cnt_1s;
    instance->data.rx_freq.cnt_1s = 0;
}

/**
 * @brief 注册FS-I6X遥控器实例
 * @param huart UART句柄指针
 * @return FS-I6X遥控器实例指针
 */
I6xInstance_s* I6x_Register(UART_HandleTypeDef *huart){
    UsartInitConfig_s* usart_config = user_malloc(sizeof(UsartInitConfig_s));
    WatchDogInitConfig_s* watch_dog_config = user_malloc(sizeof(WatchDogInitConfig_s));
    I6xInstance_s* i6x_instance = user_malloc(sizeof(I6xInstance_s));
    if(usart_config == NULL || i6x_instance == NULL || watch_dog_config == NULL){
        Log_Error("FS-I6X SbusInstance UartConfig or i6x_instance or watchdog Malloc Failed");
        user_free(usart_config);
        user_free(i6x_instance);
        user_free(watch_dog_config);
        return NULL;
    }
    memset(usart_config, 0, sizeof(UsartInitConfig_s));
    memset(i6x_instance, 0, sizeof(I6xInstance_s));
    memset(watch_dog_config, 0, sizeof(WatchDogInitConfig_s));
    usart_config->topic_name = "FS-I6X";
    usart_config->huart_handle = huart;
    usart_config->mode = DMA_MODE;
    usart_config->direction = RX_MODE;
    usart_config->rx_len = SBUS_FRAME_SIZE;
    usart_config->first_rx_buf = sbus_rx_first_buff;
    usart_config->second_rx_buf = sbus_rx_second_buff;
    usart_config->usart_module_callback = NULL;
    usart_config->parent_ptr = i6x_instance;
    usart_config->usart_module_callback = Sbus_Callback;
    i6x_instance->usart_instance = Usart_Register(usart_config);
    user_free(usart_config);
    Usart_RxDMA_DoubleBuffer_Init(i6x_instance->usart_instance);

    watch_dog_config->topic_name = "FS-I6X";
    watch_dog_config->parent_ptr = i6x_instance;
    watch_dog_config->watchdog_callback = Monitor_I6x;
    i6x_instance->watchdog_instance = WatchDog_Register(watch_dog_config);
    user_free(watch_dog_config);

    Ramp_init(&i6x_instance->data.rc_data.x_ramp,7 );
    Ramp_init(&i6x_instance->data.rc_data.y_ramp,7 );
    Ramp_init(&i6x_instance->data.rc_data.pitch_ramp,7 );
    Ramp_init(&i6x_instance->data.rc_data.yaw_ramp,7 );
    Ramp_init(&i6x_instance->data.rc_data.fine_pitch,7 );
    Ramp_init(&i6x_instance->data.rc_data.fine_yaw,7 );

    return i6x_instance;
}