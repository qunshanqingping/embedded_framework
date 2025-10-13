#include "old_sbus.h"

#include "plf_log.h"
#include "memory_management.h"
#include <string.h>
#include "bsp_usart.h"

__attribute((section(".axid1"), aligned(32))) uint8_t sbus_rx_buf[2][size_temp] = {0};
static uint8_t remoter_3stage_switch_parse(const int16_t ch)
{
    if (ch < SBUS_SWIRCH_MIDLE_VAL0)
    {
        return 0;
    }
    if (ch == SBUS_SWIRCH_MIDLE_VAL0)
    {
        return 1;
    }
    return 2;
}


static void sbus_frame_parse(const uint8_t *rx_buff, Sbus_Data_s *sbus_data) {
    if (rx_buff[0] != 0x0F || rx_buff[24] != 0x00) {
        return;
    }
    if (rx_buff[23] == 0x0c)
        sbus_data->online = 0;
    else
        sbus_data->online = 1;
    sbus_data->rc.ch[0] = (rx_buff[1] | rx_buff[2] << 8) & 0x07FF;
    sbus_data->rc.ch[1] = (rx_buff[2] >> 3 | rx_buff[3] << 5) & 0x07FF;
    sbus_data->rc.ch[2] = (rx_buff[3] >> 6 | rx_buff[4] << 2 | rx_buff[5] << 10) & 0x07FF;
    sbus_data->rc.ch[3] = (rx_buff[5] >> 1 | rx_buff[6] << 7) & 0x07FF;
    sbus_data->rc.ch[4] = (rx_buff[6] >> 4 | rx_buff[7] << 4) & 0x07FF;
    sbus_data->rc.ch[5] = (rx_buff[7] >> 7 | rx_buff[8] << 1 | rx_buff[9] << 9) & 0x07FF;
    sbus_data->rc.ch[6] = (rx_buff[9] >> 2 | rx_buff[10] << 6) & 0x07FF;
    sbus_data->rc.ch[7] = (rx_buff[10] >> 5 | rx_buff[11] << 3) & 0x07FF;
    sbus_data->rc.ch[8] = (rx_buff[12] | rx_buff[13] << 8) & 0x07FF;
    sbus_data->rc.ch[9] = (rx_buff[13] >> 3 | rx_buff[14] << 5) & 0x07FF;

    sbus_data->joy.right_x = sbus_data->rc.ch[0]-0X0400;
    sbus_data->joy.right_y = sbus_data->rc.ch[1]-0X0400;
    sbus_data->joy.left_y = sbus_data->rc.ch[2]-0X0400;
    sbus_data->joy.left_x = sbus_data->rc.ch[3]-0X0400;

    sbus_data->var.a = sbus_data->rc.ch[4] - 0X0400;
    sbus_data->var.b = sbus_data->rc.ch[5] - 0X0400;

    sbus_data->sw.a = remoter_3stage_switch_parse(sbus_data->rc.ch[6]);
    sbus_data->sw.b = remoter_3stage_switch_parse(sbus_data->rc.ch[7]);
    sbus_data->sw.c = remoter_3stage_switch_parse(sbus_data->rc.ch[8]);
    sbus_data->sw.d = remoter_3stage_switch_parse(sbus_data->rc.ch[9]);
}

static void Sbus_Callback(UsartInstance_s *usart_instance, uint16_t Size){
    if (usart_instance == NULL || usart_instance->parent_ptr == NULL || Size != SBUS_SIZE)
    {
        return;
    }
    I6xInstance_s *i6x_instance = usart_instance->parent_ptr;
    if ((((DMA_Stream_TypeDef*)usart_instance->huart_handle->hdmarx->Instance)->CR & DMA_SxCR_CT) == RESET)
    {
        __HAL_DMA_DISABLE(usart_instance->huart_handle->hdmarx);
        ((DMA_Stream_TypeDef*)usart_instance->huart_handle->hdmarx->Instance)->CR |= DMA_SxCR_CT;
        __HAL_DMA_SET_COUNTER(usart_instance->huart_handle->hdmarx,SBUS_SIZE * 2);
        if(Size == SBUS_SIZE)
        {
            // sbus_frame_parse(usart_instance->first_rx_buf,&i6x_instance->remote_control);
        }
    }
    else
    {
            __HAL_DMA_DISABLE(usart_instance->huart_handle->hdmarx);
            ((DMA_Stream_TypeDef  *)usart_instance->huart_handle->hdmarx->Instance)->CR &= ~(DMA_SxCR_CT);
            __HAL_DMA_SET_COUNTER(usart_instance->huart_handle->hdmarx,SBUS_SIZE * 2);
            if(Size == SBUS_SIZE)
            {
                // sbus_frame_parse(usart_instance->second_rx_buf,&i6x_instance->remote_control);
            }
        }
    __HAL_DMA_ENABLE(usart_instance->huart_handle->hdmarx);
    }

    // if(((((DMA_Stream_TypeDef  *)usart_instance->huart_handle->hdmarx->Instance)->CR) & DMA_SxCR_CT ) == RESET)
    // {
    //     __HAL_DMA_DISABLE(usart_instance->huart_handle->hdmarx);
    //
    //     ((DMA_Stream_TypeDef  *)usart_instance->huart_handle->hdmarx->Instance)->CR |= DMA_SxCR_CT;
    //
    //     __HAL_DMA_SET_COUNTER(usart_instance->huart_handle->hdmarx,SBUS_RX_BUF_NUM);
    //
    //     if(Size == RC_FRAME_LENGTH)
    //     {
    //         SBUS_TO_RC(SBUS_MultiRx_Buf[0],&remote_ctrl);
    //     }
    //
    // }else{
    //     __HAL_DMA_DISABLE(usart_instance->huart_handle->hdmarx);
    //
    //     ((DMA_Stream_TypeDef  *)usart_instance->huart_handle->hdmarx->Instance)->CR &= ~(DMA_SxCR_CT);
    //
    //     __HAL_DMA_SET_COUNTER(usart_instance->huart_handle->hdmarx,SBUS_RX_BUF_NUM);
    //
    //     if(Size == RC_FRAME_LENGTH)
    //     {
    //         SBUS_TO_RC(SBUS_MultiRx_Buf[1],&remote_ctrl);
    //     }
    // }
    // __HAL_DMA_ENABLE(usart_instance->huart_handle->hdmarx);


I6xInstance_s* Sbus_Register(UART_HandleTypeDef *huart){
    UsartInitConfig_s* usart_config = user_malloc(sizeof(UsartInitConfig_s));
    I6xInstance_s* i6x_instance = user_malloc(sizeof(I6xInstance_s));
    if(usart_config == NULL || i6x_instance == NULL){
        Log_Error("FS-I6X SbusInstance UartConfig or i6x_instance Malloc Failed");
        user_free(usart_config);
        user_free(i6x_instance);
        return NULL;
    }
    memset(usart_config, 0, sizeof(UsartInitConfig_s));
    memset(i6x_instance, 0, sizeof(I6xInstance_s));
    usart_config->topic_name = "FS-I6X";
    usart_config->huart_handle = huart;
    usart_config->mode = DMA_MODE;
    usart_config->direction = RX_MODE;
    usart_config->rx_len = size_temp;
    usart_config->first_rx_buf = sbus_rx_buf[0];
    usart_config->second_rx_buf = sbus_rx_buf[1];
    usart_config->usart_module_callback = NULL;
    usart_config->parent_ptr = i6x_instance;
    usart_config->usart_module_callback = Sbus_Callback;
    i6x_instance->usart_instance = Usart_Register(usart_config);

    Usart_RxDMA_DoubleBuffer_Init(i6x_instance->usart_instance);
    return i6x_instance;
}