/*
 * @file dbus.c
 * @brief DJI DT7/DR16 Remote Control Protocol Decoder
 * @author Adonis Jin
 * @date 2021/05/05
 * @version 1.0.0
 * @note This module handles the decoding of data received from DJI DT7/DR16 remote controller
 */

#include "dbus.h"
#include "basic_math.h"
#include "string.h"
/*!
 * @brief Decode DR16 receiver data buffer into remote control structure
 * @param[in] dbus_buf Raw data buffer received from DR16 remote controller
 * @param[out] remote_ctrl_data Decoded remote control data structure
 * @return None
 * @note This function parses the raw byte stream from DR16 and extracts all control channels
 * @todo Keyboard & mouse status parsing needs to be implemented
 */
void Remote_Ctrl_Dbus_Decode(volatile const uint8_t *dbus_buf, RemoteCtrlInfo_s *remote_ctrl_data)
{
    /* Null pointer check to prevent segmentation fault */
    if (dbus_buf == NULL || remote_ctrl_data == NULL)
    {
        return;
    }

    /* Decode RC channel data (channels 0-3) */
    remote_ctrl_data->rc.ch[0] = (int16_t)((dbus_buf[0] | (dbus_buf[1] << 8))& 0x07ff);  /* Channel 0 */
    remote_ctrl_data->rc.ch[1] = (int16_t)(((dbus_buf[1] >> 3)| (dbus_buf[2] << 5))& 0x07ff);  /* Channel 1 */
    remote_ctrl_data->rc.ch[2] = (int16_t)(((dbus_buf[2] >> 6)| (dbus_buf[3] << 2)| (dbus_buf[4] << 10))& 0x07ff);  /* Channel 2 */
    remote_ctrl_data->rc.ch[3] = (int16_t)(((dbus_buf[4] >> 1)| (dbus_buf[5] << 7))& 0x07ff);  /* Channel 3 */
    remote_ctrl_data->rc.wheel = (int16_t)((dbus_buf[16] | (dbus_buf[17] << 8))& 0x07ff);  /* Wheel channel */

    /* Decode switch lever positions (left and right switches) */
    remote_ctrl_data->rc.s[0] = ((dbus_buf[5] >> 4)& 0x0003);       //!< Left switch position
    remote_ctrl_data->rc.s[1] = ((dbus_buf[5] >> 4)& 0x000C) >> 2;  //!< Right switch position

    /* Decode mouse axis data (X, Y coordinates and wheel) */
    remote_ctrl_data->mouse.x = (int16_t)(dbus_buf[6] | (dbus_buf[7] << 8));      //!< Mouse X-axis
    remote_ctrl_data->mouse.y = (int16_t)(dbus_buf[8] | (dbus_buf[9] << 8));      //!< Mouse Y-axis
    remote_ctrl_data->mouse.wheel = (int16_t)(dbus_buf[10] | (dbus_buf[11] << 8)); //!< Mouse wheel

    /* Decode mouse button states */
    remote_ctrl_data->mouse.left_button = dbus_buf[12];   //!< Left mouse button state
    remote_ctrl_data->mouse.right_button = dbus_buf[13];  //!< Right mouse button state

    /* Decode keyboard data */
    remote_ctrl_data->key.v = (int16_t)(dbus_buf[14] | (dbus_buf[15] << 8));  //!< Keyboard state

    /* Apply zero-point calibration to channels */
    remote_ctrl_data->rc.ch[0] -= DT7_CH_MEDIAN;
    remote_ctrl_data->rc.ch[1] -= DT7_CH_MEDIAN;
    remote_ctrl_data->rc.ch[2] -= DT7_CH_MEDIAN;
    remote_ctrl_data->rc.ch[3] -= DT7_CH_MEDIAN;
    remote_ctrl_data->rc.wheel -= DT7_CH_MEDIAN;
}

/*!
 * @brief DBUS module UART receive complete callback function
 * @param[in] parent_pointer Pointer to DbusInstance_s structure (passed as user data)
 * @param[in] size Size of received data in bytes
 * @return None
 * @note UART driver calls this function when DMA reception is complete
 * @note Implements double buffer switching for continuous data reception
 */
void Dbus_RxCallback(void* parent_pointer, uint16_t size)
{
    DbusInstance_s *dbus_instance = (DbusInstance_s *)parent_pointer;
    const UART_HandleTypeDef *uart_handle_type_def = dbus_instance->uart_instance->uart_handle;

    /* Check which DMA buffer is currently in use */
    /* Current memory buffer used is Memory 0 */
    if(((((DMA_Stream_TypeDef  *)uart_handle_type_def->hdmarx->Instance)->CR) & DMA_SxCR_CT )== RESET)
    {
        /* Disable DMA to prevent race conditions during buffer switching */
        __HAL_DMA_DISABLE(uart_handle_type_def->hdmarx);

        /* Switch from Memory 0 to Memory 1 by setting CT bit */
        ((DMA_Stream_TypeDef  *)uart_handle_type_def->hdmarx->Instance)->CR |= DMA_SxCR_CT;

        /* Reset DMA reception counter for next transfer */
        __HAL_DMA_SET_COUNTER(uart_handle_type_def->hdmarx, dbus_instance->uart_instance->rx_len * 2);

        /* Validate received data size before processing */
        if(size == dbus_instance->uart_instance->rx_len)
        {
            /* Decode data from Memory 0 buffer */
            Remote_Ctrl_Dbus_Decode(dbus_instance->uart_instance->rx_first_buff, &dbus_instance->remote_ctrl_data);
        }
    }
    /* Current memory buffer used is Memory 1 */
    else
    {
        /* Disable DMA to prevent race conditions during buffer switching */
        __HAL_DMA_DISABLE(uart_handle_type_def->hdmarx);

        /* Switch from Memory 1 to Memory 0 by clearing CT bit */
        ((DMA_Stream_TypeDef  *)uart_handle_type_def->hdmarx->Instance)->CR &= ~(DMA_SxCR_CT);

        /* Reset DMA reception counter for next transfer */
        __HAL_DMA_SET_COUNTER(uart_handle_type_def->hdmarx, dbus_instance->uart_instance->rx_len * 2);

        /* Validate received data size before processing */
        if(size == dbus_instance->uart_instance->rx_len)
        {
            /* Decode data from Memory 1 buffer */
            Remote_Ctrl_Dbus_Decode(dbus_instance->uart_instance->rx_second_buff, &dbus_instance->remote_ctrl_data);
        }
    }

}

/*!
 * @brief Register and initialize a new DBUS instance
 * @param[in] config Pointer to DBUS configuration structure
 * @return Pointer to create DbusInstance_s structure, or NULL if failed
 * @note Allocates memory for new instance and registers with UART driver
 */
DbusInstance_s *Dbus_Register(DbusConfig_s *config){
    /* Validate input parameter */
    if (config == NULL){
        return NULL;
    }

    /* Allocate memory for new DBUS instance */
    DbusInstance_s *instance = (DbusInstance_s *)user_malloc(sizeof(DbusInstance_s));
    if (instance == NULL){
        return NULL;
    }

    /* Initialize allocated memory to zero */
    memset(instance, 0, sizeof(DbusInstance_s));

    /* Set instance pointer as user data for UART callback */
    config->uart_config.parent_pointer = instance;
    config->uart_config.uart_module_callback = Dbus_RxCallback; // Set the callback function for UART reception
    /* Register with UART driver */
    instance->uart_instance = Uart_Register(&config->uart_config);

    if (instance->uart_instance == NULL){
        /* Clean up allocated memory if UART registration fails */
        user_free(instance);
        return NULL;
    }
    return instance;
}
