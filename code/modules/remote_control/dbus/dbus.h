/*
 * @file dbus.h
 * @brief DJI DT7/DR16 Remote Control Protocol Decoder
 * @author Adonis Jin
 * @date 2021/05/05
 * @version 1.0.0
 * @note This module handles the decoding of data received from DJI DT7/DR16 remote controller
 */
#ifndef DBUS_H
#define DBUS_H

// Standard C library headers
#include <stdint.h>
#include <stdbool.h>

// Project specific headers
#include "bsp_usart.h"
#include "module_typedef.h"

// Constants
#define DT7_CH_MEDIAN 1024U  //!< Median value for DT7 remote controller channels

/**
 * @brief Remote control information structure
 * @details Contains all decoded data from the DJI DT7/DR16 remote controller
 * including RC channels, mouse data, keyboard state and connection status
 */
typedef struct
{
    /**
     * @brief RC receiver data
     * @details Contains stick channels and switch positions
     */
    struct
    {
        int16_t ch[4];  //!< RC channels 0-3 values (typically -660 to +660)
        uint8_t s[2];   //!< Switch positions (0: left switch, 1: right switch)
        int16_t wheel;  //!< Wheel/scroll value
    } rc;

    /**
     * @brief Mouse data
     * @details Contains mouse movement and button states
     */
    struct
    {
        int16_t x;              //!< Mouse X-axis movement
        int16_t y;              //!< Mouse Y-axis movement
        int16_t wheel;          //!< Mouse wheel movement
        uint8_t left_button;    //!< Left mouse button state (0: released, 1: pressed)
        uint8_t right_button;   //!< Right mouse button state (0: released, 1: pressed)
    } mouse;

    /**
     * @brief Keyboard data
     * @details Union structure for accessing keyboard state as a single value or individual keys
     */
    union
    {
        uint16_t v;  //!< Combined keyboard value
        struct
        {
            uint16_t W:1;     //!< W key state
            uint16_t S:1;     //!< S key state
            uint16_t A:1;     //!< A key state
            uint16_t D:1;     //!< D key state
            uint16_t SHIFT:1; //!< SHIFT key state
            uint16_t CTRL:1;  //!< CTRL key state
            uint16_t Q:1;     //!< Q key state
            uint16_t E:1;     //!< E key state
            uint16_t R:1;     //!< R key state
            uint16_t F:1;     //!< F key state
            uint16_t G:1;     //!< G key state
            uint16_t Z:1;     //!< Z key state
            uint16_t X:1;     //!< X key state
            uint16_t C:1;     //!< C key state
            uint16_t V:1;     //!< V key state
            uint16_t B:1;     //!< B key state
        } set;  //!< Individual key states
    } key;

    bool rc_lost;  //!< Remote control connection lost flag
} RemoteCtrlInfo_s;

/**
 * @brief DBUS module configuration structure
 * @details Contains configuration parameters for initializing the DBUS module
 */
typedef struct
{
    UartConfig_s uart_config;  //!< UART configuration for DBUS communication
} DbusConfig_s;

/**
 * @brief DBUS module instance structure
 * @details Contains runtime data for a DBUS module instance
 */
typedef struct
{
    UartInstance_s *uart_instance;              //!< UART instance for communication
    RemoteCtrlInfo_s remote_ctrl_data;          //!< Decoded remote control data
    KeyboardMouseOperation_s keyboard_mouse_operation;  //!< Keyboard and mouse operation data
} DbusInstance_s;

// Function declarations

/**
 * @brief Decode DR16 receiver data buffer into remote control structure
 * @param[in] dbus_buf Raw data buffer received from DR16 remote controller
 * @param[out] remote_ctrl_data Decoded remote control data structure
 * @return None
 * @note This function parses the raw byte stream from DR16 and extracts all control channels
 */
void Remote_Ctrl_Dbus_Decode(volatile const uint8_t *dbus_buf, RemoteCtrlInfo_s *remote_ctrl_data);

/**
 * @brief Register and initialize a new DBUS instance
 * @param[in] config Pointer to DBUS configuration structure
 * @return Pointer to created DbusInstance_s structure, or NULL if failed
 * @note Allocates memory for new instance and registers with UART driver
 */
DbusInstance_s *Dbus_Register(DbusConfig_s *config);

/**
 * @brief DBUS module UART receive complete callback function
 * @param[in] id Pointer to DbusInstance_s structure (passed as user data)
 * @param[in] size Size of received data in bytes
 * @return None
 * @note This function is called by UART driver when DMA reception is complete
 * @note Implements double buffer switching for continuous data reception
 */
void Dbus_RxCallback(void* id, uint16_t size);

#endif
