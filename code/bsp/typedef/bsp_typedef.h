/*
 * @file bsp_typedef.h
 * @brief Basic type definitions for the bsp library header file to improve reuse rate
 * @author Adonis Jin
 * @date 2025/08/07
 * @version 1.0.0
 */

#ifndef BSP_TYPEDEF_H
#define BSP_TYPEDEF_H

typedef enum {
    STD_FRAME = 0, // 标准帧
    EXT_FRAME = 1  // 扩展帧
}CAN_DataFrameTypeDef;
/**
 * @brief Communication peripheral transfer modes
 * @author honor
 * @date 2025/08/07
 * @version 1.0.0
 * @note Defines the data transfer modes for communication peripherals
 */
typedef enum
{
    BLOCK_MODE = 0,  //!< Blocking mode - CPU waits for transfer completion
    IT_MODE = 1,     //!< Interrupt mode - CPU handles transfer via interrupts
    DMA_MODE = 2     //!< DMA mode - Dedicated hardware handles data transfer
} TransferMode_e;

/**
 * @brief Communication direction modes
 * @author honor
 * @date 2025/08/07
 * @version 1.0.0
 * @note Defines the communication direction for peripherals
 */
typedef enum
{
    RX_TX_MODE = 0,  //!< Full-duplex mode - simultaneous receive and transmit
    RX_MODE = 1,     //!< Receive mode only
    TX_MODE = 2      //!< Transmit mode only
} DirectionMode_e;

/**
 * @brief Device operation modes
 * @author honor
 * @date 2025/08/07
 * @version 1.0.0
 * @note Defines master/slave operation modes for communication protocols
 */
typedef enum {
    SLAVE_MODE = 0,  //!< Slave mode - device responds to master commands
    MASTER_MODE = 1  //!< Master mode - device controls communication
} DeviceMode_e;

/**
 * @brief Buffer modes for data transfer
 * @author honor
 * @date 2025/08/07
 * @version 1.0.0
 * @note Defines buffering strategies for data transfer operations
 */
typedef enum {
    SINGLE_BUFFER_MODE = 0,  //!< Single buffer mode - one buffer for data transfer
    DOUBLE_BUFFER_MODE = 1   //!< Double buffer mode - two buffers for continuous transfer
} BufferMode_e;

#endif //BSP_TYPEDEF_H
