/*
 * @file sum_init.c
 * @brief Sum initialization includes application initialization,
 * module initialization,bsp initialization
 * @author Adonis Jin
 * @date 2021-09-07
 * @version 1.0.0
 */

#include "sum_init.h"

#include <usart.h>

#include "basic_math.h"

#include "bsp_dwt.h"
#include "bsp_log.h"
// #include "bsp_fdcan.h"
#include "bsp_can.h"
#include "cmsis_os.h"
static void test_can(void);
/**
 * @brief Initializes the Board Support Package (BSP)
 * @todo repair the log initialization function
 */
static void Bsp_Init(void)
{
    /* Initialize the BSP */
    Dwt_Init();
    Log_Init();
}

/**
 * @brief Initializes the module library.
 * @details This function initializes the DBus communication. If the initialization fails, it logs an error message.
 */

/**
 * @brief Initializes the application
 *
 * This function is responsible for performing any necessary initialization steps for the application.
 * It is part of the overall initialization process, which includes initializing the Board Support Package (BSP),
 * the module library, and finally the application itself.
 */
static void App_Init(void)
{
    /* Initialize the application */
}

/**
 * @brief Initializes the system by sequentially initializing the Board Support Package (BSP), module library, and application.
 * This function serves as a starting point for the initialization process, ensuring that all necessary components are set up before the main application logic begins.
 */
void Sum_Init(void)
{
    /* Initialize the BSP */
    Bsp_Init();

    /* Initialize the module library */

    /* Initialize the application */
    App_Init();
}

/**
 * @brief Dbus initialization function
 * @details This function initializes the DBUS module by allocating memory for the configuration
 * @return Pointer to the initialized DbusInstance_s structure, or NULL if initialization failed
 * @note It sets up the UART configuration for DBUS communication, registers the instance,
 */

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
static void test_init(void)
{
    test = Can_Register(&test_config);
}

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