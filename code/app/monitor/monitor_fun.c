#include "monitor_fun.h"
#include "cmsis_os.h"
#include "watch_dog.h"
#include "plf_log.h"
/* USER CODE BEGIN Header_monitor_task */
/**
* @brief Function implementing the monitor thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_monitor_task */
void monitor_task(void const * argument)
{
    /* USER CODE BEGIN monitor_task */
    Log_Information("WatchDog Stat");
    /* Infinite loop */
    for(;;)
    {
        WatchDog_Callback();
        osDelay(1000);
    }
    /* USER CODE END monitor_task */
}