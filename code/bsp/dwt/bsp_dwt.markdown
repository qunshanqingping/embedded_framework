# bsp_dwt

## 简介
bsp_dwt 是一个基于HAL库的通过计算CPU频率来实现系统时间和阻塞延时的封装库。

## 用法
### 1. 初始化
在主函数中调用 `Dwt_Init()` 函数进行初始化。注意，该函数必须在 `HAL_Init()` 和 `SystemClock_Config()` 之后调用。

```c
  /* USER CODE BEGIN 2 */
  Dwt_Init();
  /* USER CODE END 2 */
```
### 2. 更新系统时间
裸机在主循环中调用 `Dwt_Update()` 函数来更新系统时间。使用RTOS时，可以在空闲任务中调用该函数。

```c
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    Dwt_Update();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
```
或者

```c
/* USER CODE BEGIN Header_Cmd_Task */
/**
* @brief Function implementing the Start_Cmd_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Cmd_Task */
__weak void Cmd_Task(void const * argument)
{
  /* USER CODE BEGIN Cmd_Task */
  /* Infinite loop */
  for(;;)
  {
    Dwt_Update();
    osDelay(1);
  }
  /* USER CODE END Cmd_Task */
}
```
### 3. 计算间隔时间
```c
float Get_Time_Delta(uint32_t *cnt_last)
double Get_Time_Delta_64(uint32_t *cnt_last)
```
单精度和双精度版本，传入上次的计数值指针，返回当前时间与上次时间的差值，单位为秒，并更新上次计数值。

### 4. 获取系统时间
```c
'Dwt_Get_Time_Line_S()' 函数返回系统运行时间，单位为秒。
'Dwt_Get_Time_Line_Ms()' 函数返回系统运行时间，单位为毫秒。
'Dwt_Get_Time_Line_Us()' 函数返回系统运行时间，单位为微秒。
```
返回值均为float类型。

### 5. 阻塞延时
```c
/**
 * @brief 阻塞式延时函数(秒)
 * @note 使用DWT计数器实现精确延时，单位为秒,范围为1-255秒
 * @param delay_time 延时时间，单位为秒
 */
void Dwt_Delay_S(const uint8_t delay_time);

/**
 * @brief 阻塞式延时函数(毫秒)
 * @note 使用DWT计数器实现精确延时，单位为毫秒，范围为1-65535毫秒
 * @param delay_time 延时时间，单位为毫秒
 */
void Dwt_Delay_Ms(const uint16_t delay_time);

/**
 * @brief 阻塞延式时函数(微秒)
 * @note 使用DWT计数器实现精确延时，单位为微秒，范围为1-65535微秒
 * @param delay_time 延时时间，单位为微秒
 */
void Dwt_Delay_Us(const uint16_t delay_time);
```