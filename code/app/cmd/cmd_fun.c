#include "cmd_fun.h"
#include "fs_i6x.h"
#include "basic_math.h"
#include "cmsis_os.h"
#include "bmi088.h"
RcCmd_s rc_cmd;
I6xInstance_s* i6x;
float accel_data[3];
float gyro_data[3];
float temperature;
extern Bmi088Instance_s *bmi088_Instance;
void USER_CMD_Init(void)
{
    Bmi088_Init(&hspi2);
    BMI088_Init();
    i6x = I6x_Register(&huart5);
}

void Cmd_Read(void){
    rc_cmd.data.x_val = Ramp_Read(&i6x->data.rc_data.x_ramp);
    rc_cmd.data.y_val = Ramp_Read(&i6x->data.rc_data.y_ramp);
    rc_cmd.data.pitch_pos = Ramp_Read(&i6x->data.rc_data.pitch_ramp);
    rc_cmd.data.yaw_pos = Ramp_Read(&i6x->data.rc_data.yaw_ramp);
    rc_cmd.data.fine_yaw_pos = Ramp_Read(&i6x->data.rc_data.fine_yaw);
    rc_cmd.data.fine_pitch_pos = Ramp_Read(&i6x->data.rc_data.fine_pitch);
    rc_cmd.state.cmd_state = i6x->data.sw.b;
    rc_cmd.state.chassis_state = i6x->data.sw.c;
    rc_cmd.state.gimbal_state = i6x->data.sw.d;
}

/* USER CODE BEGIN Header_cmd_task */
/**
* @brief Function implementing the cmd thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_cmd_task */
 void cmd_task(void const * argument)
{
    /* USER CODE BEGIN cmd_task */
    /* Infinite loop */
    for(;;)
    {
        BMI088_read(accel_data, gyro_data, &temperature);
        Cmd_Read();
        osDelay(1);
    }
    /* USER CODE END cmd_task */
}
