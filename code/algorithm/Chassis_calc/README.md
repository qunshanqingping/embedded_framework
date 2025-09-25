# chassis_calc的使用文档

## 注意事项
1. 请注意麦轮轮组的装配方式，根据俯视图(从顶部看车体轮胎分布)可以分成**X形**和**O形**两种装配方式
    - X形装配：底盘config为**Mecanum_Wheel**
    - O形装配：目前不支持
2. 前置库文件依赖性：
    - `dev_motor_dji`：用于电机控制
    - `bsp_can`：用于CAN总线通信
    - `alg_pid`：用于PID控制

## 概述

chassis_calc是一个用于控制底盘的解算程序，提供了基础的麦克纳姆轮，全向轮的解算后的正常移动，底盘跟随，小陀螺等功能。

## 硬件要求

1. 本解算程序采用右手系,x轴正方向为前侧,y轴正方向为左方,且要求轮子正转为逆时针方向
2. 面朝底盘正方向，电机索引0-3（motor_id数组内)分别位于底盘**左前，左后，右后，右前**，请将电机实际编号（和CANID相关）按照这个顺序填入motor_id数组
> 例如，实际底盘电机按照顺序的CANID分别为1，3，4，2，则motor_id数组应为{1,3,4,2}，注意电机编号从1开始

## 使用例程

```c
float Vx = 0.0f; // 底盘X轴速度
float Vy = 0.0f; // 底盘Y轴速度
float Vw = 0.0f; // 底盘旋转角速度
float Vx_max = 1.0f; // 底盘X轴最大速度
float Vy_max = 1.0f; // 底盘Y轴最大速度
float Vw_max = 1.0f; // 底盘旋转角最大

Dr16Instance_s *dr16_instance = NULL;
ChassisInstance_s* chassis = NULL;

const DjiMotorInitConfig_s chassis_motor_config = {
    .type = M3508,
    .control_mode = DJI_VELOCITY,
    .id = 1,
    .can_config = {
        .can_handle = &hcan1,
        .tx_id = 0x200,
        .rx_id = 0x201,
    },
    .reduction_ratio = 19,
    .angle_pid_config = {
        .kp = 0.5f,
        .ki = 0.01f,
        .kd = 0.01f,
        .kf = 0.0f,
        .angle_max = 0,
        .i_max = 1000.0f,
        .out_max = 16384.0f,
        .dead_zone = 0,
        .i_variable_min = 0.0f,
        .i_variable_max = 0.0f,
        .d_first = 0
    },
    .velocity_pid_config = {
        .kp = 15.0f,
        .ki = 1.5f,
        .kd = 0.0f,
        .kf = 0.0f,
        .angle_max = 0,
        .i_max = 2000.0f,
        .out_max = 16384.0f,
        .dead_zone = 0,
        .i_variable_min = 0.0f,
        .i_variable_max = 0.0f,
        .d_first = 0
    }
};

ChassisInitConfig_s chassis_init = {
    .type = Mecanum_Wheel,
    .motor_config = chassis_motor_config,
    .motor_id = {1, 3, 4, 2},
    .wheel_radius = 0.076f,
    .length_a = 0.37f,
    .length_b = 0.451f,
    .chassis_radius = 0.292f,
    .gimbal_yaw_half = 0.0f,
    .gimbal_yaw_zero = 0.0f,
};

void App_ChassisTask(void const* argument)
{
    Can_Init();
    // 注册底盘实例
    while (chassis == NULL) chassis = Chassis_Register(&chassis_init);

    while (1)
    {
        chassis->Chassis_speed.Vx = Vx;
        chassis->Chassis_speed.Vy = Vy;
        chassis->Chassis_speed.Vw = Vw;

        Chassis_Control(chassis);
        osDelay(1);
    }
}
```
## 使用示例
- 首先我们需要在app层底盘task文件配置如下底盘config(以全向轮为例)
```c
static ChassisInitConfig_s Chassis_config={
		.type = Omni_Wheel,//底盘类型
		.gimbal_yaw_zero = ,//正方向云台编码值
		.gimbal_yaw_half = ,//负方向云台编码值(如果不需要两个方向上的底盘跟随可以不配置)
        .wheel_radius= ,//轮子半径
	    .chassis_radius= ,//底盘半径
		.gimbal_follow_pid_config={
	  .kp = ,//理论为负
      .ki = ,
      .kd = ,
      .i_max = ,
      .out_max = ,
		},//底盘跟随pid
		.motor_config={
    .type = M3508,
    .control_mode = DJI_VELOCITY,
    .id = 1,
    .can_config = {
      .can_handle = &hcan1,//根据实际使用为准
      .tx_id =,
      .rx_id =,
    },
    .reduction_ratio = 19.0f,
    .velocity_pid_config={
      .kp = ,
      .ki = ,
      .kd = ,
      .i_max =,
      .out_max = ,
    }
  }
	};//底盘电机pid(配置1号电机即可,会依次自动配置)
```
- 然后在task调用底盘实例 
```c
ChassisInstance_s *Chassis;
```
- 注册底盘实例 
```c
Chassis = Chassis_Register(&Chassis_config);
```
- 在循环中选择底盘工作模式并将所需速度传入Chassis_Speed结构体中并调用函数
```c
Chassis_Mode_Choose(Chassis);
Chassis_Control(Chassis);
```
-如上即可

## 注意事项

- 使用前先确保各轮子正转方向为逆时针
- 轮子电调为id为1——4
- 为了实现坐标系转换的正确和底盘跟随,需要在云台task中将yaw轴电机编码值发给底盘实例的gimbal_yaw_angle(如果仅测试底盘能动性,则可无视)
- 先调好使用陀螺仪下yaw轴的pid后再使用小陀螺与跟随模式
- 如果发现底盘未注册成功,请检查config配置

## 待开发

> 舵轮程序测试及完善

> 添加功率限制



