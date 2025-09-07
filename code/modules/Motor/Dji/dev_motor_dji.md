# dev_motor_dji的
## 概述
dev_motor_dji是一个用于驱动大疆电机的程序，提供了大疆电机的控制功能。

## 开发环境
- 硬件平台：STM32F4系列(RoboMasterC板)
- 开发工具：Keil MDK-ARM
- 相关库：CMSIS, HAL

## 适配电机
- 大疆M2006、M3508、GM6020

## 软件要求
- 需要bsp_can、alg_pid库

## 实例定义
```c
#define DJI_MOTOR_MAX_CNT 16               // DJI电机最大数量

typedef struct{
    DjiMotorType_e type;                   // 电机类型
    uint8_t id;                            // 电机ID(0~8)
    DjiMotorControlMode_e control_mode;    // 电机控制模式
    CanInstance_s *can_instance;           // 电机CAN实例

    PidInstance_s *angle_pid;              // 角度控制PID
    PidInstance_s *velocity_pid;           // 速度控制PID
    PidInstance_s *imu_pid;                // imu角度控制PID

    uint8_t reduction_ratio;               // 减速比
    
    float out_position;                    // 电机输出轴角度(-PI~PI)
    float out_velocity;                    // 电机输出轴速度(rpm)
    
    int16_t total_position;                // 电机转子角度编码器值
    int16_t total_last_position;           // 电机转子上次角度编码器值
    float total_velocity;                  // 电机转子速度(rpm)
    int16_t total_sum_angle;               // 电机转子累计旋转总角度
    float current;                         // 电机电流(A)
    float voltage;                         // 电机电压(V)
    float power;                           // 电机功率(W)
    float temperature;                     // 电机温度(°C)
    
    float imu;              // IMU角度(-PI~PI)
    float target_position;  // 电机目标角度(-PI~PI)
    float target_velocity;  // 电机目标速度(rpm)
    float target_imu;       // IMU目标角度(-PI~PI)

    float output;           // 电机设定值(电流或电压)
}DjiMotorInstance_s;
```

## 外部接口
```c
/**
 * @brief 注册Dji电机实例
 * @param config DJI电机初始化配置结构体指针
 * @return 成功返回DjiMotorInstance_s指针，失败返回NULL
 * @note 调用前需要确认CAN初始化成功
 * @date 2025-07-03
 */
DjiMotorInstance_s *Motor_Dji_Register(DjiMotorInitConfig_s *config);

/**
 * @brief Dji电机获取imu数据
 * @details 该函数用于获取电机的IMU数据，专门为RM比赛中需要更具陀螺仪控制而设计
 * @param motor 电机实例指针
 * @param angle IMU角度(-PI~PI)
 * @return 成功返回true，失败返回false
 * @note 该函数仅在有IMU控制模式下有效,需要在电机控制前调用
 * @date 2025-07-03
 */
bool Motor_Dji_Getimudate(DjiMotorInstance_s *motor, float angle);

/**
 * @brief Dji电机控制函数
 * @details 该函数会根据电机的工作模式自动控制
 * @param motor 电机实例指针
 * @param taget 控制量目标值
 * @return 成功返回true，失败返回false
 * @note 如果电机是速度模式，则target为目标转速 
 * @note 如果电机是位置模式，则targrt为目标角度
 * @note 如果电机是IMU模式，电机会更具Motor_Dji_Getimudate得到的数据和设定值做控制
 * @date 2025-07-03
 */
bool Motor_Dji_Control(DjiMotorInstance_s *motor, float taget);

/**
 * @brief Dji发送数据函数
 * @param motor 电机实例指针
 * @return 如果成功发送数据返回true，失败返回false 
 * @note 该函数会自动把与该电机发送id一致的电机数据拉取并发送
 * @date 2025-07-03
 */
bool Motor_Dji_Transmit(DjiMotorInstance_s *motor);

/**
 * @brief Dji电机切换工作模式
 * @param motor 电机实例指针
 * @param target_mode 目标工作模式
 * @return 如果成功切换模式返回true，失败返回false
 * @date 2025-07-03
 */
bool Motor_Dji_Change_Mode(DjiMotorInstance_s *motor, DjiMotorControlMode_e target_mode);
```

## 私有函数
```c
/**
 * @brief Dji电机分组
 * @param tx_id 发送id  
 * @param can_handle can句柄
 * @param tx_buff 发送缓存
 * @return 分组成功返回true
 * @date 2025-07-03
 */
static bool Motor_Dji_Grouping(uint32_t tx_id, CAN_HandleTypeDef *can_handle, uint8_t tx_buff[8])

/**
 * @brief Dji电机解码
 * @param can_instance can实例指针
 * @date 2025-07-03
 */
static void Motor_Dji_Decode(CanInstance_s *can_instance)
```

## 参考资料
- [湖南大学跃鹿战队开源框架](https://github.com/HNUYueLuRM/basic_framework)
- [RoboMaster C板手册](https://www.robomaster.com/zh-CN/products/components/general/development-board-type-c/info)




