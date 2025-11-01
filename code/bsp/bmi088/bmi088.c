/**
 * @file bmi088.c
 * @brief BMI088 6轴IMU传感器驱动实现文件
 * @details 实现BMI088加速度计和陀螺仪的初始化、配置和数据读取功能
 * @author Embedded Framework Team
 * @date 2025
 * @version 1.0
 */

#include "bmi088.h"
#include "bmi088reg.h"
#include "bsp_spi.h"
#include "memory_management.h"
#include "memory.h"
#include "plf_log.h"
#include "string.h"
#include "robot_config.h"

/* ========================= 全局变量定义 ========================= */

float BMI088_ACCEL_SEN = BMI088_ACCEL_3G_SEN; /**< 加速度计灵敏度系数 */
float BMI088_GYRO_SEN = BMI088_GYRO_2000_SEN; /**< 陀螺仪灵敏度系数 */
static Bmi088Instance_s* bmi088; /**< BMI088实例指针 */

/* ========================= 私有函数实现 ========================= */

/**
 * @brief BMI088微秒级延时函数
 * @param us 要延时的微秒数
 * @details 使用SysTick定时器实现精确的微秒级延时
 * @note 基于480MHz系统时钟计算
 */
static void BMI088_delay_us(uint16_t us)
{
    uint32_t ticks = 0; // 需要的时钟周期数
    uint32_t told = 0; // 上一次的计数值
    uint32_t tnow = 0; // 当前的计数值
    uint32_t tcnt = 0; // 累计的时钟周期
    uint32_t reload = 0; // 重载值

    reload = SysTick->LOAD; // 获取SysTick重载值
    ticks = us * 480; // 计算需要的时钟周期(480MHz)
    told = SysTick->VAL; // 获取当前计数值

    while (1)
    {
        tnow = SysTick->VAL; // 获取新的计数值
        if (tnow != told)
        {
            if (tnow < told) // 正常递减
            {
                tcnt += told - tnow;
            }
            else // 发生溢出
            {
                tcnt += reload - tnow + told;
            }
            told = tnow;
            if (tcnt >= ticks) // 达到目标时钟周期
            {
                break;
            }
        }
    }
}

/**
 * @brief BMI088毫秒级延时函数
 * @param ms 要延时的毫秒数
 * @details 延时指定毫秒数，通过微秒延时实现
 */
static void BMI088_delay_ms(uint16_t ms)
{
    while (ms--)
    {
        BMI088_delay_us(1000); // 调用微秒延时实现毫秒延时
    }
}

/**
 * @brief BMI088内部注册函数
 * @param config BMI088初始化配置结构体指针
 * @return BMI088实例指针，注册成功返回实例指针，失败返回NULL
 * @details 根据配置参数创建BMI088实例，并注册加速度计和陀螺仪SPI接口
 */
Bmi088Instance_s* Bmi088_Internal_Register(Bmi088InitConfig_s* config)
{
    if (config == NULL)
    {
        Log_Error("Bmi088_Internal_Register : config is NULL");
        return NULL;
    }

    // 分配内存给BMI088实例
    Bmi088Instance_s* instance = user_malloc(sizeof(Bmi088Instance_s));
    if (instance == NULL)
    {
        Log_Error("Bmi088_Internal_Register : instance Malloc Failed");
        return NULL;
    }

    memset(instance, 0, sizeof(Bmi088Instance_s));

    // 注册加速度计和陀螺仪SPI接口
    instance->accel = Spi_Register(&config->accel);
    instance->gyro = Spi_Register(&config->gyro);

    if (instance->accel == NULL || instance->gyro == NULL)
    {
        Log_Error("Bmi088_Internal_Register : Spi_accel or Spi_gyro Malloc Failed");
        user_free(instance);
        return NULL;
    }

    return instance;
}

/**
 * @brief BMI088注册函数
 * @param spi_handle_number SPI句柄编号
 * @return BMI088实例指针，注册成功返回实例指针，失败返回NULL
 * @details 使用默认配置参数创建BMI088实例，自动配置加速度计和陀螺仪SPI接口
 */
Bmi088Instance_s* Bmi088_Register(uint8_t spi_handle_number)
{
    // 分配配置结构体内存
    Bmi088InitConfig_s* config = user_malloc(sizeof(Bmi088InitConfig_s));
    if (config == NULL)
    {
        Log_Error("Bmi088_Register : config is NULL");
        return NULL;
    }

    memset(config, 0, sizeof(Bmi088InitConfig_s));

    // 配置加速度计SPI参数
    config->accel.topic_name = "BMI088_ACCEL_SPI";
    config->accel.spi_handle_number = spi_handle_number;
    config->accel.mode = BLOCK_MODE;
    config->accel.timeout = 1000;
    config->accel.cs_port = ACC_CS_GPIO_Port;
    config->accel.cs_pin = ACC_CS_Pin;

    // 配置陀螺仪SPI参数
    config->gyro.topic_name = "BMI088_GYRO_SPI";
    config->gyro.spi_handle_number = spi_handle_number;
    config->gyro.mode = BLOCK_MODE;
    config->gyro.timeout = 1000;
    config->gyro.cs_port = GYRO_CS_GPIO_Port;
    config->gyro.cs_pin = GYRO_CS_Pin;

    // 分配实例内存
    Bmi088Instance_s* instance = user_malloc(sizeof(Bmi088Instance_s));
    if (instance == NULL)
    {
        Log_Error("Bmi088_Register : instance Malloc Failed");
        user_free(config);
        return NULL;
    }

    memset(instance, 0, sizeof(Bmi088Instance_s));

    // 注册加速度计和陀螺仪SPI接口
    instance->accel = Spi_Register(&config->accel);
    instance->gyro = Spi_Register(&config->gyro);

    if (instance->accel == NULL || instance->gyro == NULL)
    {
        Log_Error("Bmi088_Register : Spi_accel or Spi_gyro Malloc Failed");
        user_free(instance);
        user_free(config);
        return NULL;
    }

    user_free(config); // 释放配置结构体内存
    return instance;
}

/**
 * @brief 读取BMI088加速度计寄存器数据
 * @param reg 寄存器地址
 * @param rx_buf 存储读取数据的缓冲区
 * @param len 读取的数据长度
 * @details BMI088要求在不释放CS的情况下连续读取，读取时需要设置读取位(0x80)
 */
static void Accel_Read(const uint8_t reg, uint8_t* rx_buf, const uint8_t len)
{
    uint8_t tx_buf[8] = {0};
    tx_buf[0] = reg | 0x80; // 设置读取位
    Spi_TransmitReceive(bmi088->accel, tx_buf, rx_buf, len + 2, 1000);
}

/**
 * @brief 读取BMI088陀螺仪寄存器数据
 * @param reg 寄存器地址
 * @param rx_buf 存储读取数据的缓冲区
 * @details BMI088要求在不释放CS的情况下连续读取，读取时需要设置读取位(0x80)
 */
static void Gyro_Read(uint8_t reg, uint8_t* rx_buf)
{
    uint8_t tx_buf[9] = {0};
    tx_buf[0] = reg | 0x80; // 设置读取位
    Spi_TransmitReceive(bmi088->gyro, tx_buf, rx_buf, 9, 1000);
}

/**
 * @brief 写入BMI088加速度计单个寄存器
 * @param reg 寄存器地址
 * @param data 写入的数据
 * @details 向加速度计的指定寄存器写入一个字节的数据
 */
static void Accel_Write_Single_Reg(const uint8_t reg, const uint8_t data)
{
    uint8_t tx_buf[2] = {reg, data}; // 组装发送数据：寄存器地址+数据
    Spi_Transmit(bmi088->accel, tx_buf, 2);
}

/**
 * @brief 写入BMI088陀螺仪单个寄存器
 * @param reg 寄存器地址
 * @param data 写入的数据
 * @details 向陀螺仪的指定寄存器写入一个字节的数据
 */
static void Gyro_Write_Single_Reg(const uint8_t reg, const uint8_t data)
{
    uint8_t tx_buf[2] = {reg, data}; // 组装发送数据：寄存器地址+数据
    Spi_Transmit(bmi088->gyro, tx_buf, 2);
}

/**
 * @brief 读取BMI088加速度计单个寄存器
 * @param reg 寄存器地址
 * @return 读取到的寄存器值
 * @details 从加速度计的指定寄存器读取一个字节的数据
 */
static uint8_t Accel_Read_Single_Reg(const uint8_t reg)
{
    uint8_t tx_buf[3] = {0};
    tx_buf[0] = reg | 0x80; // 设置读取位
    uint8_t rx_buf[3] = {0};
    Spi_TransmitReceive(bmi088->accel, tx_buf, rx_buf, 3, 1000);
    return rx_buf[2]; // 返回第3个字节（实际数据）
}

/**
 * @brief 读取BMI088陀螺仪单个寄存器
 * @param reg 寄存器地址
 * @return 读取到的寄存器值
 * @details 从陀螺仪的指定寄存器读取一个字节的数据
 */
static uint8_t Gyro_Read_Single_Reg(uint8_t reg)
{
    uint8_t tx_buf[3] = {0};
    uint8_t rx_buf[3] = {0};
    tx_buf[0] = reg | 0x80; // 设置读取位
    Spi_TransmitReceive(bmi088->gyro, tx_buf, rx_buf, 2, 1000);
    return rx_buf[1]; // 返回第2个字节（实际数据）
}


/* ========================= 配置数据表 ========================= */

/**
 * @brief BMI088加速度计初始化寄存器配置表
 * @details 每一行包含：[寄存器地址, 配置值, 错误代码]
 *          用于加速度计的初始化配置和错误检测
 */
static uint8_t write_BMI088_accel_reg_data_error[BMI088_WRITE_ACCEL_REG_NUM][3] =
{
    {BMI088_ACC_PWR_CTRL, BMI088_ACC_ENABLE_ACC_ON, BMI088_ACC_PWR_CTRL_ERROR}, // 电源控制：开启加速度计
    {BMI088_ACC_PWR_CONF, BMI088_ACC_PWR_ACTIVE_MODE, BMI088_ACC_PWR_CONF_ERROR}, // 电源配置：活跃模式
    {BMI088_ACC_CONF, BMI088_ACC_NORMAL | BMI088_ACC_800_HZ | BMI088_ACC_CONF_MUST_Set, BMI088_ACC_CONF_ERROR},
    // 加速度计配置：正常模式+800Hz采样率
    {BMI088_ACC_RANGE, BMI088_ACC_RANGE_3G, BMI088_ACC_RANGE_ERROR}, // 量程配置：±3g
    {
        BMI088_INT1_IO_CTRL, BMI088_ACC_INT1_IO_ENABLE | BMI088_ACC_INT1_GPIO_PP | BMI088_ACC_INT1_GPIO_LOW,
        // INT1中断配置：使能+推挑输出+低电平有效
        BMI088_INT1_IO_CTRL_ERROR
    },
    {BMI088_INT_MAP_DATA, BMI088_ACC_INT1_DRDY_INTERRUPT, BMI088_INT_MAP_DATA_ERROR} // 中断映射：数据就绪中断映射到INT1

};
/**
 * @brief BMI088陀螺仪初始化寄存器配置表
 * @details 每一行包含：[寄存器地址, 配置值, 错误代码]
 *          用于陀螺仪的初始化配置和错误检测
 */
static uint8_t write_BMI088_gyro_reg_data_error[BMI088_WRITE_GYRO_REG_NUM][3] =
{
    {BMI088_GYRO_RANGE, BMI088_GYRO_2000, BMI088_GYRO_RANGE_ERROR}, // 量程配置：±2000°/s
    {BMI088_GYRO_BANDWIDTH, BMI088_GYRO_1000_116_HZ | BMI088_GYRO_BANDWIDTH_MUST_Set, BMI088_GYRO_BANDWIDTH_ERROR},
    // 带宽配置：1000Hz ODR + 116Hz带宽
    {BMI088_GYRO_LPM1, BMI088_GYRO_NORMAL_MODE, BMI088_GYRO_LPM1_ERROR}, // 电源模式：正常模式
    {BMI088_GYRO_CTRL, BMI088_DRDY_ON, BMI088_GYRO_CTRL_ERROR}, // 控制寄存器：开启数据就绪
    {
        BMI088_GYRO_INT3_INT4_IO_CONF, BMI088_GYRO_INT3_GPIO_PP | BMI088_GYRO_INT3_GPIO_LOW, // INT3中断配置：推挑输出+低电平有效
        BMI088_GYRO_INT3_INT4_IO_CONF_ERROR
    },
    {BMI088_GYRO_INT3_INT4_IO_MAP, BMI088_GYRO_DRDY_IO_INT3, BMI088_GYRO_INT3_INT4_IO_MAP_ERROR} // 中断映射：数据就绪中断映射到INT3

};

/* ========================= 初始化函数 ========================= */

/**
 * @brief BMI088加速度计初始化函数
 * @return 错误码，BMI088_NO_ERROR表示成功，其他值表示对应错误
 * @details 初始化BMI088加速度计，包括芯片ID检查、软复位和寄存器配置
 */
static uint8_t bmi088_accel_init(void)
{
    uint8_t write_reg_num = 0; // 寄存器编号计数器
    uint8_t res = 0; // 读取结果存储

    // 第一次读取芯片ID（可能不成功）
    res = Accel_Read_Single_Reg(BMI088_ACC_CHIP_ID);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    // 第二次读取芯片ID（确保成功）
    res = Accel_Read_Single_Reg(BMI088_ACC_CHIP_ID);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    // 执行软复位
    Accel_Write_Single_Reg(BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_VALUE);
    BMI088_delay_ms(BMI088_LONG_DELAY_TIME); // 等待复位完成

    // 复位后再次读取芯片ID进行验证
    res = Accel_Read_Single_Reg(BMI088_ACC_CHIP_ID);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    res = Accel_Read_Single_Reg(BMI088_ACC_CHIP_ID);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    // 检查芯片ID是否正确
    if (res != BMI088_ACC_CHIP_ID_VALUE)
    {
        return BMI088_NO_SENSOR;
    }

    // 循环配置所有寄存器
    for (write_reg_num = 0; write_reg_num < BMI088_WRITE_ACCEL_REG_NUM; write_reg_num++)
    {
        // 写入配置值
        Accel_Write_Single_Reg(write_BMI088_accel_reg_data_error[write_reg_num][0],
                               write_BMI088_accel_reg_data_error[write_reg_num][1]);
        BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        // 读回验证配置是否成功
        res = Accel_Read_Single_Reg(write_BMI088_accel_reg_data_error[write_reg_num][0]);
        BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        // 检查配置是否正确
        if (res != write_BMI088_accel_reg_data_error[write_reg_num][1])
        {
            return write_BMI088_accel_reg_data_error[write_reg_num][2]; // 返回对应错误代码
        }
    }
    return BMI088_NO_ERROR; // 初始化成功
}

/**
 * @brief BMI088陀螺仪初始化函数
 * @return 错误码，BMI088_NO_ERROR表示成功，其他值表示对应错误
 * @details 初始化BMI088陀螺仪，包括芯片ID检查、软复位和寄存器配置
 */
static uint8_t bmi088_gyro_init(void)
{
    uint8_t write_reg_num = 0; // 寄存器编号计数器
    uint8_t res = 0; // 读取结果存储

    // 第一次读取芯片ID（可能不成功）
    res = Gyro_Read_Single_Reg(BMI088_GYRO_CHIP_ID);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    // 第二次读取芯片ID（确保成功）
    res = Gyro_Read_Single_Reg(BMI088_GYRO_CHIP_ID);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    // 执行软复位
    Gyro_Write_Single_Reg(BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET_VALUE);
    BMI088_delay_ms(BMI088_LONG_DELAY_TIME); // 等待复位完成

    // 复位后再次读取芯片ID进行验证
    res = Gyro_Read_Single_Reg(BMI088_GYRO_CHIP_ID);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    res = Gyro_Read_Single_Reg(BMI088_GYRO_CHIP_ID);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    // 检查芯片ID是否正确
    if (res != BMI088_GYRO_CHIP_ID_VALUE)
    {
        return BMI088_NO_SENSOR;
    }

    // 循环配置所有寄存器
    for (write_reg_num = 0; write_reg_num < BMI088_WRITE_GYRO_REG_NUM; write_reg_num++)
    {
        // 写入配置值
        Gyro_Write_Single_Reg(write_BMI088_gyro_reg_data_error[write_reg_num][0],
                              write_BMI088_gyro_reg_data_error[write_reg_num][1]);
        BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        // 读回验证配置是否成功
        res = Gyro_Read_Single_Reg(write_BMI088_gyro_reg_data_error[write_reg_num][0]);
        BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        // 检查配置是否正确
        if (res != write_BMI088_gyro_reg_data_error[write_reg_num][1])
        {
            return write_BMI088_gyro_reg_data_error[write_reg_num][2]; // 返回对应错误代码
        }
    }

    return BMI088_NO_ERROR; // 初始化成功
}

/* ========================= 公共接口函数 ========================= */

/**
 * @brief BMI088传感器初始化函数
 * @return 错误码，BMI088_NO_ERROR表示成功，其他值表示对应错误
 * @details 初始化BMI088传感器，包括SPI接口注册、加速度计和陀螺仪初始化
 */
uint8_t BMI088_init(void)
{
    uint8_t error = BMI088_NO_ERROR; // 错误代码累加器

    // 注册BMI088实例
    bmi088 = Bmi088_Register(SPI_IMU_HANDLE);
    if (bmi088 == NULL)
    {
        return BMI088_NO_SENSOR;
    }

    // 初始化加速度计和陀螺仪
    error |= bmi088_accel_init(); // 加速度计初始化
    error |= bmi088_gyro_init(); // 陀螺仪初始化

    return error; // 返回累加错误代码
}


/**
 * @brief 读取BMI088传感器数据
 * @param gyro 陀螺仪数据数组[X, Y, Z]（°/s）
 * @param accel 加速度计数据数组[X, Y, Z]（g）
 * @param temperate 温度数据指针（℃）
 * @details 从 BMI088 读取陀螺仪、加速度计和温度数据，并转换为物理单位
 */
void Bmi088_read(float gyro[3], float accel[3], float* temperate)
{
    uint8_t accel_buf[8] = {0}; // 加速度计数据缓冲区
    uint8_t gyro_buf[9] = {0}; // 陀螺仪数据缓冲区

    // 读取加速度计数据（从 X 轴低字节开始，连续读取 6 个字节）
    Accel_Read(BMI088_ACCEL_XOUT_L, accel_buf, 6);

    // 解析加速度计数据（低字节在前，高字节在后）
    for (uint8_t i = 0; i < 3; i++)
    {
        accel[i] = BMI088_ACCEL_SEN * (float)(int16_t)(accel_buf[i * 2 + 3] << 8 | accel_buf[i * 2 + 2]);
    }

    // 读取陀螺仪数据（从芯片ID寄存器开始，连续读取 9 个字节）
    Gyro_Read(BMI088_GYRO_CHIP_ID, gyro_buf);

    // 验证陀螺仪芯片ID后解析数据
    if (gyro_buf[1] == BMI088_GYRO_CHIP_ID_VALUE)
    {
        // 解析陀螺仪数据（低字节在前，高字节在后）
        for (uint8_t i = 0; i < 3; i++)
        {
            gyro[i] = BMI088_GYRO_SEN * (float)(int16_t)(gyro_buf[i * 2 + 4] << 8 | gyro_buf[i * 2 + 3]);
        }
    }

    // 读取温度数据（从温度高字节开始，连续读取 2 个字节）
    Accel_Read(BMI088_TEMP_M, accel_buf, 2);

    // 解析温度数据（只使用 11 位数据）并转换为摄氏度
    *temperate = (float)(int16_t)(accel_buf[2] << 3 | accel_buf[3] >> 5) * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;
}
