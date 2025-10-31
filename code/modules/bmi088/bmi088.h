/**
 * @file bmi088.h
 * @brief BMI088 6轴IMU传感器驱动头文件
 * @details 定义BMI088传感器的数据结构、函数接口和相关配置
 * @author Embedded Framework Team
 * @date 2025
 * @version 1.0
 */

#ifndef BMI088_H
#define BMI088_H

/* ========================= 头文件包含 ========================= */

#include "stdint.h"
#include "bsp_spi.h"

/* ========================= 数据结构定义 ========================= */

/**
 * @brief BMI088实例结构体
 * @details 包含BMI088传感器的加速度计和陀螺仪SPI接口实例
 */
typedef struct
{
    SpiInstance_s* accel;  /**< 加速度计SPI接口实例指针 */
    SpiInstance_s* gyro;   /**< 陀螺仪SPI接口实例指针 */
} Bmi088Instance_s;

/**
 * @brief BMI088初始化配置结构体
 * @details 包含BMI088传感器初始化所需的SPI配置参数
 */
typedef struct
{
    SpiInitConfig_s accel; /**< 加速度计SPI初始化配置 */
    SpiInitConfig_s gyro;  /**< 陀螺仪SPI初始化配置 */
} Bmi088InitConfig_s;

/* ========================= 函数声明 ========================= */

/**
 * @brief BMI088传感器初始化函数
 * @return 错误码，BMI088_NO_ERROR表示成功，其他值表示对应错误
 * @details 初始化BMI088传感器，包括SPI接口注册、加速度计和陀螺仪初始化
 */
uint8_t BMI088_init(void);

/**
 * @brief 读取BMI088传感器数据
 * @param gyro 陀螺仪数据数组[X, Y, Z]（°/s）
 * @param accel 加速度计数据数组[X, Y, Z]（g）
 * @param temperate 温度数据指针（℃）
 * @details 从BMI088读取陀螺仪、加速度计和温度数据，并转换为物理单位
 */
void Bmi088_read(float gyro[3], float accel[3], float* temperate);

#endif // BMI088_H
