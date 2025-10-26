#ifndef BMI088_H
#define BMI088_H
#include "spi.h"
#include "bsp_spi.h"
#define BMI088_USING_SPI_UNIT hspi2
#define BMI088_SPI_TIMEOUT_MS 1000
typedef struct{

    SpiInstance_s *accel_instance;
    SpiInstance_s *gyro_instance;
}Bmi088Instance_s;
typedef struct{
    SpiInitConfig_s accel_spi_config;
    SpiInitConfig_s gyro_spi_config;
}Bmi088InitConfig_s;
extern Bmi088Instance_s *bmi088_Instance;

/**
 * @brief BMI088 初始化函数
 * @param bmi088_using_spi 使用的 SPI 句柄指针
 */
void Bmi088_Init(SPI_HandleTypeDef *bmi088_using_spi);
/**
 * @brief 读取BMI088传感器数据
 * @param gyro 存储陀螺仪数据的数组指针，长度为3
 * @param accel 存储加速度计数据的数组指针，长度为3
 * @param temperate 存储温度数据的指针
 */
void BMI088_read(float gyro[3], float accel[3], float *temperate);


#endif //BMI088_H