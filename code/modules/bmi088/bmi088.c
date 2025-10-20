#include "bmi088.h"
#include "bsp_gpio.h"

Bmi088Instance_s *bmi088_Instance;
/**
 * @brief 拉低BMI088加速度计片选
 */
void BMI088_ACCEL_NS_L(void)
{
    Gpio_Reset(bmi088_Instance->accel_instance->cs_pin);
    // HAL_GPIO_WritePin(ACC_CS_GPIO_Port, ACC_CS_Pin, GPIO_PIN_RESET);
}

/**
 * @brief 拉高BMI088加速度计片选
 */
void BMI088_ACCEL_NS_H(void)
{
    Gpio_Set(bmi088_Instance->accel_instance->cs_pin);
    // HAL_GPIO_WritePin(ACC_CS_GPIO_Port, ACC_CS_Pin, GPIO_PIN_SET);
}

/**
 * @brief 拉低BMI088陀螺仪片选
 */
void BMI088_GYRO_NS_L(void)
{
    Gpio_Reset(bmi088_Instance->gyro_instance->cs_pin);
    // HAL_GPIO_WritePin(GYRO_CS_GPIO_Port, GYRO_CS_Pin, GPIO_PIN_RESET);
}

/**
 * @brief 拉高BMI088陀螺仪片选
 */
void BMI088_GYRO_NS_H(void)
{
    Gpio_Set(bmi088_Instance->gyro_instance->cs_pin);
    // HAL_GPIO_WritePin(GYRO_CS_GPIO_Port, GYRO_CS_Pin, GPIO_PIN_SET);
}
/**
 * @brief BMI088 SPI 读写一个字节
 * @param txdata 要发送的数据
 * @return 接收到的数据
 */
uint8_t BMI088_read_write_byte(uint8_t txdata)
{
    uint8_t rx_data;
    HAL_SPI_TransmitReceive(&BMI088_USING_SPI_UNIT, &txdata, &rx_data, 1, 1000);
    return rx_data;
}

/**
 * @brief BMI088 写单个寄存器
 * @param reg 寄存器地址
 * @param data 要写入的数据
 */
 void BMI088_write_single_reg(uint8_t reg, uint8_t data)
{
    BMI088_read_write_byte(reg);
    BMI088_read_write_byte(data);
}

/**
 * @brief BMI088 读单个寄存器
 * @param reg 寄存器地址
 * @param return_data 读取到的数据指针
 */
 void BMI088_read_single_reg(uint8_t reg, uint8_t *return_data)
{
    BMI088_read_write_byte(reg | 0x80);
    *return_data = BMI088_read_write_byte(0x55);
}

/**
 * @brief BMI088 读多个寄存器
 * @param reg 起始寄存器地址
 * @param buf 读取到的数据缓冲区指针
 * @param len 要读取的字节数
 */
void BMI088_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len)
{
    BMI088_read_write_byte(reg | 0x80);

    while (len != 0)
    {

        *buf = BMI088_read_write_byte(0x55);
        buf++;
        len--;
    }
}