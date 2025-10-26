#include "bmi088.h"
#include "bsp_gpio.h"
#include "../modules/bmi088/BMI088reg.h"
#include "bsp_dwt.h"
#include "bsp_spi.h"
#include "plf_log.h"
#include "string.h"
#include "stdint.h"
#include "memory_management.h"
Bmi088Instance_s *bmi088_Instance;
float BMI088_ACCEL_SEN = BMI088_ACCEL_3G_SEN;
float BMI088_GYRO_SEN = BMI088_GYRO_2000_SEN;
/**
 * @brief BMI088 写加速度计寄存器数量
 */
static uint8_t write_BMI088_accel_reg_data_error[BMI088_WRITE_ACCEL_REG_NUM][3] =
{
    {BMI088_ACC_PWR_CTRL, BMI088_ACC_ENABLE_ACC_ON, BMI088_ACC_PWR_CTRL_ERROR},
    {BMI088_ACC_PWR_CONF, BMI088_ACC_PWR_ACTIVE_MODE, BMI088_ACC_PWR_CONF_ERROR},
    {BMI088_ACC_CONF,  BMI088_ACC_NORMAL| BMI088_ACC_800_HZ | BMI088_ACC_CONF_MUST_Set, BMI088_ACC_CONF_ERROR},
    {BMI088_ACC_RANGE, BMI088_ACC_RANGE_3G, BMI088_ACC_RANGE_ERROR},
    {BMI088_INT1_IO_CTRL, BMI088_ACC_INT1_IO_ENABLE | BMI088_ACC_INT1_GPIO_PP | BMI088_ACC_INT1_GPIO_LOW, BMI088_INT1_IO_CTRL_ERROR},
    {BMI088_INT_MAP_DATA, BMI088_ACC_INT1_DRDY_INTERRUPT, BMI088_INT_MAP_DATA_ERROR}

};

/**
 * @brief BMI088 写陀螺仪寄存器数量
 */
static uint8_t write_BMI088_gyro_reg_data_error[BMI088_WRITE_GYRO_REG_NUM][3] =
{
    {BMI088_GYRO_RANGE, BMI088_GYRO_2000, BMI088_GYRO_RANGE_ERROR},
    {BMI088_GYRO_BANDWIDTH, BMI088_GYRO_1000_116_HZ | BMI088_GYRO_BANDWIDTH_MUST_Set, BMI088_GYRO_BANDWIDTH_ERROR},
    {BMI088_GYRO_LPM1, BMI088_GYRO_NORMAL_MODE, BMI088_GYRO_LPM1_ERROR},
    {BMI088_GYRO_CTRL, BMI088_DRDY_ON, BMI088_GYRO_CTRL_ERROR},
    {BMI088_GYRO_INT3_INT4_IO_CONF, BMI088_GYRO_INT3_GPIO_PP | BMI088_GYRO_INT3_GPIO_LOW, BMI088_GYRO_INT3_INT4_IO_CONF_ERROR},
    {BMI088_GYRO_INT3_INT4_IO_MAP, BMI088_GYRO_DRDY_IO_INT3, BMI088_GYRO_INT3_INT4_IO_MAP_ERROR}

};
/**
 * @brief 拉低BMI088加速度计片选
 */
void BMI088_ACCEL_NS_L(void)
{
    // Gpio_Reset(bmi088_Instance->accel_instance->cs_pin);
    HAL_GPIO_WritePin(ACC_CS_GPIO_Port, ACC_CS_Pin, GPIO_PIN_RESET);
}

/**
 * @brief 拉高BMI088加速度计片选
 */
void BMI088_ACCEL_NS_H(void)
{
    // Gpio_Set(bmi088_Instance->accel_instance->cs_pin);
    HAL_GPIO_WritePin(ACC_CS_GPIO_Port, ACC_CS_Pin, GPIO_PIN_SET);
}

/**
 * @brief 拉低BMI088陀螺仪片选
 */
void BMI088_GYRO_NS_L(void)
{
    // Gpio_Reset(bmi088_Instance->gyro_instance->cs_pin);
    HAL_GPIO_WritePin(GYRO_CS_GPIO_Port, GYRO_CS_Pin, GPIO_PIN_RESET);
}

/**
 * @brief 拉高BMI088陀螺仪片选
 */
void BMI088_GYRO_NS_H(void)
{
    // Gpio_Set(bmi088_Instance->gyro_instance->cs_pin);
    HAL_GPIO_WritePin(GYRO_CS_GPIO_Port, GYRO_CS_Pin, GPIO_PIN_SET);
}
/**
 * @brief BMI088 读写单个字节
 * @param instance SPI 实例指针
 * @param tx_data 要发送的数据
 * @return 接收到的数据
 */
uint8_t BMI088_read_write_byte(SpiInstance_s *instance,uint8_t tx_data)
{
    uint8_t rx_data;
    // Spi_TransmitReceive(instance, &tx_data, &rx_data, 1, BMI088_SPI_TIMEOUT_MS);
    HAL_SPI_TransmitReceive(&BMI088_USING_SPI_UNIT, &tx_data, &rx_data, 1, 1000);
    return rx_data;
}

/**
 * @brief BMI088 写单个寄存器
 * @param instance SPI 实例指针
 * @param reg 寄存器地址
 * @param data 要写入的数据
 */
 void BMI088_write_single_reg(SpiInstance_s* instance, uint8_t reg, uint8_t data)
{
    BMI088_read_write_byte(instance,reg);
    BMI088_read_write_byte(instance,data);
}

/**
 * @brief BMI088 读单个寄存器
 * @param instance SPI 实例指针
 * @param reg 寄存器地址
 * @param return_data 读取到的数据指针
 */
 void BMI088_read_single_reg(SpiInstance_s* instance,uint8_t reg, uint8_t *return_data)
{
    BMI088_read_write_byte(instance,reg | 0x80);
    *return_data = BMI088_read_write_byte(instance,0x55);
}

/**
 * @brief BMI088 读多个寄存器
 * @param reg 起始寄存器地址
 * @param buf 读取到的数据缓冲区指针
 * @param len 要读取的字节数
 */
void BMI088_read_muli_reg(SpiInstance_s* instance,uint8_t reg, uint8_t *buf, uint8_t len)
{
    BMI088_read_write_byte(instance,reg | 0x80);
    uint8_t tx_buf[6] = {0};
    HAL_SPI_TransmitReceive(&BMI088_USING_SPI_UNIT, tx_buf, buf, len, 1000);
    // while (len != 0)
    // {
    //
    //     *buf = BMI088_read_write_byte(instance,0x55);
    //     buf++;
    //     len--;
    // }
}
void BMI088_accel_write_single_reg(uint8_t reg, uint8_t data)
{
    BMI088_ACCEL_NS_L();
    BMI088_write_single_reg(bmi088_Instance->accel_instance,(reg), (data));
    BMI088_ACCEL_NS_H();
}

// void BMI088_accel_read_single_reg(uint8_t reg,uint8_t data)
// {
//     BMI088_ACCEL_NS_L();
//     BMI088_read_write_byte(bmi088_Instance->accel_instance,(reg) | 0x80);
//     BMI088_read_write_byte(bmi088_Instance->accel_instance,0x55);
//     (data) = BMI088_read_write_byte(bmi088_Instance->accel_instance,0x55);
//     BMI088_ACCEL_NS_H();
// }
void BMI088_accel_read_single_reg(uint8_t reg, uint8_t *data)
{
    BMI088_ACCEL_NS_L();
    BMI088_read_write_byte(bmi088_Instance->accel_instance, reg | 0x80);
    *data = BMI088_read_write_byte(bmi088_Instance->accel_instance, 0x55);
    BMI088_ACCEL_NS_H();
}


void BMI088_gyro_read_single_reg(uint8_t reg,uint8_t data)
{
    BMI088_GYRO_NS_L();
    BMI088_read_single_reg(bmi088_Instance->gyro_instance,(reg), &(data));
    BMI088_GYRO_NS_H();
}
void BMI088_gyro_write_single_reg(uint8_t reg,uint8_t data)
{
    BMI088_GYRO_NS_L();
    BMI088_write_single_reg(bmi088_Instance->gyro_instance,(reg), (data));
    BMI088_GYRO_NS_H();
}


// void BMI088_accel_read_muli_reg(uint8_t reg,uint8_t *data,uint8_t len)
// {
//     BMI088_ACCEL_NS_L();
//     BMI088_read_write_byte(bmi088_Instance->accel_instance,(reg) | 0x80);
//     BMI088_read_muli_reg(bmi088_Instance->accel_instance,reg,data, len);
//     BMI088_ACCEL_NS_H();
// }
void BMI088_accel_read_muli_reg(uint8_t reg, uint8_t *data, uint8_t len)
{
    // data[0] = reg | 0x80;
    BMI088_ACCEL_NS_L();
    BMI088_read_muli_reg(bmi088_Instance->accel_instance, reg, data, len);
    BMI088_ACCEL_NS_H();
}
void BMI088_gyro_read_muli_reg(uint8_t reg,uint8_t* data,uint8_t len)
{
    BMI088_GYRO_NS_L();
    BMI088_read_muli_reg(bmi088_Instance->gyro_instance,(reg), (data), (len));
    BMI088_GYRO_NS_H();
}

uint8_t bmi088_accel_init(void)
{
    uint8_t res = 0;
    uint8_t write_reg_num = 0;

    //check commiunication
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, &res);
    Dwt_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, &res);
    Dwt_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    //accel software reset
    BMI088_accel_write_single_reg(BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_VALUE);
    Dwt_delay_ms(BMI088_LONG_DELAY_TIME);

    //check commiunication is normal after reset
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, &res);
    Dwt_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, &res);
    Dwt_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    if (res != BMI088_ACC_CHIP_ID_VALUE)
    {
        return BMI088_NO_SENSOR;
    }

    //set accel sonsor config and check
    for (write_reg_num = 0; write_reg_num < BMI088_WRITE_ACCEL_REG_NUM; write_reg_num++)
    {

        BMI088_accel_write_single_reg(write_BMI088_accel_reg_data_error[write_reg_num][0], write_BMI088_accel_reg_data_error[write_reg_num][1]);
        Dwt_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        BMI088_accel_read_single_reg(write_BMI088_accel_reg_data_error[write_reg_num][0], &res);
        Dwt_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        if (res != write_BMI088_accel_reg_data_error[write_reg_num][1])
        {
            return write_BMI088_accel_reg_data_error[write_reg_num][2];
        }
    }
    return BMI088_NO_ERROR;
}

uint8_t bmi088_gyro_init(void)
{
    uint8_t write_reg_num = 0;
    uint8_t res = 0;

    //check commiunication
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    Dwt_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    Dwt_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    //reset the gyro sensor
    BMI088_gyro_write_single_reg(BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET_VALUE);
    Dwt_delay_ms(BMI088_LONG_DELAY_TIME);
    //check commiunication is normal after reset
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    Dwt_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    Dwt_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    // check the "who am I"
    if (res != BMI088_GYRO_CHIP_ID_VALUE)
    {
        return BMI088_NO_SENSOR;
    }

    //set gyro sonsor config and check
    for (write_reg_num = 0; write_reg_num < BMI088_WRITE_GYRO_REG_NUM; write_reg_num++)
    {

        BMI088_gyro_write_single_reg(write_BMI088_gyro_reg_data_error[write_reg_num][0], write_BMI088_gyro_reg_data_error[write_reg_num][1]);
        Dwt_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        BMI088_gyro_read_single_reg(write_BMI088_gyro_reg_data_error[write_reg_num][0], res);
        Dwt_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        if (res != write_BMI088_gyro_reg_data_error[write_reg_num][1])
        {
            return write_BMI088_gyro_reg_data_error[write_reg_num][2];
        }
    }

    return BMI088_NO_ERROR;
}

uint8_t BMI088_init(void)
{
    uint8_t error = BMI088_NO_ERROR;

    error |= bmi088_accel_init();
    error |= bmi088_gyro_init();

    return error;
}

/**
 * @brief 读取BMI088传感器数据
 * @param gyro 存储陀螺仪数据的数组指针，长度为3
 * @param accel 存储加速度计数据的数组指针，长度为3
 * @param temperate 存储温度数据的指针
 */
void BMI088_read(float gyro[3], float accel[3], float *temperate)
{
    uint8_t buf[8] = {0, 0, 0, 0, 0, 0};

    BMI088_accel_read_muli_reg(BMI088_ACCEL_XOUT_L, buf, 6);

    int16_t bmi088_raw_temp = (int16_t)(((uint16_t)buf[1] << 8) | (uint16_t)buf[0]);

    accel[0] = (float)bmi088_raw_temp * BMI088_ACCEL_SEN;
    bmi088_raw_temp = (int16_t)(((uint16_t)buf[3] << 8) | (uint16_t)buf[2]);
    accel[1] = (float)bmi088_raw_temp * BMI088_ACCEL_SEN;
    bmi088_raw_temp = (int16_t)(((uint16_t)buf[5] << 8) | (uint16_t)buf[4]);
    accel[2] = (float)bmi088_raw_temp * BMI088_ACCEL_SEN;


    BMI088_gyro_read_muli_reg(BMI088_GYRO_CHIP_ID, buf, 8);
    if(buf[0] == BMI088_GYRO_CHIP_ID_VALUE)
    {
        bmi088_raw_temp = (int16_t)(((uint16_t)buf[3] << 8) | (uint16_t)buf[2]);
        gyro[0] = (float)bmi088_raw_temp * BMI088_GYRO_SEN;
        bmi088_raw_temp = (int16_t)(((uint16_t)buf[5] << 8) | (uint16_t)buf[4]);
        gyro[1] = (float)bmi088_raw_temp * BMI088_GYRO_SEN;
        bmi088_raw_temp = (int16_t)(((uint16_t)buf[7] << 8) | (uint16_t)buf[6]);
        gyro[2] = (float)bmi088_raw_temp * BMI088_GYRO_SEN;
    }
    BMI088_accel_read_muli_reg(BMI088_TEMP_M, buf, 2);

    bmi088_raw_temp = (int16_t)((buf[0] << 3) | (buf[1] >> 5));

    if (bmi088_raw_temp > 1023)
    {
        bmi088_raw_temp -= 2048;
    }

    *temperate = (float)bmi088_raw_temp * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;
}

Bmi088Instance_s* BMI088_Register(Bmi088InitConfig_s * config)
{
    if (config == NULL)
    {
        return NULL;
    }
    bmi088_Instance = (Bmi088Instance_s*)user_malloc(sizeof(Bmi088Instance_s));
    if (bmi088_Instance == NULL)
    {
        return NULL;
    }
    memset(bmi088_Instance, 0, sizeof(Bmi088Instance_s));
    bmi088_Instance->accel_instance =Spi_Register(&config->accel_spi_config);
    bmi088_Instance->gyro_instance =Spi_Register(&config->gyro_spi_config);
    if (bmi088_Instance->accel_instance == NULL || bmi088_Instance->gyro_instance == NULL){
        Log_Error("BMI088_Register : Spi_accel or Spi_gyro Malloc Failed");
        user_free(bmi088_Instance);
        return NULL;
    }
    return bmi088_Instance;
}
uint8_t accel_error = 0;
uint8_t gyro_error = 0;

/**
 * @brief BMI088 初始化函数
 * @param bmi088_using_spi 使用的 SPI 句柄指针
 */
void Bmi088_Init(SPI_HandleTypeDef *bmi088_using_spi){
     Bmi088InitConfig_s* config = user_malloc(sizeof(Bmi088InitConfig_s));
    if (config == NULL ){
        Log_Error("Bmi088_Init : config Malloc Failed");
        return;
    }
    memset(config, 0, sizeof(Bmi088InitConfig_s));
    config->accel_spi_config.topic_name = "BMI088_ACCEL_SPI";
    config->accel_spi_config.spi_handle = bmi088_using_spi;
    config->accel_spi_config.mode = BLOCK_MODE;
    config->accel_spi_config.timeout = BMI088_SPI_TIMEOUT_MS;
    config->accel_spi_config.cs_port = ACC_CS_GPIO_Port;
    config->accel_spi_config.cs_pin = ACC_CS_Pin;
    config->gyro_spi_config.topic_name = "BMI088_GYRO_SPI";
    config->gyro_spi_config.spi_handle = bmi088_using_spi;
    config->gyro_spi_config.mode = BLOCK_MODE;
    config->gyro_spi_config.timeout = BMI088_SPI_TIMEOUT_MS;
    config->gyro_spi_config.cs_port = GYRO_CS_GPIO_Port;
    config->gyro_spi_config.cs_pin = GYRO_CS_Pin;

    bmi088_Instance = BMI088_Register(config);
    if (bmi088_Instance == NULL){
        Log_Error("Bmi088_Init : BMI088_Register Failed");
    }
    user_free(config);


    accel_error = bmi088_accel_init();
    gyro_error = bmi088_gyro_init();
}