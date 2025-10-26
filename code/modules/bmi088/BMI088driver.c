#include "BMI088driver.h"
#include "BMI088reg.h"
#include "BMI088Middleware.h"
#include "bsp_spi.h"
#include "memory_management.h"
#include "memory.h"
#include "plf_log.h"
#include "string.h"
#define BMI088_USING_SPI_UNIT hspi2

float BMI088_ACCEL_SEN = BMI088_ACCEL_3G_SEN;
float BMI088_GYRO_SEN = BMI088_GYRO_2000_SEN;
static Bmi088Instance_s* bmi088;
static Bmi088InitConfig_s bmi088_config = {
    .accel = {
        .topic_name = "accel",
        .spi_handle = &BMI088_USING_SPI_UNIT,
        .cs_port = ACC_CS_GPIO_Port,
        .cs_pin = ACC_CS_Pin,
        .mode = BLOCK_MODE,
        .timeout = 1000,
    },
    .gyro = {
        .topic_name = "gyro",
        .spi_handle = &BMI088_USING_SPI_UNIT,
        .cs_port = GYRO_CS_GPIO_Port,
        .cs_pin = GYRO_CS_Pin,
        .mode = BLOCK_MODE,
        .timeout = 1000,
    }
};

Bmi088Instance_s* Bmi088_Register(Bmi088InitConfig_s* config){
    if (config == NULL){
        Log_Error("Bmi088_Register : config is NULL");
        return NULL;
    }
    Bmi088Instance_s* instance = (Bmi088Instance_s*)user_malloc(sizeof(Bmi088Instance_s));
    if (instance == NULL){
        Log_Error("Bmi088_Register : instance Malloc Failed");
        return NULL;
    }
    memset(instance, 0, sizeof(Bmi088Instance_s));
    instance->accel = Spi_Register(&config->accel);
    instance->gyro = Spi_Register(&config->gyro);
    if (instance->accel == NULL || instance->gyro == NULL){
        Log_Error("Bmi088_Register : Spi_accel or Spi_gyro Malloc Failed");
        user_free(instance);
        return NULL;
    }
    return instance;
}

/**
 * @brief          读取BMI088寄存器Accel. BMI088要求在不释放CS的情况下连续读取
 * @param reg      寄存器地址
 * @param rx_buf   存储读取数据的缓冲区
 */
static void Accel_Read(uint8_t reg, uint8_t* rx_buf ,uint8_t len){
    uint8_t tx_buf[8] = {0};
    tx_buf[0] = reg | 0x80;
    Spi_TransmitReceive(bmi088->accel, tx_buf, rx_buf, len+2, 1000);
}

/**
 * @brief          读取BMI088寄存器Gyro. BMI088要求在不释放CS的情况下连续读取
 * @param reg      寄存器地址
 * @param rx_buf   存储读取数据的缓冲区
 */
static void Gyro_Read(uint8_t reg, uint8_t* rx_buf){
    uint8_t tx_buf[9] = {0};
    tx_buf[0] = reg | 0x80;
    Spi_TransmitReceive(bmi088->gyro, tx_buf, rx_buf, 9, 1000);
}

/**
 * @brief 单个写入加速度计的寄存器
 * @param reg  寄存器地址
 * @param data 写入的数据
 */
static void Accel_Write_Single_Reg(uint8_t reg , uint8_t data){
    uint8_t tx_buf[2] = {reg , data};
    Spi_Transmit(bmi088->accel,tx_buf,2);
}

/**
 * @brief 单个写入陀螺仪的寄存器
 * @param reg   寄存器地址
 * @param data  写入的数据
 */
static void Gyro_Write_Single_Reg(uint8_t reg , uint8_t data){
    uint8_t tx_buf[2] = {reg, data};
    Spi_Transmit(bmi088->gyro,tx_buf,2);
}

static uint8_t Accel_Read_Single_Reg(uint8_t reg){
    uint8_t tx_buf[3] = {0};
    tx_buf[0] = reg| 0x80;
    uint8_t rx_buf[3] = {0};
    Spi_TransmitReceive(bmi088->accel,tx_buf,rx_buf,3,1000);
    return   rx_buf[2];
}

static uint8_t Gyro_Read_Single_Reg(uint8_t reg){
    uint8_t tx_buf[3] = {0};
    uint8_t rx_buf[3] = {0};
    tx_buf[0] = reg| 0x80;
    Spi_TransmitReceive(bmi088->gyro,tx_buf,rx_buf,4,1000);
    return rx_buf[1];
}


/**
************************************************************************
* @brief:      	write_BMI088_accel_reg_data_error_init(void)
* @param:       void
* @retval:     	void
* @details:    	BMI088加速度传感器寄存器数据写入错误处理初始化
************************************************************************
**/
static uint8_t write_BMI088_accel_reg_data_error[BMI088_WRITE_ACCEL_REG_NUM][3] =
{
    {BMI088_ACC_PWR_CTRL, BMI088_ACC_ENABLE_ACC_ON, BMI088_ACC_PWR_CTRL_ERROR},
    {BMI088_ACC_PWR_CONF, BMI088_ACC_PWR_ACTIVE_MODE, BMI088_ACC_PWR_CONF_ERROR},
    {BMI088_ACC_CONF, BMI088_ACC_NORMAL | BMI088_ACC_800_HZ | BMI088_ACC_CONF_MUST_Set, BMI088_ACC_CONF_ERROR},
    {BMI088_ACC_RANGE, BMI088_ACC_RANGE_3G, BMI088_ACC_RANGE_ERROR},
    {
        BMI088_INT1_IO_CTRL, BMI088_ACC_INT1_IO_ENABLE | BMI088_ACC_INT1_GPIO_PP | BMI088_ACC_INT1_GPIO_LOW,
        BMI088_INT1_IO_CTRL_ERROR
    },
    {BMI088_INT_MAP_DATA, BMI088_ACC_INT1_DRDY_INTERRUPT, BMI088_INT_MAP_DATA_ERROR}

};
/**
************************************************************************
* @brief:      	write_BMI088_gyro_reg_data_error_init(void)
* @param:       void
* @retval:     	void
* @details:    	BMI088陀螺仪传感器寄存器数据写入错误处理初始化
************************************************************************
**/
static uint8_t write_BMI088_gyro_reg_data_error[BMI088_WRITE_GYRO_REG_NUM][3] =
{
    {BMI088_GYRO_RANGE, BMI088_GYRO_2000, BMI088_GYRO_RANGE_ERROR},
    {BMI088_GYRO_BANDWIDTH, BMI088_GYRO_1000_116_HZ | BMI088_GYRO_BANDWIDTH_MUST_Set, BMI088_GYRO_BANDWIDTH_ERROR},
    {BMI088_GYRO_LPM1, BMI088_GYRO_NORMAL_MODE, BMI088_GYRO_LPM1_ERROR},
    {BMI088_GYRO_CTRL, BMI088_DRDY_ON, BMI088_GYRO_CTRL_ERROR},
    {
        BMI088_GYRO_INT3_INT4_IO_CONF, BMI088_GYRO_INT3_GPIO_PP | BMI088_GYRO_INT3_GPIO_LOW,
        BMI088_GYRO_INT3_INT4_IO_CONF_ERROR
    },
    {BMI088_GYRO_INT3_INT4_IO_MAP, BMI088_GYRO_DRDY_IO_INT3, BMI088_GYRO_INT3_INT4_IO_MAP_ERROR}

};

uint8_t BMI088_init(void){
    uint8_t error = BMI088_NO_ERROR;
    bmi088 = Bmi088_Register(&bmi088_config);
    error |= bmi088_accel_init();
    error |= bmi088_gyro_init();

    return error;
}

uint8_t bmi088_accel_init(void){
    uint8_t write_reg_num = 0;
    uint8_t res = 0;

    res = Accel_Read_Single_Reg(BMI088_ACC_CHIP_ID);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    res = Accel_Read_Single_Reg(BMI088_ACC_CHIP_ID);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    Accel_Write_Single_Reg(BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_VALUE);
    BMI088_delay_ms(BMI088_LONG_DELAY_TIME);

    res = Accel_Read_Single_Reg(BMI088_ACC_CHIP_ID);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    res = Accel_Read_Single_Reg(BMI088_ACC_CHIP_ID);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    if (res != BMI088_ACC_CHIP_ID_VALUE){
        return BMI088_NO_SENSOR;
    }

    for (write_reg_num = 0; write_reg_num < BMI088_WRITE_ACCEL_REG_NUM; write_reg_num++){

        Accel_Write_Single_Reg(write_BMI088_accel_reg_data_error[write_reg_num][0],
                          write_BMI088_accel_reg_data_error[write_reg_num][1]);
        BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        res = Accel_Read_Single_Reg(write_BMI088_accel_reg_data_error[write_reg_num][0]);
        BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        if (res != write_BMI088_accel_reg_data_error[write_reg_num][1]){
            return write_BMI088_accel_reg_data_error[write_reg_num][2];
        }
    }
    return BMI088_NO_ERROR;
}

uint8_t bmi088_gyro_init(void){
    uint8_t write_reg_num = 0;
    uint8_t res = 0;

    res = Gyro_Read_Single_Reg(BMI088_GYRO_CHIP_ID);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    res = Gyro_Read_Single_Reg(BMI088_GYRO_CHIP_ID);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    Gyro_Write_Single_Reg(BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET_VALUE);
    BMI088_delay_ms(BMI088_LONG_DELAY_TIME);

    res = Gyro_Read_Single_Reg(BMI088_GYRO_CHIP_ID);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    res = Gyro_Read_Single_Reg(BMI088_GYRO_CHIP_ID);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);


    if (res != BMI088_GYRO_CHIP_ID_VALUE){
        return BMI088_NO_SENSOR;
    }


    for (write_reg_num = 0; write_reg_num < BMI088_WRITE_GYRO_REG_NUM; write_reg_num++){
        Gyro_Write_Single_Reg(write_BMI088_gyro_reg_data_error[write_reg_num][0],
                                     write_BMI088_gyro_reg_data_error[write_reg_num][1]);
        BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        res = Gyro_Read_Single_Reg(write_BMI088_gyro_reg_data_error[write_reg_num][0]);
        BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        if (res != write_BMI088_gyro_reg_data_error[write_reg_num][1]){
            return write_BMI088_gyro_reg_data_error[write_reg_num][2];
        }
    }

    return BMI088_NO_ERROR;
}

/**
 * @brief 读取BMI088 陀螺仪，加速度计，温度的值
 * @param gyro 陀螺仪的值
 * @param accel 加速度计的值
 * @param temperate 温度
 */
void Bmi088_read(float gyro[3], float accel[3], float* temperate){
    uint8_t accel_buf[8] = {0};
    uint8_t gyro_buf[9] = {0};
    Accel_Read(BMI088_ACCEL_XOUT_L, accel_buf,6);
    for (uint8_t i = 0; i < 3; i++){
        accel[i] = BMI088_ACCEL_SEN * (float)(int16_t)(accel_buf[i * 2 + 3] << 8 | accel_buf[i * 2 + 2]);
    }
    Gyro_Read(BMI088_GYRO_CHIP_ID, gyro_buf);
    if (gyro_buf[1] == BMI088_GYRO_CHIP_ID_VALUE){
        for (uint8_t i = 0; i < 3; i++){
            gyro[i] = BMI088_GYRO_SEN * (float)(int16_t)(gyro_buf[i * 2 + 4] << 8 | gyro_buf[i * 2 + 3]);
        }
    }
 Accel_Read(BMI088_TEMP_M, accel_buf,2);
 *temperate = (float)(int16_t)((accel_buf[2] << 3) | (accel_buf[3] >> 5)) * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;
}
