#ifndef BMI088_H
#define BMI088_H
#include "spi.h"
#include "bsp_spi.h"
#define BMI088_USING_SPI_UNIT hspi2

typedef struct{
    SpiInstance_s *accel_instance;
    SpiInstance_s *gyro_instance;
}Bmi088Instance_s;
#endif //BMI088_H