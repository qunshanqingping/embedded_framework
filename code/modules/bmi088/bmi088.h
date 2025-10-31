#ifndef BMI088_H
#define BMI088_H

#include "stdint.h"
#include  "bsp_spi.h"



typedef struct{

    SpiInstance_s *accel;
    SpiInstance_s *gyro;
}Bmi088Instance_s;
typedef struct{
    SpiInitConfig_s accel;
    SpiInitConfig_s gyro;
}Bmi088InitConfig_s;


 uint8_t BMI088_init(void);

 void Bmi088_read(float gyro[3], float accel[3], float *temperate);



#endif
