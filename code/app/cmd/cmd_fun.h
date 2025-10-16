#ifndef CMD_FUN_H
#define CMD_FUN_H
#include "basic_math.h"


typedef enum{
    CMD_DISABLE = 0,
    CMD_ENABLE = 1,
    CMD_SNIP = 2
}CmdState_e;
typedef enum{
    CHASSIS_NORMAL= 0,
    CHASSIS_FOLLOWING = 1,
    CHASSIS_GYRO = 2
}ChassisState_e;
typedef enum{
    GIMBAL_NORMAL = 0,
    GIMBAL_LOCKING = 1,
    GIMBAL_AIMING = 2
}GimbalState_e;

typedef struct{
    CmdState_e cmd_state;
    ChassisState_e chassis_state;
    GimbalState_e gimbal_state;
}RcState_s;
typedef struct{
    float x_val;
    float y_val;
    float pitch_pos;
    float yaw_pos;
    float fine_pitch_pos;
    float fine_yaw_pos;
}RcOutData_s;

typedef struct{
    RcState_s state;
    RcOutData_s data;
}RcCmd_s;

void USER_CMD_Init(void);
#endif //CMD_FUN_H