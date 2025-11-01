/**
 * @file bmi088reg.h
 * @brief BMI088 6轴IMU传感器寄存器定义文件
 * @details 包含BMI088加速度计和陀螺仪的所有寄存器地址、配置参数、
 *          数据结构和错误代码定义
 * @author Embedded Framework Team
 * @date 2025
 * @version 1.0
 */

#ifndef BMI088REG_H
#define BMI088REG_H

/* ========================= 加速度计寄存器定义 ========================= */

/** @defgroup BMI088_ACCEL_REGISTERS 加速度计寄存器
 * @{
 */

#define BMI088_ACC_CHIP_ID 0x00         /**< 加速度计芯片ID寄存器 - "Who am I" */
#define BMI088_ACC_CHIP_ID_VALUE 0x1E   /**< 加速度计芯片ID值 */

#define BMI088_ACC_ERR_REG 0x02                     /**< 加速度计错误寄存器 */
#define BMI088_ACCEL_CONGIF_ERROR_SHFITS 0x2        /**< 加速度计配置错误位偏移 */
#define BMI088_ACCEL_CONGIF_ERROR (1 << BMI088_ACCEL_CONGIF_ERROR_SHFITS)  /**< 加速度计配置错误标志 */
#define BMI088_FATAL_ERROR_SHFITS 0x0               /**< 致命错误位偏移 */
#define BMI088_FATAL_ERROR (1 << BMI088_FATAL_ERROR) /**< 致命错误标志 */

#define BMI088_ACC_STATUS 0x03                      /**< 加速度计状态寄存器 */
#define BMI088_ACCEL_DRDY_SHFITS 0x7                /**< 加速度计数据就绪位偏移 */
#define BMI088_ACCEL_DRDY (1 << BMI088_ACCEL_DRDY_SHFITS)  /**< 加速度计数据就绪标志 */

/** @defgroup BMI088_ACCEL_DATA_REGISTERS 加速度计数据寄存器
 * @{
 */
#define BMI088_ACCEL_XOUT_L 0x12                    /**< X轴加速度数据低字节 */
#define BMI088_ACCEL_XOUT_M 0x13                    /**< X轴加速度数据高字节 */
#define BMI088_ACCEL_YOUT_L 0x14                    /**< Y轴加速度数据低字节 */
#define BMI088_ACCEL_YOUT_M 0x15                    /**< Y轴加速度数据高字节 */
#define BMI088_ACCEL_ZOUT_L 0x16                    /**< Z轴加速度数据低字节 */
#define BMI088_ACCEL_ZOUT_M 0x17                    /**< Z轴加速度数据高字节 */
/** @} */

/** @defgroup BMI088_SENSOR_TIME_REGISTERS 传感器时间寄存器
 * @{
 */
#define BMI088_SENSORTIME_DATA_L 0x18               /**< 传感器时间数据低字节 */
#define BMI088_SENSORTIME_DATA_M 0x19               /**< 传感器时间数据中字节 */
#define BMI088_SENSORTIME_DATA_H 0x1A               /**< 传感器时间数据高字节 */
/** @} */

#define BMI088_ACC_INT_STAT_1 0x1D                  /**< 加速度计中断状态寄存器1 */
#define BMI088_ACCEL_DRDY_INTERRUPT_SHFITS 0x7      /**< 加速度计数据就绪中断位偏移 */
#define BMI088_ACCEL_DRDY_INTERRUPT (1 << BMI088_ACCEL_DRDY_INTERRUPT_SHFITS)  /**< 加速度计数据就绪中断标志 */

/** @defgroup BMI088_TEMPERATURE_REGISTERS 温度寄存器
 * @{
 */
#define BMI088_TEMP_M 0x22                          /**< 温度数据高字节 */
#define BMI088_TEMP_L 0x23                          /**< 温度数据低字节 */
/** @} */

/** @defgroup BMI088_ACCEL_CONFIG_REGISTERS 加速度计配置寄存器
 * @{
 */
#define BMI088_ACC_CONF 0x40                        /**< 加速度计配置寄存器 */
#define BMI088_ACC_CONF_MUST_Set 0x80               /**< 加速度计配置必须设置位 */
#define BMI088_ACC_BWP_SHFITS 0x4                   /**< 加速度计带宽参数位偏移 */
#define BMI088_ACC_OSR4 (0x0 << BMI088_ACC_BWP_SHFITS)      /**< 加速度计OSR4模式 */
#define BMI088_ACC_OSR2 (0x1 << BMI088_ACC_BWP_SHFITS)      /**< 加速度计OSR2模式 */
#define BMI088_ACC_NORMAL (0x2 << BMI088_ACC_BWP_SHFITS)    /**< 加速度计正常模式 */

/** @defgroup BMI088_ACCEL_ODR 加速度计输出数据率配置
 * @{
 */
#define BMI088_ACC_ODR_SHFITS 0x0                   /**< 加速度计ODR位偏移 */
#define BMI088_ACC_12_5_HZ (0x5 << BMI088_ACC_ODR_SHFITS)   /**< 12.5Hz输出数据率 */
#define BMI088_ACC_25_HZ (0x6 << BMI088_ACC_ODR_SHFITS)     /**< 25Hz输出数据率 */
#define BMI088_ACC_50_HZ (0x7 << BMI088_ACC_ODR_SHFITS)     /**< 50Hz输出数据率 */
#define BMI088_ACC_100_HZ (0x8 << BMI088_ACC_ODR_SHFITS)    /**< 100Hz输出数据率 */
#define BMI088_ACC_200_HZ (0x9 << BMI088_ACC_ODR_SHFITS)    /**< 200Hz输出数据率 */
#define BMI088_ACC_400_HZ (0xA << BMI088_ACC_ODR_SHFITS)    /**< 400Hz输出数据率 */
#define BMI088_ACC_800_HZ (0xB << BMI088_ACC_ODR_SHFITS)    /**< 800Hz输出数据率 */
#define BMI088_ACC_1600_HZ (0xC << BMI088_ACC_ODR_SHFITS)   /**< 1600Hz输出数据率 */
/** @} */

#define BMI088_ACC_RANGE 0x41                       /**< 加速度计量程寄存器 */

/** @defgroup BMI088_ACCEL_RANGE 加速度计量程配置
 * @{
 */
#define BMI088_ACC_RANGE_SHFITS 0x0                 /**< 加速度计量程位偏移 */
#define BMI088_ACC_RANGE_3G (0x0 << BMI088_ACC_RANGE_SHFITS)    /**< ±3g量程 */
#define BMI088_ACC_RANGE_6G (0x1 << BMI088_ACC_RANGE_SHFITS)    /**< ±6g量程 */
#define BMI088_ACC_RANGE_12G (0x2 << BMI088_ACC_RANGE_SHFITS)   /**< ±12g量程 */
#define BMI088_ACC_RANGE_24G (0x3 << BMI088_ACC_RANGE_SHFITS)   /**< ±24g量程 */
/** @} */

/** @defgroup BMI088_ACCEL_INT1_CONFIG 加速度计INT1中断配置
 * @{
 */
#define BMI088_INT1_IO_CTRL 0x53                    /**< INT1中断IO控制寄存器 */
#define BMI088_ACC_INT1_IO_ENABLE_SHFITS 0x3        /**< INT1 IO使能位偏移 */
#define BMI088_ACC_INT1_IO_ENABLE (0x1 << BMI088_ACC_INT1_IO_ENABLE_SHFITS)  /**< INT1 IO使能 */
#define BMI088_ACC_INT1_GPIO_MODE_SHFITS 0x2        /**< INT1 GPIO模式位偏移 */
#define BMI088_ACC_INT1_GPIO_PP (0x0 << BMI088_ACC_INT1_GPIO_MODE_SHFITS)    /**< INT1推挑输出 */
#define BMI088_ACC_INT1_GPIO_OD (0x1 << BMI088_ACC_INT1_GPIO_MODE_SHFITS)    /**< INT1开漏输出 */
#define BMI088_ACC_INT1_GPIO_LVL_SHFITS 0x1         /**< INT1 GPIO电平位偏移 */
#define BMI088_ACC_INT1_GPIO_LOW (0x0 << BMI088_ACC_INT1_GPIO_LVL_SHFITS)    /**< INT1低电平有效 */
#define BMI088_ACC_INT1_GPIO_HIGH (0x1 << BMI088_ACC_INT1_GPIO_LVL_SHFITS)   /**< INT1高电平有效 */
/** @} */

/** @defgroup BMI088_ACCEL_INT2_CONFIG 加速度计INT2中断配置
 * @{
 */
#define BMI088_INT2_IO_CTRL 0x54                    /**< INT2中断IO控制寄存器 */
#define BMI088_ACC_INT2_IO_ENABLE_SHFITS 0x3        /**< INT2 IO使能位偏移 */
#define BMI088_ACC_INT2_IO_ENABLE (0x1 << BMI088_ACC_INT2_IO_ENABLE_SHFITS)  /**< INT2 IO使能 */
#define BMI088_ACC_INT2_GPIO_MODE_SHFITS 0x2        /**< INT2 GPIO模式位偏移 */
#define BMI088_ACC_INT2_GPIO_PP (0x0 << BMI088_ACC_INT2_GPIO_MODE_SHFITS)    /**< INT2推挑输出 */
#define BMI088_ACC_INT2_GPIO_OD (0x1 << BMI088_ACC_INT2_GPIO_MODE_SHFITS)    /**< INT2开漏输出 */
#define BMI088_ACC_INT2_GPIO_LVL_SHFITS 0x1         /**< INT2 GPIO电平位偏移 */
#define BMI088_ACC_INT2_GPIO_LOW (0x0 << BMI088_ACC_INT2_GPIO_LVL_SHFITS)    /**< INT2低电平有效 */
#define BMI088_ACC_INT2_GPIO_HIGH (0x1 << BMI088_ACC_INT2_GPIO_LVL_SHFITS)   /**< INT2高电平有效 */
/** @} */

#define BMI088_INT_MAP_DATA 0x58                    /**< 中断映射数据寄存器 */
#define BMI088_ACC_INT2_DRDY_INTERRUPT_SHFITS 0x6   /**< INT2数据就绪中断位偏移 */
#define BMI088_ACC_INT2_DRDY_INTERRUPT (0x1 << BMI088_ACC_INT2_DRDY_INTERRUPT_SHFITS)  /**< INT2数据就绪中断 */
#define BMI088_ACC_INT1_DRDY_INTERRUPT_SHFITS 0x2   /**< INT1数据就绪中断位偏移 */
#define BMI088_ACC_INT1_DRDY_INTERRUPT (0x1 << BMI088_ACC_INT1_DRDY_INTERRUPT_SHFITS)  /**< INT1数据就绪中断 */

/** @defgroup BMI088_ACCEL_SELF_TEST 加速度计自测试
 * @{
 */
#define BMI088_ACC_SELF_TEST 0x6D                   /**< 加速度计自测试寄存器 */
#define BMI088_ACC_SELF_TEST_OFF 0x00               /**< 关闭自测试 */
#define BMI088_ACC_SELF_TEST_POSITIVE_SIGNAL 0x0D   /**< 正信号自测试 */
#define BMI088_ACC_SELF_TEST_NEGATIVE_SIGNAL 0x09   /**< 负信号自测试 */
/** @} */

/** @defgroup BMI088_ACCEL_POWER_CONFIG 加速度计电源管理
 * @{
 */
#define BMI088_ACC_PWR_CONF 0x7C                    /**< 加速度计电源配置寄存器 */
#define BMI088_ACC_PWR_SUSPEND_MODE 0x03            /**< 加速度计挂起模式 */
#define BMI088_ACC_PWR_ACTIVE_MODE 0x00             /**< 加速度计活跃模式 */

#define BMI088_ACC_PWR_CTRL 0x7D                    /**< 加速度计电源控制寄存器 */
#define BMI088_ACC_ENABLE_ACC_OFF 0x00              /**< 关闭加速度计 */
#define BMI088_ACC_ENABLE_ACC_ON 0x04               /**< 开启加速度计 */

#define BMI088_ACC_SOFTRESET 0x7E                   /**< 加速度计软复位寄存器 */
#define BMI088_ACC_SOFTRESET_VALUE 0xB6             /**< 加速度计软复位值 */
/** @} */
/** @} */

/* ========================= 陀螺仪寄存器定义 ========================= */

/** @defgroup BMI088_GYRO_REGISTERS 陀螺仪寄存器
 * @{
 */

#define BMI088_GYRO_CHIP_ID 0x00                    /**< 陀螺仪芯片ID寄存器 - "Who am I" */
#define BMI088_GYRO_CHIP_ID_VALUE 0x0F              /**< 陀螺仪芯片ID值 */

/** @defgroup BMI088_GYRO_DATA_REGISTERS 陀螺仪数据寄存器
 * @{
 */
#define BMI088_GYRO_X_L 0x02                        /**< X轴陀螺仪数据低字节 */
#define BMI088_GYRO_X_H 0x03                        /**< X轴陀螺仪数据高字节 */
#define BMI088_GYRO_Y_L 0x04                        /**< Y轴陀螺仪数据低字节 */
#define BMI088_GYRO_Y_H 0x05                        /**< Y轴陀螺仪数据高字节 */
#define BMI088_GYRO_Z_L 0x06                        /**< Z轴陀螺仪数据低字节 */
#define BMI088_GYRO_Z_H 0x07                        /**< Z轴陀螺仪数据高字节 */
/** @} */

#define BMI088_GYRO_INT_STAT_1 0x0A                 /**< 陀螺仪中断状态寄存器1 */
#define BMI088_GYRO_DYDR_SHFITS 0x7                 /**< 陀螺仪数据就绪位偏移 */
#define BMI088_GYRO_DYDR (0x1 << BMI088_GYRO_DYDR_SHFITS)  /**< 陀螺仪数据就绪标志 */

/** @defgroup BMI088_GYRO_RANGE_CONFIG 陀螺仪量程配置
 * @{
 */
#define BMI088_GYRO_RANGE 0x0F                      /**< 陀螺仪量程寄存器 */
#define BMI088_GYRO_RANGE_SHFITS 0x0                /**< 陀螺仪量程位偏移 */
#define BMI088_GYRO_2000 (0x0 << BMI088_GYRO_RANGE_SHFITS)  /**< ±2000°/s量程 */
#define BMI088_GYRO_1000 (0x1 << BMI088_GYRO_RANGE_SHFITS)  /**< ±1000°/s量程 */
#define BMI088_GYRO_500 (0x2 << BMI088_GYRO_RANGE_SHFITS)   /**< ±500°/s量程 */
#define BMI088_GYRO_250 (0x3 << BMI088_GYRO_RANGE_SHFITS)   /**< ±250°/s量程 */
#define BMI088_GYRO_125 (0x4 << BMI088_GYRO_RANGE_SHFITS)   /**< ±125°/s量程 */
/** @} */

/** @defgroup BMI088_GYRO_BANDWIDTH_CONFIG 陀螺仪带宽配置
 * @brief 第一个数字表示输出数据率，第二个数字表示带宽
 * @{
 */
#define BMI088_GYRO_BANDWIDTH 0x10                  /**< 陀螺仪带宽寄存器 */
#define BMI088_GYRO_BANDWIDTH_MUST_Set 0x80         /**< 陀螺仪带宽必须设置位 */
#define BMI088_GYRO_2000_532_HZ 0x00                /**< 2000Hz ODR, 532Hz带宽 */
#define BMI088_GYRO_2000_230_HZ 0x01                /**< 2000Hz ODR, 230Hz带宽 */
#define BMI088_GYRO_1000_116_HZ 0x02                /**< 1000Hz ODR, 116Hz带宽 */
#define BMI088_GYRO_400_47_HZ 0x03                  /**< 400Hz ODR, 47Hz带宽 */
#define BMI088_GYRO_200_23_HZ 0x04                  /**< 200Hz ODR, 23Hz带宽 */
#define BMI088_GYRO_100_12_HZ 0x05                  /**< 100Hz ODR, 12Hz带宽 */
#define BMI088_GYRO_200_64_HZ 0x06                  /**< 200Hz ODR, 64Hz带宽 */
#define BMI088_GYRO_100_32_HZ 0x07                  /**< 100Hz ODR, 32Hz带宽 */
/** @} */

/** @defgroup BMI088_GYRO_POWER_MODE 陀螺仪电源模式
 * @{
 */
#define BMI088_GYRO_LPM1 0x11                       /**< 陀螺仪低功耗模式1寄存器 */
#define BMI088_GYRO_NORMAL_MODE 0x00                /**< 陀螺仪正常模式 */
#define BMI088_GYRO_SUSPEND_MODE 0x80               /**< 陀螺仪挂起模式 */
#define BMI088_GYRO_DEEP_SUSPEND_MODE 0x20          /**< 陀螺仪深度挂起模式 */
/** @} */

#define BMI088_GYRO_SOFTRESET 0x14                  /**< 陀螺仪软复位寄存器 */
#define BMI088_GYRO_SOFTRESET_VALUE 0xB6            /**< 陀螺仪软复位值 */

#define BMI088_GYRO_CTRL 0x15                       /**< 陀螺仪控制寄存器 */
#define BMI088_DRDY_OFF 0x00                        /**< 关闭数据就绪功能 */
#define BMI088_DRDY_ON 0x80                         /**< 开启数据就绪功能 */

/** @defgroup BMI088_GYRO_INT_CONFIG 陀螺仪中断配置
 * @{
 */
#define BMI088_GYRO_INT3_INT4_IO_CONF 0x16           /**< 陀螺仪INT3/INT4 IO配置寄存器 */
#define BMI088_GYRO_INT4_GPIO_MODE_SHFITS 0x3       /**< INT4 GPIO模式位偏移 */
#define BMI088_GYRO_INT4_GPIO_PP (0x0 << BMI088_GYRO_INT4_GPIO_MODE_SHFITS)  /**< INT4推挑输出 */
#define BMI088_GYRO_INT4_GPIO_OD (0x1 << BMI088_GYRO_INT4_GPIO_MODE_SHFITS)  /**< INT4开漏输出 */
#define BMI088_GYRO_INT4_GPIO_LVL_SHFITS 0x2        /**< INT4 GPIO电平位偏移 */
#define BMI088_GYRO_INT4_GPIO_LOW (0x0 << BMI088_GYRO_INT4_GPIO_LVL_SHFITS)  /**< INT4低电平有效 */
#define BMI088_GYRO_INT4_GPIO_HIGH (0x1 << BMI088_GYRO_INT4_GPIO_LVL_SHFITS) /**< INT4高电平有效 */
#define BMI088_GYRO_INT3_GPIO_MODE_SHFITS 0x1       /**< INT3 GPIO模式位偏移 */
#define BMI088_GYRO_INT3_GPIO_PP (0x0 << BMI088_GYRO_INT3_GPIO_MODE_SHFITS)  /**< INT3推挑输出 */
#define BMI088_GYRO_INT3_GPIO_OD (0x1 << BMI088_GYRO_INT3_GPIO_MODE_SHFITS)  /**< INT3开漏输出 */
#define BMI088_GYRO_INT3_GPIO_LVL_SHFITS 0x0        /**< INT3 GPIO电平位偏移 */
#define BMI088_GYRO_INT3_GPIO_LOW (0x0 << BMI088_GYRO_INT3_GPIO_LVL_SHFITS)  /**< INT3低电平有效 */
#define BMI088_GYRO_INT3_GPIO_HIGH (0x1 << BMI088_GYRO_INT3_GPIO_LVL_SHFITS) /**< INT3高电平有效 */

#define BMI088_GYRO_INT3_INT4_IO_MAP 0x18           /**< 陀螺仪INT3/INT4 IO映射寄存器 */

#define BMI088_GYRO_DRDY_IO_OFF 0x00                /**< 关闭数据就绪中断输出 */
#define BMI088_GYRO_DRDY_IO_INT3 0x01               /**< 数据就绪中断输出到INT3 */
#define BMI088_GYRO_DRDY_IO_INT4 0x80               /**< 数据就绪中断输出到INT4 */
#define BMI088_GYRO_DRDY_IO_BOTH (BMI088_GYRO_DRDY_IO_INT3 | BMI088_GYRO_DRDY_IO_INT4)  /**< 数据就绪中断输出到两个引脚 */
/** @} */

/** @defgroup BMI088_GYRO_SELF_TEST 陀螺仪自测试
 * @{
 */
#define BMI088_GYRO_SELF_TEST 0x3C                  /**< 陀螺仪自测试寄存器 */
#define BMI088_GYRO_RATE_OK_SHFITS 0x4              /**< 陀螺仪速率正常位偏移 */
#define BMI088_GYRO_RATE_OK (0x1 << BMI088_GYRO_RATE_OK_SHFITS)  /**< 陀螺仪速率正常标志 */
#define BMI088_GYRO_BIST_FAIL_SHFITS 0x2            /**< 陀螺仪内置自测试失败位偏移 */
#define BMI088_GYRO_BIST_FAIL (0x1 << BMI088_GYRO_BIST_FAIL_SHFITS)  /**< 陀螺仪内置自测试失败标志 */
#define BMI088_GYRO_BIST_RDY_SHFITS 0x1             /**< 陀螺仪内置自测试就绪位偏移 */
#define BMI088_GYRO_BIST_RDY (0x1 << BMI088_GYRO_BIST_RDY_SHFITS)    /**< 陀螺仪内置自测试就绪标志 */
#define BMI088_GYRO_TRIG_BIST_SHFITS 0x0            /**< 陀螺仪触发内置自测试位偏移 */
#define BMI088_GYRO_TRIG_BIST (0x1 << BMI088_GYRO_TRIG_BIST_SHFITS)  /**< 陀螺仪触发内置自测试标志 */
/** @} */
/** @} */

/* ========================= BMI088通用常量定义 ========================= */

/** @defgroup BMI088_COMMON_CONSTANTS BMI088通用常量
 * @{
 */

/** @defgroup BMI088_TEMPERATURE_CONSTANTS 温度转换常量
 * @{
 */
#define BMI088_TEMP_FACTOR 0.125f                   /**< 温度转换系数 */
#define BMI088_TEMP_OFFSET 23.0f                    /**< 温度偏移值 */
/** @} */

/** @defgroup BMI088_INIT_CONSTANTS 初始化常量
 * @{
 */
#define BMI088_WRITE_ACCEL_REG_NUM 6                /**< 加速度计初始化寄存器数量 */
#define BMI088_WRITE_GYRO_REG_NUM 6                 /**< 陀螺仪初始化寄存器数量 */
/** @} */

/** @defgroup BMI088_DATA_READY_BITS 数据就绪位定义
 * @{
 */
#define BMI088_GYRO_DATA_READY_BIT 0                /**< 陀螺仪数据就绪位 */
#define BMI088_ACCEL_DATA_READY_BIT 1               /**< 加速度计数据就绪位 */
#define BMI088_ACCEL_TEMP_DATA_READY_BIT 2          /**< 加速度计温度数据就绪位 */
/** @} */

/** @defgroup BMI088_TIMING_CONSTANTS 时序常量
 * @{
 */
#define BMI088_LONG_DELAY_TIME 80                   /**< 长延时时间(毫秒) */
#define BMI088_COM_WAIT_SENSOR_TIME 150             /**< 通信等待传感器时间(微秒) */
/** @} */


/** @defgroup BMI088_I2C_ADDRESSES I2C地址定义
 * @{
 */
#define BMI088_ACCEL_IIC_ADDRESSE (0x18 << 1)       /**< 加速度计I2C地址 */
#define BMI088_GYRO_IIC_ADDRESSE (0x68 << 1)        /**< 陀螺仪I2C地址 */
/** @} */

/** @defgroup BMI088_DEFAULT_RANGE_CONFIG 默认量程配置
 * @brief 选择使用的默认量程，只能选择一个
 * @{
 */
#define BMI088_ACCEL_RANGE_3G                       /**< 使用加速度计±3g量程 */
//#define BMI088_ACCEL_RANGE_6G                     /**< 使用加速度计±6g量程 */
//#define BMI088_ACCEL_RANGE_12G                    /**< 使用加速度计±12g量程 */
//#define BMI088_ACCEL_RANGE_24G                    /**< 使用加速度计±24g量程 */

#define BMI088_GYRO_RANGE_2000                      /**< 使用陀螺仪±2000°/s量程 */
//#define BMI088_GYRO_RANGE_1000                    /**< 使用陀螺仪±1000°/s量程 */
//#define BMI088_GYRO_RANGE_500                     /**< 使用陀螺仪±500°/s量程 */
//#define BMI088_GYRO_RANGE_250                     /**< 使用陀螺仪±250°/s量程 */
//#define BMI088_GYRO_RANGE_125                     /**< 使用陀螺仪±125°/s量程 */
/** @} */


/** @defgroup BMI088_ACCEL_SENSITIVITY 加速度计灵敏度系数
 * @brief 不同量程下的加速度计灵敏度系数 (g/LSB)
 * @{
 */
#define BMI088_ACCEL_3G_SEN 0.0008974358974f        /**< ±3g量程灵敏度 */
#define BMI088_ACCEL_6G_SEN 0.00179443359375f       /**< ±6g量程灵敏度 */
#define BMI088_ACCEL_12G_SEN 0.0035888671875f       /**< ±12g量程灵敏度 */
#define BMI088_ACCEL_24G_SEN 0.007177734375f        /**< ±24g量程灵敏度 */
/** @} */


/** @defgroup BMI088_GYRO_SENSITIVITY 陀螺仪灵敏度系数
 * @brief 不同量程下的陀螺仪灵敏度系数 (°/s/LSB)
 * @{
 */
#define BMI088_GYRO_2000_SEN 0.00106526443603169529841533860381f   /**< ±2000°/s量程灵敏度 */
#define BMI088_GYRO_1000_SEN 0.00053263221801584764920766930190693f /**< ±1000°/s量程灵敏度 */
#define BMI088_GYRO_500_SEN 0.00026631610900792382460383465095346f  /**< ±500°/s量程灵敏度 */
#define BMI088_GYRO_250_SEN 0.00013315805450396191230191732547673f  /**< ±250°/s量程灵敏度 */
#define BMI088_GYRO_125_SEN 0.000066579027251980956150958662738366f /**< ±125°/s量程灵敏度 */
/** @} */


/* ========================= BMI088数据结构定义 ========================= */

/** @defgroup BMI088_DATA_STRUCTURES BMI088数据结构
 * @{
 */

/**
 * @brief BMI088原始数据结构体
 * @details 存储从传感器读取的原始数据，未经过校准和单位转换
 */
typedef  struct BMI088_RAW_DATA
{
    uint8_t status;                                 /**< 传感器状态 */
    int16_t accel[3];                               /**< 加速度计原始数据 [X, Y, Z] */
    int16_t temp;                                   /**< 温度原始数据 */
    int16_t gyro[3];                                /**< 陀螺仪原始数据 [X, Y, Z] */
} bmi088_raw_data_t;

/**
 * @brief BMI088实际数据结构体
 * @details 存储经过校准和单位转换后的实际物理量数据
 */
typedef struct BMI088_REAL_DATA
{
    uint8_t status;                                 /**< 传感器状态 */
    float accel[3];                                 /**< 加速度数据 [X, Y, Z] (g) */
    float temp;                                     /**< 温度数据 (℃) */
    float gyro[3];                                  /**< 陀螺仪数据 [X, Y, Z] (°/s) */
    float time;                                     /**< 时间戳 */
} bmi088_real_data_t;
/** @} */


/* ========================= BMI088错误代码定义 ========================= */

/** @defgroup BMI088_ERROR_CODES BMI088错误代码
 * @brief BMI088初始化和操作过程中可能出现的错误代码
 * @{
 */
enum
{
    BMI088_NO_ERROR = 0x00,                        /**< 无错误 */
    
    /* 加速度计错误代码 (0x01-0x07) */
    BMI088_ACC_PWR_CTRL_ERROR = 0x01,              /**< 加速度计电源控制错误 */
    BMI088_ACC_PWR_CONF_ERROR = 0x02,              /**< 加速度计电源配置错误 */
    BMI088_ACC_CONF_ERROR = 0x03,                  /**< 加速度计配置错误 */
    BMI088_ACC_SELF_TEST_ERROR = 0x04,             /**< 加速度计自测试错误 */
    BMI088_ACC_RANGE_ERROR = 0x05,                 /**< 加速度计量程错误 */
    BMI088_INT1_IO_CTRL_ERROR = 0x06,              /**< INT1 IO控制错误 */
    BMI088_INT_MAP_DATA_ERROR = 0x07,              /**< 中断映射数据错误 */
    
    /* 陀螺仪错误代码 (0x08-0x0D) */
    BMI088_GYRO_RANGE_ERROR = 0x08,                /**< 陀螺仪量程错误 */
    BMI088_GYRO_BANDWIDTH_ERROR = 0x09,            /**< 陀螺仪带宽错误 */
    BMI088_GYRO_LPM1_ERROR = 0x0A,                 /**< 陀螺仪低功耗模式1错误 */
    BMI088_GYRO_CTRL_ERROR = 0x0B,                 /**< 陀螺仪控制错误 */
    BMI088_GYRO_INT3_INT4_IO_CONF_ERROR = 0x0C,    /**< 陀螺仪INT3/INT4 IO配置错误 */
    BMI088_GYRO_INT3_INT4_IO_MAP_ERROR = 0x0D,     /**< 陀螺仪INT3/INT4 IO映射错误 */

    /* 自测试错误代码 */
    BMI088_SELF_TEST_ACCEL_ERROR = 0x80,           /**< 加速度计自测试失败 */
    BMI088_SELF_TEST_GYRO_ERROR = 0x40,            /**< 陀螺仪自测试失败 */
    
    /* 传感器不存在 */
    BMI088_NO_SENSOR = 0xFF,                       /**< 未检测到传感器 */
};
/** @} */

/** @} */

#endif  /* BMI088REG_H */
