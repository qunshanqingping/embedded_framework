/**
 * @file module_typedef.h
 * @brief Module type definitions for module library header files to improve reuse rate
 * @author Adonis Jin
 * @date 2025/08/08
 * @version 1.0.0
 */

#ifndef MODULE_TYPEDEF_H
#define MODULE_TYPEDEF_H
#include <stdint.h>
#include <stdbool.h>
/**
 * @brief Key status enumeration
 * @author Adonis Jin
 * @date 2025/08/08
 * @version 1.0.0
 * @note Defines the various states a key can be in
 */
typedef enum
{
    UP,          //!< Key is in up (not pressed) state
    SHORT_DOWN,  //!< Key is briefly pressed
    LONG_DOWN,   //!< Key is held down for an extended period
    PRESS,       //!< Key is currently pressed
    RELEASE      //!< Key has been released
} KeyStatus_e;

/**
 * @brief Key information structure
 * @author Adonis Jin
 * @date 2025/08/08
 * @version 1.0.0
 * @note Contains state information and tracking data for a single key
 */
typedef struct
{
    uint16_t count;             //!< Counter for key press duration
    KeyStatus_e status;         //!< Current key status
    KeyStatus_e last_status;    //!< Previous key status
    bool LAST_KEY_PRESS;        //!< Previous key press state
    bool KEY_PRESS;             //!< Current key press state
} KeyInfo_s;

/**
 * @brief Mouse data structure
 * @author Adonis Jin
 * @date 2025/08/08
 * @version 1.0.0
 * @note Contains mouse movement data and button states
 */
typedef struct
{
    int16_t mouse_x;            //!< Mouse X-axis movement
    int16_t mouse_y;            //!< Mouse Y-axis movement
    int16_t mouse_wheel;        //!< Mouse wheel movement
    KeyInfo_s left_button;      //!< Left mouse button information
    KeyInfo_s right_button;     //!< Right mouse button information
} Mouse_s;

/**
 * @brief Keyboard data structure
 * @author Adonis Jin
 * @date 2025/08/08
 * @version 1.0.0
 * @note Contains information for all supported keyboard keys
 */
typedef struct
{
    KeyInfo_s W;        //!< W key information
    KeyInfo_s S;        //!< S key information
    KeyInfo_s A;        //!< A key information
    KeyInfo_s D;        //!< D key information
    KeyInfo_s SHIFT;    //!< SHIFT key information
    KeyInfo_s CTRL;     //!< CTRL key information
    KeyInfo_s Q;        //!< Q key information
    KeyInfo_s E;        //!< E key information
    KeyInfo_s R;        //!< R key information
    KeyInfo_s F;        //!< F key information
    KeyInfo_s G;        //!< G key information
    KeyInfo_s Z;        //!< Z key information
    KeyInfo_s X;        //!< X key information
    KeyInfo_s C;        //!< C key information
    KeyInfo_s V;        //!< V key information
    KeyInfo_s B;        //!< B key information
} Keyboard_s;

/**
 * @brief Combined keyboard and mouse operation structure
 * @author Adonis Jin
 * @date 2025/08/08
 * @version 1.0.0
 * @note Container for all keyboard and mouse input data
 */
typedef struct
{
    Mouse_s mouse;
    Keyboard_s keyboard;
} KeyboardMouseOperation_s;

/**
 * @brief 频率统计结构体
 * @note 用于统计1秒内的事件次数和计算频率
 */
typedef struct{
    uint16_t cnt_1s;    // 1秒内事件计数
    uint16_t frequency; // 保存得到的频率
}Frequency_s;
#endif //MODULE_TYPEDEF_H
