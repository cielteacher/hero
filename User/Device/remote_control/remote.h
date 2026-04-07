#ifndef DR16_H
#define DR16_H

#include "stdint.h"
#include "stdbool.h"
#include <stddef.h>
#include <string.h>
#include "RMQueue.h"
#include "bsp_usart.h"
#include "math.h"
#include "usart.h"

/**@brief 去除遥控器摇杆死区 */
#define deadline_limit(value, deadline)                 \
    do {                                                 \
        if ((value) <= (deadline) &&                    \
            (value) >= -(deadline))                     \
        {                                                \
            (value) = 0;                                \
        }                                                \
    } while (0)

// 新遥控器SBUS数据长度
#define SBUS_RX_BUF_NUM 21u

// 摇杆/开关定义
#define RC_CH_VALUE_MIN ((uint16_t)364)
#define RC_CH_VALUE_OFFSET ((uint16_t)1024)
#define RC_CH_VALUE_MAX ((uint16_t)1684)
#define MOUSE_MAX 32767

// 按键定义
#define KEY_PRESS 0
#define KEY_PRESS_WITH_CTRL 1
#define KEY_PRESS_WITH_SHIFT 2

// 新遥控器模式定义 (mode_sw)
#define RC_MODE_STOP       0    // 停止模式
#define RC_MODE_REMOTE     1    // 遥控器模式
#define RC_MODE_KEY_MOUSE  2    // 键鼠模式

// 输入模式枚举
typedef enum {
    STOP_INPUT = 0,          // 停止模式
    REMOTE_INPUT = 1,        // 遥控器控制
    KEY_MOUSE_INPUT = 2,     // 键鼠控制
} InputMode_e;

// 键位掩码定义
#define Key_W     (1U<<0)
#define Key_S     (1U<<1)
#define Key_D     (1U<<2)
#define Key_A     (1U<<3)
#define Key_Shift (1U<<4)
#define Key_Ctrl  (1U<<5)
#define Key_Q     (1U<<6)
#define Key_E     (1U<<7)
#define Key_R     (1U<<8)
#define Key_F     (1U<<9)
#define Key_G     (1U<<10)
#define Key_Z     (1U<<11)
#define Key_X     (1U<<12)
#define Key_C     (1U<<13)
#define Key_V     (1U<<14)
#define Key_B     (1U<<15)

// 键位索引定义
#define key_W     0
#define key_S     1
#define key_D     2
#define key_A     3
#define key_Shift 4
#define key_Ctrl  5
#define key_Q     6
#define key_E     7
#define key_R     8
#define key_F     9
#define key_G     10
#define key_Z     11
#define key_X     12
#define key_C     13
#define key_V     14
#define key_B     15

// 键盘状态联合体
typedef union {
    uint16_t all_keys;
    struct {
        uint16_t W : 1;
        uint16_t S : 1;
        uint16_t D : 1;
        uint16_t A : 1;
        uint16_t Shift : 1;
        uint16_t Ctrl : 1;
        uint16_t Q : 1;
        uint16_t E : 1;
        uint16_t R : 1;
        uint16_t F : 1;
        uint16_t G : 1;
        uint16_t Z : 1;
        uint16_t X : 1;
        uint16_t C : 1;
        uint16_t V : 1;
        uint16_t B : 1;
    } bits;
} KeyState;

// 新遥控器数据结构
typedef struct {
    // 摇杆通道 (0-3)
    volatile uint16_t ch0;
    volatile uint16_t ch1;
    volatile uint16_t ch2;
    volatile uint16_t ch3;
    
    // 新遥控器特有字段
    volatile uint8_t mode_sw;    // 模式开关 0:停止 1:遥控器 2:键鼠
    volatile uint8_t pause;      // 暂停键
    volatile uint8_t fn_1;       // 功能键1
    volatile uint8_t fn_2;       // 功能键2
    volatile uint16_t wheel;     // 滚轮 (11位)
    volatile uint8_t trigger;    // 扳机键
    
    // 鼠标数据
    volatile int16_t x;
    volatile int16_t y;
    volatile int16_t z;
    volatile uint8_t press_l;
    volatile uint8_t press_r;
    volatile uint8_t press_m;
    volatile uint8_t last_mouse_press_l;
    volatile uint8_t last_mouse_press_r;
    
    // 键盘数据
    volatile KeyState last_keys;
    volatile KeyState keys;
    volatile uint8_t key_count[3][16];
    
    // 长按标志
    bool long_press_l;
    bool long_press_r;
    
    // 鼠标速度转位置增量
    volatile float Postion_x;
    volatile float Postion_y;
    volatile float Postion_z;
    
    // 归一化摇杆数据
    volatile float Normalize_ch0;
    volatile float Normalize_ch1;
    volatile float Normalize_ch2;
    volatile float Normalize_ch3;
    
    // 归一化滚轮数据 (-1.0 ~ 1.0)
    volatile float Normalize_wheel;
    
    // 当前输入模式
    InputMode_e input_mode;
} Ctrl_data;

// 遥控器实例结构体
typedef struct {
    volatile Ctrl_data control_data;
    USARTInstance *USART_instance;
    volatile uint32_t dr16_last_rx_ms;
    volatile uint8_t dr16_data_error_flag;
    volatile uint8_t dr16_online_flag;
    uint16_t crc16;  // CRC校验
} DR16_instance_t;

extern DR16_instance_t DR16_instance;

void DR16_Init(void);
void DR16_AliveCheck(void);

#endif

