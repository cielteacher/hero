#include "remote.h"
#include "bsp_dwt.h"
#include <string.h>
#include <stdint.h>

DR16_instance_t DR16_instance = {0};

USART_Init_Config_s DR16_config = {
    .transmit_mode = USART_TRANSFER_DMA,
    .receive_mode = USART_RECEIVE_IDLE_DMA,
    .usart_handle = &huart5,
    .data_len = SBUS_RX_BUF_NUM,  // 新协议21字节
    .temp_depth = 5,
};

/* ------------------ 内部工具函数 ------------------*/
// 通道归一化 (-1.0 ~ 1.0)
static float RC_Channel_Normalize(uint16_t raw_ch);
// 回调函数
static void DecodeDR16(USARTInstance *_instance);
/**@brief 新遥控器数据乱码检测 */
static uint8_t REMOTE_IfDataError(void);

// ------------------ 初始化 ------------------
void DR16_Init(void) 
{
    memset(&DR16_instance, 0, sizeof(DR16_instance_t));
    DR16_instance.dr16_data_error_flag = 0;
    DR16_instance.dr16_online_flag = 0;
    DR16_instance.dr16_last_rx_ms = DWT_GetTime_ms();
    
    DR16_instance.USART_instance = USARTRegister(&DR16_config);
    if (DR16_instance.USART_instance != NULL)
    {
        DR16_instance.USART_instance->uart_module_callback = DecodeDR16;
        DR16_instance.USART_instance->id = &DR16_instance;
    }
    else
    {
        DR16_instance.dr16_data_error_flag = 1;
        DR16_instance.dr16_online_flag = 0;
    }
    
    memset((void*)&DR16_instance.control_data, 0, sizeof(Ctrl_data));
}
/**
 * @brief 解析新遥控器SBUS数据 (21字节协议)
 * @note 帧头 0xA9 0x53
 */
static void SBUS_TO_RC(uint8_t *sbus_buf)
{
    if (sbus_buf == NULL) return;
    
    // 帧头检验
    if (sbus_buf[0] != 0xA9 || sbus_buf[1] != 0x53) return;
    
    DR16_instance_t *instance = &DR16_instance;
    int32_t control_time = DWT_GetTime_ms() - instance->dr16_last_rx_ms;
    instance->dr16_last_rx_ms = DWT_GetTime_ms();
    
    // 保存上一轮状态
    instance->control_data.last_mouse_press_l = instance->control_data.press_l;
    instance->control_data.last_mouse_press_r = instance->control_data.press_r;
    instance->control_data.last_keys = instance->control_data.keys;
    
    /* Channel 0, 1, 2, 3 (11bit each) */
    instance->control_data.ch0 = (sbus_buf[2] | (sbus_buf[3] << 8)) & 0x07FF;
    instance->control_data.ch1 = ((sbus_buf[3] >> 3) | (sbus_buf[4] << 5)) & 0x07FF;
    instance->control_data.ch2 = ((sbus_buf[4] >> 6) | (sbus_buf[5] << 2) | (sbus_buf[6] << 10)) & 0x07FF;
    instance->control_data.ch3 = ((sbus_buf[6] >> 1) | (sbus_buf[7] << 7)) & 0x07FF;
    
    /* 新遥控器特有字段 */
    instance->control_data.mode_sw  = (sbus_buf[7] >> 4) & 0x03;    // 2bit mode switch
    instance->control_data.pause    = (sbus_buf[7] >> 6) & 0x01;    // 1bit pause
    instance->control_data.fn_1     = (sbus_buf[7] >> 7) & 0x01;    // 1bit fn1
    instance->control_data.fn_2     = sbus_buf[8] & 0x01;           // 1bit fn2
    instance->control_data.wheel    = ((sbus_buf[8] >> 1) | (sbus_buf[9] << 7)) & 0x07FF;  // 11bit wheel
    instance->control_data.trigger  = (sbus_buf[9] >> 4) & 0x01;    // 1bit trigger
    
    /* Mouse axis: X, Y, Z */
    instance->control_data.x = (int16_t)(sbus_buf[10] | (sbus_buf[11] << 8));
    instance->control_data.y = (int16_t)(sbus_buf[12] | (sbus_buf[13] << 8));
    instance->control_data.z = (int16_t)(sbus_buf[14] | (sbus_buf[15] << 8));
    
    /* Mouse buttons */
    instance->control_data.press_l = sbus_buf[16] & 0x03;
    instance->control_data.press_r = (sbus_buf[16] >> 2) & 0x03;
    instance->control_data.press_m = (sbus_buf[16] >> 4) & 0x03;
    
    /* KeyBoard value */
    instance->control_data.keys.all_keys = (uint16_t)(sbus_buf[17] | (sbus_buf[18] << 8));
    
    /* CRC16 */
    instance->crc16 = (sbus_buf[19] | (sbus_buf[20] << 8)) & 0x07FF;
    
    // 数据有效性检查
    if (!REMOTE_IfDataError()) 
    {
        instance->dr16_data_error_flag = 1;
        memset((void*)&instance->control_data, 0, sizeof(Ctrl_data));
        return;
    }
    instance->dr16_data_error_flag = 0;
    
    // 更新输入模式
    switch (instance->control_data.mode_sw)
    {
        case RC_MODE_REMOTE:
            instance->control_data.input_mode = REMOTE_INPUT;
            break;
        case RC_MODE_KEY_MOUSE:
            instance->control_data.input_mode = KEY_MOUSE_INPUT;
            break;
        case RC_MODE_STOP:
            instance->control_data.input_mode = Control_Stop;
        default:
            break;
    }
    
    // 摇杆数据归一化
    instance->control_data.Normalize_ch0 = RC_Channel_Normalize(instance->control_data.ch0);
    instance->control_data.Normalize_ch1 = RC_Channel_Normalize(instance->control_data.ch1);
    instance->control_data.Normalize_ch2 = RC_Channel_Normalize(instance->control_data.ch2);
    instance->control_data.Normalize_ch3 = RC_Channel_Normalize(instance->control_data.ch3);
    
    // 滚轮数据归一化
    instance->control_data.Normalize_wheel = RC_Channel_Normalize(instance->control_data.wheel);
    
    // 鼠标数据处理
    float mouse_x = instance->control_data.x / (float)MOUSE_MAX;
    float mouse_y = instance->control_data.y / (float)MOUSE_MAX;
    float mouse_z = instance->control_data.z / (float)MOUSE_MAX;
    deadline_limit(mouse_x, 0.01f);
    deadline_limit(mouse_y, 0.01f);
    deadline_limit(mouse_z, 0.01f);
    
    // 位置增量计算
    if (control_time >= 10 && control_time <= 100) 
    {
        instance->control_data.Postion_x = mouse_x * control_time * 0.01f;
        instance->control_data.Postion_y = mouse_y * control_time * 0.01f;
        instance->control_data.Postion_z = mouse_z * control_time * 0.01f;
    }
    
    // 鼠标长按检测
    static uint16_t l_press_time = 0;
    static uint16_t r_press_time = 0;
    
    if (instance->control_data.press_l)
    {
        l_press_time++;
        if (l_press_time >= 50) // 长按阈值约50 * 18ms = 900ms
            instance->control_data.long_press_l = true;
    }
    else
    {
        l_press_time = 0;
        instance->control_data.long_press_l = false;
    }
    
    if (instance->control_data.press_r)
    {
        r_press_time++;
        if (r_press_time >= 50)
            instance->control_data.long_press_r = true;
    }
    else
    {
        r_press_time = 0;
        instance->control_data.long_press_r = false;
    }
}

static void DecodeDR16(USARTInstance *_instance) 
{
    // Decode directly from bsp_usart DMA queue; no extra polling task/buffer is needed.
    uint8_t *rx_buff = RMQueueTop(&_instance->rx_queue);
    if (!rx_buff)
        return;
    SBUS_TO_RC(rx_buff);
    RMQueuePop(&_instance->rx_queue);
}
static float RC_Channel_Normalize(uint16_t raw_ch)
{
    float norm = (raw_ch - RC_CH_VALUE_OFFSET) /
                 (float)(RC_CH_VALUE_MAX - RC_CH_VALUE_OFFSET);
    norm = fmaxf(-1.0f, fminf(1.0f, norm));
    return norm;
}

static uint8_t REMOTE_IfDataError(void) 
{
    // 检查mode_sw是否在有效范围
    if (DR16_instance.control_data.mode_sw > 2)
        return 0;
    // 检查单bit开关是否有效
    if (DR16_instance.control_data.pause > 1 ||
        DR16_instance.control_data.fn_1 > 1 ||
        DR16_instance.control_data.fn_2 > 1 ||
        DR16_instance.control_data.trigger > 1)
        return 0;
    // 检查摇杆范围
    if (DR16_instance.control_data.ch0 > RC_CH_VALUE_MAX || 
        DR16_instance.control_data.ch0 < RC_CH_VALUE_MIN ||
        DR16_instance.control_data.ch1 > RC_CH_VALUE_MAX || 
        DR16_instance.control_data.ch1 < RC_CH_VALUE_MIN ||
        DR16_instance.control_data.ch2 > RC_CH_VALUE_MAX || 
        DR16_instance.control_data.ch2 < RC_CH_VALUE_MIN ||
        DR16_instance.control_data.ch3 > RC_CH_VALUE_MAX || 
        DR16_instance.control_data.ch3 < RC_CH_VALUE_MIN)
        return 0;
    // 检查滚轮范围
    if (DR16_instance.control_data.wheel > RC_CH_VALUE_MAX || 
        DR16_instance.control_data.wheel < RC_CH_VALUE_MIN)
        return 0;
    return 1;
}
// ------------------ 在线检测 ------------------
void DR16_AliveCheck(void)
{
    uint32_t now = DWT_GetTime_ms();
    
    if (now - DR16_instance.dr16_last_rx_ms > 100)  // 超过100ms没收到数据
    {
        USARTInstance *inst = DR16_instance.USART_instance;
        // 清错误标志
        __HAL_UART_CLEAR_PEFLAG(inst->usart_handle);
        __HAL_UART_CLEAR_OREFLAG(inst->usart_handle);
        __HAL_UART_CLEAR_FEFLAG(inst->usart_handle);
        inst->rx_queue.lock = 0;
        
        // 重新启动DMA接收
        uint8_t *ptr = RMQueueGetEndPtr(&inst->rx_queue);
        USART_Receive(inst, ptr, inst->data_len);
        
        // 清零数据并标记离线
        memset((void*)&DR16_instance.control_data, 0, sizeof(Ctrl_data));
        DR16_instance.dr16_online_flag = 0;
    }
    else
    {
        DR16_instance.dr16_online_flag = 1;
    }
}