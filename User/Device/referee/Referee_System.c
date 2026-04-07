//3.7 更新，新增拼包.......的注释，发现裁判系统的丢包率有点高，写拼包还要写超时等等，纯自己折磨自己
//usb的较为可靠，把拼包写到usb那里
#include "Referee_System.h"
#include "crc.h"
Referee_instance_t Referee_instance = {0};
USART_Init_Config_s Referee_System_config = // 遥控器串口配置
{
    .transmit_mode = USART_TRANSFER_DMA,
    .receive_mode = USART_RECEIVE_IDLE_DMA,
    .usart_handle = &huart1,
    .data_len = REFEREE_RXDATA_LENGTH,
    .temp_depth = 3,
};
static void DecodeReferee(USARTInstance *_instance)
{
    // static uint8_t remain_buf[256]; // 拼包缓存
    // static uint16_t remain_len = 0; // 缓存长度
    Referee_instance_t *instance = (Referee_instance_t *)_instance->id;
    instance->referee_last_rx_ms = DWT_GetTime_ms(); // 更新最后接收时间戳
    uint8_t *rx_buff = RMQueueTop(&instance->USART_instance->rx_queue);
    RMQueuePop(&instance->USART_instance->rx_queue);
    uint16_t buffer_len = instance->USART_instance->buffer_len;
    if (rx_buff == NULL)
        return;
    // /* 如果上一次有残留数据，先拼包 */
    // if (remain_len > 0)
    // {
    //     memcpy(remain_buf + remain_len, rx_buff, buffer_len);

    //     buffer_len += remain_len;

    //     rx_buff = remain_buf;

    //     remain_len = 0;
    // }
    for (int i = 0; i < buffer_len;)
    {
        if (rx_buff[i] != 0xA5)//帧头固定0xA5
        {
            i++;
            continue;
        }

        if (buffer_len - i < 5)//如果包没有达到帧头长度
            break;//等下次拼包

        if (!Verify_CRC8_Check_Sum(&rx_buff[i], 5))
        {
            i++;
            continue;
        }

        uint16_t data_len = rx_buff[i + 1] | (rx_buff[i + 2] << 8);

        if (data_len > 256) // 防数据长度位解析错误
        {
            i++;
            continue;
        }

        uint16_t frame_len = data_len + 9;

        if (buffer_len - i < frame_len)
            break;

        if (!Verify_CRC16_Check_Sum(&rx_buff[i], frame_len))//未通过CRC16校验跳过当前包
        {
            i+=frame_len;
            continue;
        }

        uint16_t cmd_id = rx_buff[i + 5] | (rx_buff[i + 6] << 8);
        uint8_t *data = &rx_buff[i + 7];

        switch (cmd_id)
        {
        case Referee_Command_ID_GAME_STATUS:
            memcpy(&instance->referee_game_status, data, data_len);
            break;

        case Referee_Command_ID_EVENT_SELF_DATA:
            memcpy(&instance->referee_event_data, data, data_len);
            break;

        case Referee_Command_ID_EVENT_SELF_REFEREE_WARNING:
            memcpy(&instance->referee_warning_data, data, data_len);
            break;

        case Referee_Command_ID_ROBOT_STATUS:
            memcpy(&instance->referee_robot_status, data, data_len);
            break;

        case Referee_Command_ID_ROBOT_POWER_HEAT:
            memcpy(&instance->referee_power_heat_data, data, data_len);
            break;

        case Referee_Command_ID_ROBOT_BUFF:
            memcpy(&instance->referee_buff_data, data, data_len);
            break;

        case Referee_Command_ID_ROBOT_BOOSTER:
            memcpy(&instance->referee_shoot_data, data, data_len);
            break;

        case Referee_Command_ID_INTERACTION:
            memcpy(&instance->referee_interaction_data, data, data_len);
            break;

        case Referee_Command_ID_INTERACTION_ROBOT_RECEIVE_CLIENT_REMOTE_CONTROL:
            memcpy(&instance->referee_mouse_key_cmd, data, data_len);
            break;

        default:
            break;
        }
        i += frame_len; // 推进到下一帧
        // if (i < buffer_len)
        // {
        //     remain_len = buffer_len - i;

        //     memcpy(remain_buf, rx_buff + i, remain_len);
        // }
    }
}
//初始化裁判系统
void Referee_Init(void)
{
    Referee_instance.referee_online_flag = 0;
    Referee_instance.referee_last_rx_ms = DWT_GetTime_ms();
    Referee_instance.USART_instance = USARTRegister(&Referee_System_config);
    Referee_instance.USART_instance->uart_module_callback = DecodeReferee;
    Referee_instance.USART_instance->id = &Referee_instance;
}
void Referee_AliveCheck(void)
{
    uint32_t now = DWT_GetTime_ms();

    if (now - Referee_instance.referee_last_rx_ms > 100) // 超过 100ms 没收到数据
    {
        Referee_instance.referee_online_flag = 0;
        USARTInstance *inst = Referee_instance.USART_instance;
        // 清错误标志
        __HAL_UART_CLEAR_PEFLAG(inst->usart_handle);
        __HAL_UART_CLEAR_OREFLAG(inst->usart_handle);
        __HAL_UART_CLEAR_FEFLAG(inst->usart_handle);
        inst->rx_queue.lock = 0; // 强制解锁
        //  获取新的队列尾指针重新启动 DMA
        uint8_t *ptr = RMQueueGetEndPtr(&inst->rx_queue);
        HAL_UARTEx_ReceiveToIdle_DMA(inst->usart_handle, ptr, inst->data_len);
    }
    else
        Referee_instance.referee_online_flag = 1;
}

