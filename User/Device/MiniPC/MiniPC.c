// 最失败的一集，把usb的板级支持包和MINIPC这个设备写到一起了
// 可能有点乱，凑合看吧
// 接收是不是可以自定义缓冲区？这样就没有拷贝数据的开销了
#include "MiniPC.h"
MiniPC_instance_t MiniPC_instance = {0};
/* 拼包参数 */
static uint16_t buffer_len = 0;
static uint8_t remain_buf[80]; // 拼包缓存
static uint16_t remain_len = 0; // 残余长度
void MiniPC_Init(void)
{
    MiniPC_instance.MiniPC_Online_Flag = 0;
    MiniPC_instance.MiniPC_Last_rx_ms = DWT_GetTime_ms();
    // 初始化循环队列（64字节是不是有点太大了🤔，usb最大包64，难道每次都是64吗？）
    if (RMQueueInit(&MiniPC_instance.MIniPC_queue, CDC_DATA_FS_MAX_PACKET_SIZE, 4) != RM_SUCCESS)
        return; // 分配失败
    // 仿照串口一初始化就开启接收？
    // 不要，收到包后usb会自动跳到接收函数，直接在函数里写处理就行，主要usb_receive函数是中断，处理不要写太长
}
void MiniPC_Send(uint8_t *Buf, uint16_t Len)
{
        CDC_Transmit_HS(Buf, Len);
}
// 在usbd_cdc中receive函数中调用
void DecodeMiniPC(uint8_t *Buf, uint32_t *Len)
{
    uint8_t *ptr = RMQueueGetEndPtr(&MiniPC_instance.MIniPC_queue);
    memcpy(ptr, Buf, *Len);
    RMQueuePushEndPtr(&MiniPC_instance.MIniPC_queue);
    MiniPC_instance.rx_buff_len = *Len;
    MiniPC_instance.MiniPC_Last_rx_ms = DWT_GetTime_ms();
    MiniPC_instance.MiniPC_Online_Flag = 0; // 存活标志更新
    // 进行任务通知
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(Task_Usb_Handle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
void Usb_Process(void)
{
	buffer_len = MiniPC_instance.rx_buff_len;
    uint8_t *rx_buff = NULL;
	rx_buff = RMQueueTop(&MiniPC_instance.MIniPC_queue);
    RMQueuePop(&MiniPC_instance.MIniPC_queue);
    if (rx_buff == NULL || buffer_len == 0)
        return;
    RMLIB_ENTER_CRITICAL();
    /* 如果上一次有残留数据，先拼包 */
    if (remain_len > 0)
    {
        if (remain_len + buffer_len > sizeof(remain_buf))
        {
            remain_len = 0; // 防止越界，丢掉残余
        }
        else
        {
            memcpy(remain_buf + remain_len, rx_buff, buffer_len);
            buffer_len += remain_len;
            rx_buff = remain_buf;
            remain_len = 0;
        }
    }

    uint16_t i = 0;
    while (i + sizeof(FrameHeader_t) <= buffer_len)
    {
        if (rx_buff[i] != RECEIVE_SOF) // 寻找帧头
        {
            i++;
            continue;
        }
        if (i + 2 >= buffer_len) // 不足以读取长度和ID，等待下一包
            break;

        uint8_t data_len = rx_buff[i + 1]; // 数据段长度
        // uint8_t data_id = rx_buff[i + 2];
        uint16_t frame_len = 3 + data_len + 1; // SOF+LEN+ID+DATA+EOF
        // 检查整帧是否在 buffer 内
        if (i + frame_len > buffer_len)
            break;                                     // 半包，下一次拼接
        if (rx_buff[i + frame_len - 1] != RECEIVE_EOF) // 寻找帧尾
        {
            i++;
            continue;
        }
        if (data_len > sizeof(MiniPC_instance.receive_data.data))
            data_len = sizeof(MiniPC_instance.receive_data.data); // 防止越界

        memcpy(&MiniPC_instance.receive_data.data, rx_buff + i + 3, data_len); // 数据段
        i += frame_len;
    }
    if (i < buffer_len) // 拼包
    {
        remain_len = buffer_len - i;
        if (remain_len <= sizeof(remain_buf))
            memcpy(remain_buf, rx_buff + i, remain_len);
        else
            remain_len = 0; // 防止越界，丢掉残余
    }
    RMLIB_EXIT_CRITICAL();
}

void MiniPC_AliveCheck(void)
{
    uint32_t current_time = DWT_GetTime_ms();
    if (current_time - MiniPC_instance.MiniPC_Last_rx_ms > 1000) // 1秒超时
    {
        MiniPC_instance.MiniPC_Online_Flag = 0; // 超时，标记离线
    }
    else
    {
        MiniPC_instance.MiniPC_Online_Flag = 1; // 在线
    }
}
