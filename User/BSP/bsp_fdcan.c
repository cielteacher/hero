//can逻辑：先调用CANRegister注册CAN实例，传入配置（包括can句柄、发送id、接收id等），注册函数会为该实例添加一个硬件过滤器，使其能接收指定id的消息。然后在main函数调用can_init，之后模块就可以通过CAN_Send函数发送消息了。
//对于接收，CAN的中断回调函数会将接收到的消息放入一个消息端口（环形缓冲区），并通知处理任务。处理任务在CAN_Process函数中被调用，会从消息端口取出消息并分发给对应的CAN实例，最后调用该实例的回调函数进行处理。
//CAN_Process调用device层传入的callback来处理数据。
//串口的逻辑也是类似的
#include "bsp_fdcan.h"
#include <string.h>
#include "usbd_cdc_if.h"
#include "bsp_dwt.h"
#include "TASK_Init.h"
//每次配can比较不舒服的就是，必须先配滤波，再启动，虽然想写等最后一个注册完就启动，但又因为rtos的问题，
//可能会出现其他还没初始化就发送了，索性就直接在Task_init里面初始化can设备，这样就没有顺序问题了
/* ================== 全局变量 ================== */
static uint8_t can_idx = 0;
static uint8_t fdcan1_idx = 0;
static uint8_t fdcan2_idx = 0;
static uint8_t fdcan3_idx = 0;
static volatile uint32_t can_error = 0;  
static volatile uint32_t lost_data_count = 0; // 丢失数据计数
static volatile uint8_t port_head = 0;
static volatile uint8_t port_tail = 0;
static CANInstance can_instance[CAN_REGISTER_CNT] = {0};
static CAN_Port can_port[CAN_PORT_CNT] = {0}; // 消息端口数组
/* ================== CAN 初始化（系统启动时调用一次） ================== */
void Can_Init(void)
{ /*开启全局中断，后面参数是；未过滤的全部丢弃*/
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan3, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
    /*	中断触发水位线	*/
    HAL_FDCAN_ConfigFifoWatermark(&hfdcan1, FDCAN_CFG_RX_FIFO0, 1);
    HAL_FDCAN_ConfigFifoWatermark(&hfdcan2, FDCAN_CFG_RX_FIFO1, 1);
    HAL_FDCAN_ConfigFifoWatermark(&hfdcan3, FDCAN_CFG_RX_FIFO0, 1);
    HAL_FDCAN_Start(&hfdcan1);
    HAL_FDCAN_Start(&hfdcan2);
    HAL_FDCAN_Start(&hfdcan3);
    /*	激活中断	*/
    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, FDCAN_INTERRUPT_LINE0);
    HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, FDCAN_INTERRUPT_LINE0);
    HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, FDCAN_INTERRUPT_LINE0);
}

/* ================== 硬件过滤器配置 ================== */
static void CANAddFilter(CANInstance *_instance)
{
    if (_instance == NULL)
        return;
    FDCAN_FilterTypeDef sFilter = {0};
    
    if (_instance->can_handle == &hfdcan1)
    {
        sFilter.FilterIndex = fdcan1_idx++;
        sFilter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
        _instance->FIFO_idx = FDCAN_RX_FIFO0;
    }
    else if (_instance->can_handle == &hfdcan2)
    {
        sFilter.FilterIndex = fdcan2_idx++;
        sFilter.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
        _instance->FIFO_idx = FDCAN_RX_FIFO1;
    }
    else if (_instance->can_handle == &hfdcan3)
    {
        sFilter.FilterIndex = fdcan3_idx++;
        sFilter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
        _instance->FIFO_idx = FDCAN_RX_FIFO0;
    }			/*	FilterIndex和cubemx的配置nbr有关	*/

    sFilter.FilterType   = FDCAN_FILTER_MASK;      // 掩码模式
    sFilter.IdType       = FDCAN_STANDARD_ID;      // 标准帧

    /* 精确匹配：ID = stdID */
    sFilter.FilterID1 = 0;//_instance->rx_id;     // 要接收的ID
    sFilter.FilterID2 = 0;//x7FF;     // 掩码全1 → 精确匹配

    if ( (HAL_FDCAN_ConfigFilter(_instance->can_handle, &sFilter)) != HAL_OK)
    {
        Error_Handler();
    }
}
/* ================== 注册 CAN 模块 ================== */
CANInstance *CANRegister(CAN_Init_Config_s *config)
{
    if (can_idx >= CAN_REGISTER_CNT)
        return NULL;
    if(config == NULL)
        return NULL;
    // 先占位，防止多线程重复注册
    CANInstance *inst = &can_instance[can_idx++];
    memset(inst, 0, sizeof(CANInstance));
	/*传入配置*/
    inst->can_handle = config->can_handle;
    inst->tx_id = config->tx_id;
    inst->rx_id = config->rx_id;
    /*接收长度配置*/
	inst->rx_len = 8;// 必须是8！
	/*发送帧头配置*/
    inst->header_type_def.Identifier = config->tx_id;
    inst->header_type_def.IdType = FDCAN_STANDARD_ID;
    inst->header_type_def.TxFrameType = FDCAN_DATA_FRAME;
    inst->header_type_def.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    inst->header_type_def.BitRateSwitch = FDCAN_BRS_OFF;
    inst->header_type_def.FDFormat = FDCAN_CLASSIC_CAN;
    inst->header_type_def.DataLength = FDCAN_DLC_BYTES_8;
    inst->header_type_def.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    inst->header_type_def.MessageMarker = 0;
		/*添加滤波*/
		CANAddFilter(inst);
    return inst;
}

/* ================== 发送函数（含超时处理） ================== */
HAL_StatusTypeDef CAN_Send(CANInstance *_instance,  uint8_t *tx_buff,float timeout)
{
    // 句柄非空检查
    assert_param(_instance != NULL);
    float start = DWT_GetTime_ms();

    while (HAL_FDCAN_GetTxFifoFreeLevel(_instance->can_handle) == 0)
    {
        if (DWT_GetTime_ms() > start + timeout)
            return HAL_BUSY;
    }

    return HAL_FDCAN_AddMessageToTxFifoQ(
        _instance->can_handle,
        &_instance->header_type_def,
        tx_buff);
}
/* ================== 消息分发 ================== */
static void CAN_Dispatch(CAN_Port *port)
{
    for (uint8_t i = 0; i < can_idx; i++)
    {
        CANInstance *inst = &can_instance[i];
        if (inst->can_handle == port->handle &&
            inst->rx_id == port->id &&
            inst->FIFO_idx == port->fifo)
        {
            inst->rx_len = 8;
            inst->rx_buff = port->data; // 指向消息端口的数据缓冲区
            if (inst->can_module_callback)
                inst->can_module_callback(inst);

            break; // 找到匹配的实例后就可以退出循环了
        }
    }
}
/* ================== 中断处理（会被回调引用） ================== */
static void CAN_Patch(FDCAN_HandleTypeDef *_hcan, uint32_t fifo)
{
    FDCAN_RxHeaderTypeDef rxHeader;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    while (HAL_FDCAN_GetRxFifoFillLevel(_hcan, fifo) > 0)
    {
        // 将消息放入消息端口
        uint8_t next = (port_tail + 1) % CAN_PORT_CNT;
        if (next == port_head) // 队列满，推进 head 覆盖最旧帧
        {
            port_head = (port_head + 1) % CAN_PORT_CNT;
            lost_data_count++; // 统计丢失的数据帧数量
        }
        HAL_FDCAN_GetRxMessage(_hcan, fifo, &rxHeader, can_port[port_tail].data);
        can_port[port_tail].handle = _hcan;
        can_port[port_tail].fifo = fifo;
        can_port[port_tail].id = rxHeader.Identifier;

        port_tail = next;
        // 通知处理任务有新消息了
        vTaskNotifyGiveFromISR(Task_Can_Handle, &xHigherPriorityTaskWoken);
    }
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
/* ================== 回调函数 ================== */
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t it)
{
    CAN_Patch(hfdcan, FDCAN_RX_FIFO1);
}
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t it)
{
    CAN_Patch(hfdcan, FDCAN_RX_FIFO0);
}
/* ================== CAN消息处理 ================== */
void CAN_Process(void)
{
    RMLIB_ENTER_CRITICAL();
    // 处理消息端口中的消息
    while (port_head != port_tail)
    {
        CAN_Port *port = &can_port[port_head];
        CAN_Dispatch(port);
        port_head = (port_head + 1) % CAN_PORT_CNT;
    }
    RMLIB_EXIT_CRITICAL();
}
void FDCAN_Restart(void)
{
    if (can_error & 0x01)
    {
        HAL_FDCAN_Stop(&hfdcan1);
        HAL_FDCAN_Start(&hfdcan1);
        can_error &= ~0x01;
    }

    if (can_error & 0x02)
    {
        HAL_FDCAN_Stop(&hfdcan2);
        HAL_FDCAN_Start(&hfdcan2);
        can_error &= ~0x02;
    }

    if (can_error & 0x04)
    {
        HAL_FDCAN_Stop(&hfdcan3);
        HAL_FDCAN_Start(&hfdcan3);
        can_error &= ~0x04;
    }
}

void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef *hfdcan) {
  if (hfdcan == &hfdcan1) {
    can_error |= 0x01; // FDCAN1 错误
  } else if (hfdcan == &hfdcan2) {
    can_error |= 0x02; // FDCAN2 错误
  } else if (hfdcan == &hfdcan3) {
    can_error |= 0x04; // FDCAN3 错误
  }
}