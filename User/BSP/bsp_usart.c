#include "bsp_usart.h"
#include "stm32h7xx_hal_uart.h"
#include "dma_heap.h"
// 裁判系统huart1 or 遥控器huart5
static uint8_t uart_idx = 0;
static volatile uint32_t lost_data_count = 0; // 丢失数据计数
// 静态创建串口实例数组
static USARTInstance usart_instance[DEVICE_USART_CNT] = {0};
// 静态创建串口端口数组
static USART_Port uart_port[USART_PORT_CNT] = {0};
static volatile uint8_t port_head = 0;
static volatile uint8_t port_tail = 0;
/**
 * @brief 启动串口,在每个实例注册之后自动启动接收 (仅DMA)
 * @param _instance 模块的串口实例
 **/
static RM_Status USARTServiceInit(USARTInstance *inst)
{
	// 开启DMA接收
	uint8_t *ptr = RMQueueGetEndPtr(&inst->rx_queue);
	uint8_t resul = 1;
	switch (inst->receive_mode)
	{
	case USART_RECEIVE_DMA:
		// 关闭会捣乱的 UART 中断
		__HAL_UART_DISABLE_IT(inst->usart_handle, UART_IT_RXNE);
		__HAL_DMA_DISABLE_IT(inst->usart_handle->hdmarx, DMA_IT_HT);

		resul = HAL_UART_Receive_DMA(inst->usart_handle, ptr, inst->data_len);
		break;
	case USART_RECEIVE_IT:
		resul = HAL_UART_Receive_IT(inst->usart_handle, ptr, inst->data_len);
		break;
	case USART_RECEIVE_BLOCKING:
		resul = HAL_UART_Receive(inst->usart_handle, ptr, inst->data_len, 100);
		break;
	case USART_RECEIVE_IDLE_DMA:
		__HAL_DMA_DISABLE_IT(inst->usart_handle->hdmarx, DMA_IT_HT);
		resul = HAL_UARTEx_ReceiveToIdle_DMA(inst->usart_handle, ptr, inst->data_len);
		break;
	default:
		resul = 1;
		break;
	}
	if (resul)
		return RM_ERROR;
	else
		return RM_SUCCESS;
}
USARTInstance *USARTRegister(const USART_Init_Config_s *_config)
{

	if (uart_idx >= DEVICE_USART_CNT || !_config || !_config->usart_handle)
		return NULL;
	/* 对队列进行初始化 */
	USARTInstance *inst = &usart_instance[uart_idx];
	uart_idx++; // 下一个实例索引
	// 清零实例
	memset(inst, 0, sizeof(USARTInstance));
	// 赋值基本信息
	inst->usart_handle = _config->usart_handle;
	inst->data_len = _config->data_len;
	inst->temp_depth = _config->temp_depth;
	inst->receive_mode = _config->receive_mode;
	inst->transmit_mode = _config->transmit_mode;

	// 使用 RMQueue 动态分配接收缓冲区
	// 每个元素大小 len, 队列深度为 depth
	if (RMQueueInitWithAllocator(&inst->rx_queue, inst->data_len, inst->temp_depth, dma_malloc, dma_free) != RM_SUCCESS)
		return NULL; // 分配失败
	if (USARTServiceInit(inst) != RM_SUCCESS)
		return NULL;
	return inst;
}

/* 串口发送 仅进行了简单包装 */
RM_Status USART_Transmit(USARTInstance *inst, uint8_t *send_buff, uint16_t data_len)
{
	uint8_t resul = 1;
	switch (inst->transmit_mode)
	{
	case USART_TRANSFER_DMA:
		resul = HAL_UART_Transmit_DMA(inst->usart_handle, send_buff, data_len);
		break;
	case USART_TRANSFER_IT:
		resul = HAL_UART_Transmit_IT(inst->usart_handle, send_buff, data_len);
		break;
	case USART_TRANSFER_BLOCKING:
		resul = HAL_UART_Transmit(inst->usart_handle, send_buff, data_len, 100);
		break;
	default:
		resul = 1;
		break;
	}
	if (resul)
		return RM_ERROR;
	else
		return RM_SUCCESS;
}
/* 串口接收 仅进行了简单包装 */
RM_Status USART_Receive(USARTInstance *inst, uint8_t *recv_buff, uint16_t data_len)
{
	uint8_t resul = 1;
	switch (inst->receive_mode)
	{
	case USART_RECEIVE_DMA:
		resul = HAL_UART_Receive_DMA(inst->usart_handle, recv_buff, data_len);
		break;
	case USART_RECEIVE_IT:
		resul = HAL_UART_Receive_IT(inst->usart_handle, recv_buff, data_len);
		break;
	case USART_RECEIVE_BLOCKING:
		resul = HAL_UART_Receive(inst->usart_handle, recv_buff, data_len, 0);
		break;
	case USART_RECEIVE_IDLE_DMA:
		resul = HAL_UARTEx_ReceiveToIdle_DMA(inst->usart_handle, recv_buff, data_len);
		break;
	default:
		resul = 1;
		break;
	}
	if (resul)
		return RM_ERROR;
	else
		return RM_SUCCESS;
}
static void USART_Patch(UART_HandleTypeDef *huart, uint16_t Size)
{
	// 找到对应 USARTInstance
	USARTInstance *inst = NULL;
	for (int i = 0; i < uart_idx; i++)
	{
		if (usart_instance[i].usart_handle == huart)
		{
			inst = &usart_instance[i];
			break;
		}
	}
	if (inst == NULL)
		return;
	// 推进队列尾指针
	RMQueuePushEndPtr(&inst->rx_queue);
	// 重新启动 DMA
	uint8_t *ptr = RMQueueGetEndPtr(&inst->rx_queue);
	USART_Receive(inst, ptr, inst->data_len);
	// 放入 FIFO 队列
	uint8_t next = (port_tail + 1) % USART_PORT_CNT;
	if (next == port_head)
	{
		lost_data_count++;							  // 统计丢失的数据帧数量
		port_head = (port_head + 1) % USART_PORT_CNT; // 队列满丢最旧
	}
	uart_port[port_tail].instance = inst;
	uart_port[port_tail].length = Size;

	port_tail = next;

	// 通知处理任务
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	vTaskNotifyGiveFromISR(Task_Usart_Handle, &xHigherPriorityTaskWoken);
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);


}
// 发生硬件错误
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	for (int i = 0; i < uart_idx; i++)
	{
		USARTInstance *inst = &usart_instance[i];
		if (inst->usart_handle == huart)
		{
			__HAL_UNLOCK(huart);
			inst->rx_queue.lock = 0; // 强制解锁
 			// 1. 停止异常的DMA
            // HAL_UART_DMAStop(huart);
            
            // 2. 清除所有错误标志
            __HAL_UART_CLEAR_PEFLAG(huart);
            __HAL_UART_CLEAR_FEFLAG(huart);
            __HAL_UART_CLEAR_NEFLAG(huart);
            __HAL_UART_CLEAR_OREFLAG(huart);
            //  清除DMA1 Stream4 传输错误标志（你的DMA通道）
      		__HAL_DMA_CLEAR_FLAG(huart->hdmarx, DMA_FLAG_FEIF0_4);
      		__HAL_DMA_CLEAR_FLAG(huart->hdmarx, DMA_FLAG_FEIF0_4);
            // 3. 重置UART状态 + 重启DMA接收
            huart->ErrorCode = HAL_UART_ERROR_NONE;
            huart->RxState = HAL_UART_STATE_READY;
            huart->gState = HAL_UART_STATE_READY;

			// 3. **放弃当前半包数据**
			// 直接重新获取队列尾指针启动 DMA
			uint8_t *ptr = RMQueueGetEndPtr(&inst->rx_queue);
			//  重新启动 DMA 接收
			USART_Receive(inst, ptr, inst->data_len);
			break;
		}
	}
}

// 感觉不用这么麻烦，直接固定一下顺序，0是遥控器，1是裁判系统
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	USART_Patch(huart, 0);
}
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	USART_Patch(huart, Size);
}
/* ================== 消息处理 ================== */
void USART_Process(void)
{
	// 处理消息端口中的消息
	while (port_head != port_tail)
	{
		RMLIB_ENTER_CRITICAL();
		USART_Port *port = &uart_port[port_head];
		// 找到对应 USARTInstance
		USARTInstance *inst = port->instance;
		inst->buffer_len = port->length; // 更新接收数据长度
		port_head = (port_head + 1) % USART_PORT_CNT;
		RMLIB_EXIT_CRITICAL();
		// 处理数据
		if (inst && inst->uart_module_callback && inst->buffer_len == inst->data_len)
			inst->uart_module_callback(inst);
	}

}
