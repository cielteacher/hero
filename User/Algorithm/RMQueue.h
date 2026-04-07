/**
 * @file RMQueue.h
 * @author yao
 * @date 2020年11月4日
 * @brief 队列模块
 *
 * @details C语言队列，可以结合DMA使用，
 */
/*| 动作       	| 现实比喻       						|
	| ----------	| ----------								|
	| GetEndPtr  	| 工人拿一个空箱子						|
	| DMA写      	| 把包裹放进去    						|
	| PushEndPtr 	| 把箱子放上传送带    				|
	| Top        	| 客服处理最前的箱子（不拿走）|
	| Pop        	| 客服处理完把箱子拿走 				|
*/
#ifndef _RMQueue_H
#define _RMQueue_H

#include "RMLibHead.h"

RMLIB_CPP_BEGIN

typedef struct {
    uint8_t  *buffer;
    uint32_t typeSize;
    uint32_t fifoSize;
    uint32_t head;
    uint32_t end;
    uint32_t size;
    uint8_t  lock;
    void (*free_func)(void*);  /* 记录用哪个 free */
} RMQueue_Handle;
/**
 * @brief 返回队列的大小
 * @param handle 队列句柄
 * @return 队列大小
 */
inline uint32_t RMQueueSize(RMQueue_Handle *handle) 
{
    return handle ? handle->size : 0;
}
/**
 * @brief 判断队列为空
 * @param handle 队列句柄
 * @return 队列为空返回真
 */
inline RM_Status RMQueueIsEmpty(RMQueue_Handle *handle) 
{
    return handle && handle->size == 0 ? RM_YES : RM_NO;
}
/**
 * @brief 清除队列中的内容
 * @param handle 队列句柄
 */

inline void RMQueueClear(RMQueue_Handle *handle)
{
    if (!handle)
			return;
    RMLIB_ENTER_CRITICAL();
    handle->head = handle->end = handle->size = 0;
    handle->lock = 0;
    RMLIB_EXIT_CRITICAL();
}
/**
 * @brief 初始化队列
 * @param handle 队列句柄
 * @param typeSize 元素大小（字节）
 * @param depth 队列深度
 * @return 成功分配内存返回RM_SUCCESS，否则返回RM_ERROR
 */
RM_Status RMQueueInit(RMQueue_Handle *handle, uint32_t typeSize, uint32_t depth);

/**
 * @brief 将数据压入队列
 * @param handle 队列句柄
 * @param dataPtr 数据指针
 * @return 成功返回RM_SUCCESS， 队列满返回RM_ERROR
 */
RM_Status RMQueuePush(RMQueue_Handle *handle, void *data);

/**
 * @brief 得到队列首元素指针，但是不会更改队列长度
 * @see RMQueuePop
 * @param handle 队列句柄
 * @return 队列首元素指针
 */
void *RMQueueTop(RMQueue_Handle *handle);

/**
 * @brief 弹出队列首元素指针，并更新队列长度
 * @note 如果使用了{@link RMQueueGetEndPtr}则不应该用这个函数获取数据，应该使用{@link RMQueueTop}获取然数据，数据读取完成后使用该函数弹出首元素
 * @see RMQueueTop RMQueueGetEndPtr
 * @param handle 队列句柄
 * @return 队列首元素指针
 */
void *RMQueuePop(RMQueue_Handle *handle);
/**
 * @brief 删除队列，释放内存
 * @param handle 队列句柄
 */
void RMQueueDelete(RMQueue_Handle *handle);
/**
 * @brief 获取队列尾指针
 * @note 结合DMA使用，先获取尾部指针用DMA写入，然后使用RMQueuePushEndPtr推入队列，推入之前不能Push
 * @see RMQueuePushEndPtr
 * @param handle 队列句柄
 * @return 队列尾指针
 */
void *RMQueueGetEndPtr(RMQueue_Handle *handle);
/**
 * @brief 将尾指针直接推入队列
 * @note 结合DMA使用，必须先使用RMQueueGetEndPtr取出一个尾指针
 * @param handle 队列句柄
 * @return 是否成功
 */
RM_Status RMQueuePushEndPtr(RMQueue_Handle *handle);
/**
 * @brief 使用自定义分配器初始化队列
 * @param handle 队列句柄
 * @param typeSize 元素大小
 * @param depth 队列深度
 * @param malloc_func 分配函数指针 (e.g., pvPortMalloc, dma_malloc)
 * @param free_func 释放函数指针 (e.g., vPortFree, dma_free)
 * @return 成功返回 RM_SUCCESS
 */
RM_Status RMQueueInitWithAllocator(
    RMQueue_Handle *handle, 
    uint32_t typeSize, 
    uint32_t depth,
    void* (*malloc_func)(size_t),
    void (*free_func)(void*)
);

/**
 * @brief 删除队列，使用指定的释放函数
 */
void RMQueueDeleteWithFree(RMQueue_Handle *handle, void (*free_func)(void*));

RMLIB_CPP_END

#endif
