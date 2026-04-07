#include "RMQueue.h"
// !!! 注意！在H7系列上（或者其他多ram的板子），freertos动态分配的内存可能并非
// 我们希望的地方（使用DMA时）需要在ld文件中更改，在heap_4中添加__attribute__((section(".RAM_D1"), aligned(32)))
// 
// 悲痛，历史性的退步，在实际使用中，在中断调用是不可避免的，freertos的临界区调用会导致一系列问题，因此把临界区保护删去
#define QUEUE_GET_ADDRESS(handle, i) \
  ((handle)->buffer + ((i) * (handle)->typeSize))
//新的DMA分配器接口，增加分配空间选择
RM_Status RMQueueInitWithAllocator(
    RMQueue_Handle *handle, 
    uint32_t typeSize, 
    uint32_t depth,
    void* (*malloc_func)(size_t),
    void (*free_func)(void*)
)
{
    if (typeSize == 0 || depth == 0 || !malloc_func)
        return RM_ERROR;
    
    handle->buffer = (uint8_t *)malloc_func(typeSize * depth);
    if (handle->buffer == NULL)
        return RM_ERROR;
    
    handle->typeSize = typeSize;
    handle->fifoSize = depth;
    handle->head = 0;
    handle->end = 0;
    handle->size = 0;
    handle->lock = 0;
    handle->free_func = free_func;  /* 需要在 RMQueue_Handle 结构体里加这一行 */
    
    return RM_SUCCESS;
}
RM_Status RMQueueInit(RMQueue_Handle *handle, uint32_t typeSize, uint32_t depth)
{
  if (typeSize == 0 || depth == 0) // para error
    return RM_ERROR;
  handle->buffer = (uint8_t *)RMLIB_MALLOC(typeSize * depth);
  if (handle->buffer == NULL) // malloc error
    return RM_ERROR;
  handle->typeSize = typeSize;
  handle->fifoSize = depth;
  handle->head = 0;
  handle->end = 0;
  handle->size = 0;
  handle->lock = 0;
  return RM_SUCCESS;
}

RM_Status RMQueuePush(RMQueue_Handle *handle, void *data)
{
  if (!handle || !data)
    return RM_ERROR;
  RM_Status ret = RM_ERROR;
  //    RMLIB_ENTER_CRITICAL();
  if (handle->size == handle->fifoSize)
  {
    handle->head = (handle->head + 1) % handle->fifoSize;
    handle->size--;
  }
  if (handle->lock == 0)
  {
    memcpy(QUEUE_GET_ADDRESS(handle, handle->end), data, handle->typeSize);
    handle->end = (handle->end + 1) % handle->fifoSize;
    handle->size++;
    ret = RM_SUCCESS;
  }
  //    RMLIB_EXIT_CRITICAL();
  return ret;
}

RM_Status RMQueuePushEndPtr(RMQueue_Handle *handle)
{
  if (!handle)
    return RM_ERROR;
  RM_Status ret = RM_ERROR;
  if (handle->lock == 1)
  {
    handle->lock = 0;
    handle->end = (handle->end + 1) % handle->fifoSize;
    handle->size++;
    ret = RM_SUCCESS;
  }
  return ret;
}
void *RMQueueGetEndPtr(RMQueue_Handle *handle)
{
  if (!handle)
    return NULL;
  void *ptr = NULL;
  if (handle->lock == 0)
  {
    /* 如果队列满，覆盖最旧数据 */
    if (handle->size == handle->fifoSize)
    {
      handle->head = (handle->head + 1) % handle->fifoSize;
      handle->size--;
    }
    handle->lock = 1;
    ptr = QUEUE_GET_ADDRESS(handle, handle->end);
  }

  return ptr;
}

void *RMQueueTop(RMQueue_Handle *handle)
{
  if (handle && handle->size != 0)
    return (QUEUE_GET_ADDRESS(handle, handle->head));
  else
    return NULL;
}

void *RMQueuePop(RMQueue_Handle *handle)
{
  if (handle && handle->size != 0)
  {

    void *tmp = QUEUE_GET_ADDRESS(handle, handle->head);
    handle->head = (handle->head + 1) % handle->fifoSize;
    handle->size--;

    return tmp;
  }
  else
    return NULL;
}

void RMQueueDelete(RMQueue_Handle *handle)
{
  if (!handle || !handle->buffer)
    return;
  RMLIB_FREE(handle->buffer);
  handle->buffer = NULL;
}


void RMQueueDeleteWithFree(RMQueue_Handle *handle, void (*free_func)(void*))
{
    if (!handle || !handle->buffer)
        return;
    if (free_func) {
        free_func(handle->buffer);
    }
    handle->buffer = NULL;
}
