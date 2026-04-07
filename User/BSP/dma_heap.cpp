#include "dma_heap.h"
#include <string.h>
/* 来自链接脚本的符号 */
extern uint8_t __RAM_D1_used_end__;
extern uint8_t __RAM_D1_end__;

static uint8_t *g_dma_cur = NULL;

static inline void dma_heap_init(void)
{
    if (g_dma_cur == NULL) {
        g_dma_cur = &__RAM_D1_used_end__;
    }
}

void *dma_malloc(size_t size)
{
    if (size == 0) return NULL;
    
    dma_heap_init();
    
    uintptr_t cur = (uintptr_t)g_dma_cur;
    uintptr_t aligned = (cur + 31) & ~31;  /* 32 字节对齐 */
    uintptr_t nxt = aligned + size;
    
    if ((uint8_t *)nxt > &__RAM_D1_end__) {
        return NULL;  /* DMA 堆溢出 */
    }
    
    g_dma_cur = (uint8_t *)nxt;
    return (void *)aligned;
}

void dma_free(void *p)
{
    /* DMA 堆不支持 free，通常长期占用 */
    (void)p;
}