#pragma once
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void *dma_malloc(size_t size);
void dma_free(void *p);

#ifdef __cplusplus
}
#endif

#define DMA_MALLOC(size) dma_malloc(size)
#define DMA_FREE(p) dma_free(p)