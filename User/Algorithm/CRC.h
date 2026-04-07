#ifndef FILTER_H
#define FILTER_H

#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
#include "RMLibHead.h"
uint8_t Get_CRC8_Check_Sum(uint8_t *pchMessage, size_t dwLength, uint8_t ucCRC8);
size_t Verify_CRC8_Check_Sum(uint8_t *pchMessage, size_t dwLength);
void Append_CRC8_Check_Sum(uint8_t *pchMessage, size_t dwLength);
uint16_t Get_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength, uint16_t wCRC);
uint32_t Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);
void Append_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);
#endif
