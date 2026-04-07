#ifndef BSP_SPI_H
#define BSP_SPI_H

/*include*/
#include "spi.h"
#include "RMLibHead.h"


/* Externs ------------------------------------------------------------------*/
extern uint8_t BMI088_Read_Write_Byte(uint8_t Tx_Data);
extern SPI_HandleTypeDef *BMI088_SPI;


#endif
