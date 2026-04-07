#include "bsp_gpio.h"
#include "gpio.h"
#include "stm32h7xx_hal_gpio.h"
/**
  * @brief RESET the BMI088_ACCEL_NS
  */
void BMI088_ACCEL_NS_L(void)
{
    HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_RESET);
}

/**
  * @brief SET the BMI088_ACCEL_NS
  */
void BMI088_ACCEL_NS_H(void)
{
    HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_SET);
}

/**
  * @brief RESET the BMI088_GYRO_NS
  */
void BMI088_GYRO_NS_L(void)
{
    HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,GPIO_PIN_RESET);
}

/**
  * @brief RESET the BMI088_GYRO_NS
  */
void BMI088_GYRO_NS_H(void)
{
    HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,GPIO_PIN_SET);
}
