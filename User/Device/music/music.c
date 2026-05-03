#include "music.h"
#include "stm32h7xx_hal_tim.h"
#include "tim.h"

/**
 * @brief 音乐播放器初始化
 */
void music_init()
{
    HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);
    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, 0); // 初始占空比为0
}

/**
 * @brief 播放指定频率的音调
 * @param freq 频率 (Hz)
 */
void music_play_tone(uint16_t freq)
{
    if (freq == 0)
    {
        music_stop();
        return;
    }
    
    uint32_t tim_clock = HAL_RCC_GetPCLK1Freq() * 2; // TIM12 时钟频率
    uint32_t period = tim_clock / freq;
    uint32_t pulse = period / 2; // 50% 占空比

    __HAL_TIM_SET_AUTORELOAD(&htim12, period - 1);
    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, pulse);
}

/**
 * @brief 停止播放
 */
void music_stop(void)
{
    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, 0);
}