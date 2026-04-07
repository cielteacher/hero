#ifndef __SLOPE_H
#define __SLOPE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef enum
{
    SLOPE_FIRST_REAL = 0,   // 真实值优先（防超调抖动）
    SLOPE_FIRST_TARGET      // 目标值优先（硬规划）
} Enum_Slope_First;

/* 斜坡规划结构体 */
typedef struct
{
    float Out;              // 本周期输出（平滑目标）
    float Now_Planning;     // 当前规划轨迹位置
    float Target;           // 原始目标值
    float Now_Real;         // 当前真实值

    float Increase_Value;   // 单周期最大上升量
    float Decrease_Value;   // 单周期最大下降量

    Enum_Slope_First Slope_First;

} Slope_s;


/* 初始化 */
void Slope_Init(Slope_s *s, float inc, float dec, Enum_Slope_First mode);

/* 计算一次斜坡输出 */
float Slope_Calc(Slope_s *s, float target, float real);

/* 强制设置当前规划值（用于急停/复位） */
void Slope_Reset(Slope_s *s, float value);

#ifdef __cplusplus
}
#endif

#endif
