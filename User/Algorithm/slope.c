#include <math.h>
#include "slope.h"
static float clamp(float x, float min, float max)
{
    if (x < min) return min;
    if (x > max) return max;
    return x;
}

void Slope_Init(Slope_s *s, float inc, float dec, Enum_Slope_First mode)
{
    s->Increase_Value = fabsf(inc);
    s->Decrease_Value = fabsf(dec);
    s->Slope_First    = mode;

    s->Out = 0.0f;
    s->Now_Planning = 0.0f;
    s->Target = 0.0f;
    s->Now_Real = 0.0f;
}

void Slope_Reset(Slope_s *s, float value)
{
    s->Now_Planning = value;
    s->Out = value;
}

float Slope_Calc(Slope_s *s, float target, float real)
{
    s->Target   = target;
    s->Now_Real = real;

    /* ====== 真实值优先模式（防规划落后导致反向抖动）====== */
    if (s->Slope_First == SLOPE_FIRST_REAL)
    {
        if ((target >= real && real >= s->Now_Planning) ||
            (target <= real && real <= s->Now_Planning))
        {
            s->Now_Planning = real;
        }
    }

    /* ====== 斜率限制核心 ====== */
    float error = target - s->Now_Planning;

    float step;
    if (error > 0.0f)
        step = clamp(error, 0.0f, s->Increase_Value);
    else
        step = clamp(error, -s->Decrease_Value, 0.0f);

    s->Now_Planning += step;
    s->Out = s->Now_Planning;

    return s->Out;
}
