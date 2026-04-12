#include "motor.h"

 float uint_to_float(int X_int, float X_min, float X_max, int Bits){
	
    float span = X_max - X_min;
    float offset = X_min;
    return ((float)X_int)*span/((float)((1<<Bits)-1)) + offset;
}

 int float_to_uint(float x, float x_min, float x_max, int bits){
	
    float span = x_max - x_min;
    float offset = x_min;
    return (int) ((x-offset)*((float)((1<<bits)-1))/span);
}
/**
 * @brief 角度就近值计算，结果在[0, 8192)范围内
 * @param target_angle 目标角度
 * @param angle_now 当前角度
 */
int16_t QuickCentering(int32_t Mch, uint16_t Exp) 
{
    int16_t Exp_ = (Exp + 4095) % 8192;
    if (Exp_ < Exp)
        return Mch < Exp_ ? Exp - 8192 : Exp;
    else
        return Mch > Exp_ ? Exp + 8192 : Exp;
}




















