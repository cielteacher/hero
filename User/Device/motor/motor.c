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
//就近转位
int16_t move_nearby_int16(uint16_t target_angle, uint16_t angle_now){
    int16_t diff = target_angle - angle_now;
    if (diff > 4000)
        diff -= 8192;
    if (diff < -4000)
        diff += 8192;
    return diff;
}
 float move_nearby_float(float target_angle, float angle_now){
    float diff = target_angle - angle_now;
    if (diff > 4000.0f)
        diff -= 8192.0f;
    if (diff < -4000.0f)
        diff += 8192.0f;
    return diff;
}



















