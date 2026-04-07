
#ifndef PID_H
#define PID_H

#include "RMLibHead.h"
#include <math.h>
#define PI 3.14159265358979f
/**
 * @brief 标准位置式PID参数
 */
typedef struct {

  float Kp; //!<@brief 比例系数
  float Ki; //!<@brief 积分系数
  float Kd; //!<@brief 微分系数

  float Iout; //!<@brief 单次积分输出
  float ITerm; //!<@brief 积分输出结果 Iout = ITerm_0 + ITerm_1 +....+ ITerm_n
               //!<多次积分之和
  float Dout; //!<@brief 微分输出
  float Pout; //!<@brief 比例输出

  float outlimit;      //!<@brief 输出限幅
  float interlimit;    //!<@brief 积分限幅
  float inter_threUp;  //!<@brief 变速积分上限 输入正的！
  float inter_threLow; //!<@brief 变速积分下限 输入正的！

  float last_measure; //!<@brief 上次测量值
  float error_last;   //!<@brief 上一次误差
  float Dout_last;    //!<@brief 上次D输出
  float DeadBand;     //!<@brief 误差死区
  float pid_out;      //!<@brief PID输出
  float dt;           //!<@brief 时间差
  uint32_t DWT_CNT;   //!<@brief 用于DWT求dt
} PID;

/**
 * @brief 带史密斯预估器的位置式PID参数
 */
typedef struct {
  float Kp;            //!<@brief 比例系数
  float Ki;            //!<@brief 积分系数
  float Kd;            //!<@brief 微分系数
  float Iout;          //!<@brief 单次积分输出
  float ITerm;         //!<@brief 积分输出结果
  float Dout;          //!<@brief 微分输出
  float Pout;          //!<@brief 比例输出
  float Output_LPF_RC; //!<@brief 输出滤波
  float outlimit;      //!<@brief 输出限幅
  float interlimit;    //!<@brief 积分限幅
  float inter_threUp;  //!<@brief 变速积分上限
  float inter_threLow; //!<@brief 变速积分下限
  float error_last;    //!<@brief 上次误差
  float DeadBand;      //!<@brief 误差死区
  float pid_out;       //!<@brief PID输出
  float last_pid_out;  //!<@brief 上次PID输出
  float dt;            //!<@brief 时间差
  uint32_t DWT_CNT;    //!<@brief 用于DWT求dt
} PID_Smis;

/**
 * @brief 增量式PID参数
 */
typedef struct {
  float Kp;         //!<@brief 比例系数
  float Ki;         //!<@brief 积分系数
  float Kd;         //!<@brief 微分系数
  float error_now;  //!<@brief 当前误差
  float error_next; //!<@brief 上一次误差
  float error_last; //!<@brief 上上次误差
  float increament; //!<@brief PID增量
} PID_ADD;

/**
 * @brief 前馈控制
 */
typedef struct {
  float K1;
  float K2;
  float K3;
  float Last_DeltIn;
  float Now_DeltIn;
  float Ref_dot;
  float Ref_ddot;
  float Last_dout;
  float Out;
  float OutMax;
  float dt;
  float DWT_CNT;
} FeedForward_Typedef;

float limit(float val, float max, float min);
/**
 * @brief 标准位置式PID
 * @param[in] current 实际值
 * @param[in] expected 期望值
 * @param[in] parameter PID参数
 */
float PID_Control(float current, float expected, PID *data);

/**
 * @brief 带史密斯预估器的位置式PID
 * @param[in] current 实际值
 * @param[in] expected 期望值
 * @param[in] parameter PID参数
 * @param[in] speed 实际速度
 */
float PID_Control_Smis(float current, float expected, PID_Smis *data,
                       float speed);

/**
 * @brief 增量式PID
 * @param[in] current 实际值
 * @param[in] expect 期望值
 * @param[in] parameter PID参数
 * @return PID增量
 */
float PID_Increment(float current, float expect, PID_ADD *parameter);

/**
 * @brief 前反馈
 * @param[in] FF 前馈参数
 * @param[in] In 前馈输入值
 * @return  前馈结果
 */
float FeedForward_Calc(FeedForward_Typedef *FF, float In);

/**
 * @brief PID积分项清零
 * @param[in] parameter PID参数
 * @return  0
 */
void PID_IoutReset(PID *parameter);

#endif
