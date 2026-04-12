/**
 * @file    PID.c
 * @author  yao
 * @date    1-May-2020
 * @modified 26-Feb-2026, Ciel
 * @brief   PID模块
 */

#include "PID.h"
#include "bsp_dwt.h"
/**
 * @brief 限幅函数
 * @param IN 限幅变量
 * @param MAX 最大值
 * @param MIN 最小值
 */
float limit(float val, float max, float min) {
  if (val < min)
    return min;
  if (val > max)
    return max;
  return val;
}

float PID_Control(float current, float expected,PID *parameter)
{ // 采样时间；获取两次pid计算的时间间隔单位
  // 秒,用于积分和微分
  parameter->dt = DWT_GetDeltaT((void *)&parameter->DWT_CNT);
  // 计算误差
  float error_now = expected - current;
  // 去除死区
  if (fabs(error_now) < parameter->DeadBand) {
    error_now = 0.0f;
    // 死区直接返回
    parameter->error_last = error_now;
    parameter->last_measure = current;

    return parameter->pid_out;
  }
  // 比例计算
  parameter->Pout = parameter->Kp * error_now;
  // 积分计算
  // 抗饱和积分  (P->PreError+P->LastError)/2*(P->PreError - P->I_L)/(P->I_U -
  // P->I_L);
  if (fabs(error_now) < parameter->inter_threLow) {
    // 梯形积分
    if (error_now <= 0)
      parameter->ITerm =
          (error_now + parameter->error_last + parameter->DeadBand) / 2;
    else
      parameter->ITerm =
          (error_now + parameter->error_last - parameter->DeadBand) / 2;
  } else if (fabs(error_now) < parameter->inter_threUp) {
    if (error_now <= 0)
      parameter->ITerm =
          (error_now + parameter->error_last + parameter->DeadBand) / 2 *
          (parameter->inter_threLow + error_now) /
          (parameter->inter_threUp - parameter->inter_threLow);
    else
      parameter->ITerm =
          (error_now + parameter->error_last - parameter->DeadBand) / 2 *
          (error_now - parameter->inter_threLow) /
          (parameter->inter_threUp - parameter->inter_threLow);
  } else
    parameter->ITerm = 0;
  // 积分限幅
  parameter->Iout += parameter->ITerm * parameter->Ki * parameter->dt;
  parameter->Iout = limit(parameter->Iout, parameter->interlimit, -parameter->interlimit);

  //  微分计算 微分先行
  if (parameter->dt > 1e-6f) {
    float diff = (current - parameter->last_measure) / parameter->dt;
    // 微分滤波
    parameter->Dout = parameter->Dout_last * 0.7f + parameter->Kd * diff * 0.3f;
  }
  // 输出计算
  parameter->pid_out = parameter->Pout + parameter->Iout + parameter->Dout;
  parameter->pid_out = limit(parameter->pid_out, parameter->outlimit, -parameter->outlimit);
  // 善后工作
  parameter->error_last = error_now;
  parameter->last_measure = current;
  parameter->Dout_last = parameter->Dout;

  return parameter->pid_out;
}

float PID_Control_Smis(float current, float expected, PID_Smis *parameter,float speed) 
{
  parameter->dt = DWT_GetDeltaT(
      (void *)&parameter->DWT_CNT); // 获取两次pid计算的时间间隔,用于积分和微分
  float error_now = expected - current;

  if (fabs(error_now) < parameter->DeadBand) {
    error_now = 0.0f;
    // 死区直接返回
    parameter->error_last = error_now;
    return parameter->pid_out;
  }

  // 积分计算
  // 抗饱和积分  (P->PreError+P->LastError)/2*(P->PreError - P->I_L)/(P->I_U -
  // P->I_L);
  if (fabs(error_now) < parameter->inter_threLow) {
    // 梯形积分
    if (error_now <= 0)
      parameter->ITerm =
          (error_now + parameter->error_last + parameter->DeadBand) / 2;
    else
      parameter->ITerm =
          (error_now + parameter->error_last - parameter->DeadBand) / 2;
  } else if (fabs(error_now) < parameter->inter_threUp) {
    if (error_now <= 0)
      parameter->ITerm =
          (error_now + parameter->error_last + parameter->DeadBand) / 2 *
          (parameter->inter_threLow + error_now) /
          (parameter->inter_threUp - parameter->inter_threLow);
    else
      parameter->ITerm =
          (error_now + parameter->error_last - parameter->DeadBand) / 2 *
          (error_now - parameter->inter_threLow) /
          (parameter->inter_threUp - parameter->inter_threLow);
  } else
    parameter->ITerm = 0;

  // 比例计算
  parameter->Pout = parameter->Kp * error_now;
  //  微分先行
  parameter->Dout = parameter->Kd * speed;
  // 积分限幅
  parameter->Iout += parameter->ITerm * parameter->Ki * parameter->dt;
  parameter->Iout = limit(parameter->Iout, parameter->interlimit, -parameter->interlimit);
  // 输出计算
  parameter->pid_out = parameter->Pout + parameter->Iout + parameter->Dout;
  // 输出限幅
  parameter->pid_out = limit(parameter->pid_out, parameter->outlimit, -parameter->outlimit);
  parameter->error_last = error_now;
  parameter->last_pid_out = parameter->pid_out;
  return parameter->pid_out;
}

float PID_Increment(float current, float expect, PID_ADD *parameter) {
  parameter->error_now = expect - current;

  parameter->increament =
      parameter->Kp * (parameter->error_now - parameter->error_next) +
      parameter->Ki * (parameter->error_now) +
      parameter->Kd * (parameter->error_now - 2 * parameter->error_next +
                       parameter->error_last);

  parameter->error_last = parameter->error_next;
  parameter->error_next = parameter->error_now;

  return parameter->increament;
}

float FeedForward_Calc(FeedForward_Typedef *FF, float In) {
  FF->dt = DWT_GetDeltaT((void *)&FF->DWT_CNT);
  FF->Now_DeltIn = In;

  FF->Ref_dot = (FF->Now_DeltIn - FF->Last_DeltIn) / FF->dt;
  FF->Ref_ddot = (FF->Ref_dot - FF->Last_dout) / FF->dt;

  FF->Out = FF->Now_DeltIn * FF->K1 + FF->Ref_dot * FF->K2 + FF->Ref_ddot * FF->K3;

  FF->Last_DeltIn = FF->Now_DeltIn;
  FF->Last_dout = FF->Ref_dot;
  FF->Out = limit(FF->Out, FF->OutMax, -FF->OutMax);

  return FF->Out;
}

void PID_IoutReset(PID *parameter) {
  parameter->Iout = 0;
  parameter->pid_out = 0;
}