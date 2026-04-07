/**
 * @file    Gimbal_Cmd.h
 * @brief   云台板命令处理模块头文件
 */
#ifndef __GIMBAL_CMD_H
#define __GIMBAL_CMD_H

#include "RMLibHead.h"
#include "Gimbal.h"
#include "Shoot.h"
#include "remote.h"
#include "can_comm.h"
#include "BMI088driver.h"
#include "MiniPC.h"

/* ============ 函数声明 ============ */
void Robot_Init(void);
void Robot_Update(void);
uint8_t VisionCanAutoAim(void);

/* ============ 全局变量声明 ============ */
extern Robot_ctrl_cmd_t robot_cmd;

#endif // __GIMBAL_CMD_H