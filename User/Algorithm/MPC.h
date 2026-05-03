/**
 * @file           : MPC.h
 * @brief          : MPC (Model Predictive Control) 算法头文件
 * @Author         : ciel
 * @Date           : 2026-05-02
 * @note
 * 基于线性时不变离散状态空间模型的无约束 MPC 控制器。
 * 适用于嵌入式平台 (STM32H7)，使用 CMSIS-DSP 矩阵运算。
 *
 * 状态空间模型：
 *     X(k+1) = A * X(k) + B * U(k)
 *     Y(k)   = X(k)   (C = I, D = 0)
 *
 * 代价函数：
 *     J = Σ (Xref - X(k+i|k))^T Q (Xref - X(k+i|k))
 *       + Σ U(k+i)^T R U(k+i)
 *       + (Xref - X(k+Np|k))^T P (Xref - X(k+Np|k))
 *
 * 解析解：
 *     U = (Psi^T * Q_bar * Psi + R_bar)^{-1} * Psi^T * Q_bar * E
 *     其中 E = Xref_augmented - Phi * Xk
 */
#ifndef MPC_H
#define MPC_H

#include "stdint.h"
#include "arm_math.h"
#include "Kalman_Filter.h"
#include "FreeRTOS.h"

#ifdef __cplusplus
extern "C" {
#endif

/* MPC 最大预测步长，用于预分配矩阵内存 */
#define MPC_MAX_HORIZON  20U

/* MPC 状态维度上限 (用于预分配) */
#define MPC_MAX_STATE_DIM 4U

/* MPC 控制输入维度上限 (用于预分配) */
#define MPC_MAX_INPUT_DIM 2U

typedef struct
{
    /* ====== 系统模型参数 ====== */
    Matrix A;          /* 状态转移矩阵  (n x n) */
    Matrix B;          /* 控制矩阵      (n x m) */
    Matrix Q;          /* 过程误差权重   (n x n) */
    Matrix R;          /* 控制输入权重   (m x m) */
    Matrix P;          /* 终端误差权重   (n x n) */
    Matrix Xk;         /* 当前状态       (n x 1) */

    float dt;          /* 采样时间 (s) */
    uint8_t Np;        /* 预测步长 */
    uint8_t n;         /* 状态维度 */
    uint8_t m;         /* 控制输入维度 */

    /* ====== 预计算矩阵 (在 init 中计算一次) ====== */
    Matrix Psi;        /* 预测矩阵      (Np*n x m*Np)  - 控制输入对预测状态的影响 */
    Matrix Phi;        /* 自由响应矩阵  (Np*n x n)     - 自由响应部分 */
    Matrix Q_bar;      /* 权重块对角矩阵 (Np*n x Np*n) */
    Matrix R_bar;      /* 控制权重块对角矩阵 (m*Np x m*Np) */

    /* ====== 运行时临时矩阵 ====== */
    Matrix Hessian;    /* H = Psi^T * Q_bar * Psi + R_bar  (m*Np x m*Np) */
    Matrix Hessian_inv;/* H 的逆矩阵 (m*Np x m*Np) */
    Matrix Gradient;   /* G = Psi^T * Q_bar * E  (m*Np x 1) */
    Matrix U_opt;      /* 最优控制序列 (m*Np x 1) */
    Matrix Xref_aug;   /* 增广参考轨迹 (Np*n x 1) */
    Matrix FreeResp;   /* 自由响应 = Phi * Xk (Np*n x 1) */
    Matrix Error;      /* E = Xref_aug - FreeResp (Np*n x 1) */

    /* ====== 缓存矩阵 (用于中间计算) ====== */
    Matrix Cache_Mat1; /* 通用缓存矩阵 */
    Matrix Cache_Mat2; /* 通用缓存矩阵 */
    Matrix Cache_Mat3; /* 通用缓存矩阵 */

    /* ====== 原始数据存储 ====== */
    float *A_data;
    float *B_data;
    float *Q_data;
    float *R_data;
    float *P_data;
    float *Xk_data;
    float *Psi_data;
    float *Phi_data;
    float *Q_bar_data;
    float *R_bar_data;
    float *Hessian_data;
    float *Hessian_inv_data;
    float *Gradient_data;
    float *U_opt_data;
    float *Xref_aug_data;
    float *FreeResp_data;
    float *Error_data;
    float *Cache_Mat1_data;
    float *Cache_Mat2_data;
    float *Cache_Mat3_data;

    arm_status MatStatus;

} MPC_Instance;

/**
 * @brief  初始化 MPC 实例
 * @param  mpc   MPC实例指针
 * @param  dt    采样时间 (秒)
 * @param  Np    预测步长
 * @param  n     状态维度
 * @param  m     控制输入维度
 */
void mpc_init(MPC_Instance *mpc, float dt, uint8_t Np, uint8_t n, uint8_t m);

/**
 * @brief  设置系统模型参数 A 和 B
 * @param  mpc     MPC实例指针
 * @param  A_data  状态转移矩阵数据 (n*n)，行优先
 * @param  B_data  控制矩阵数据 (n*m)，行优先
 */
void mpc_set_model(MPC_Instance *mpc, const float *A_data, const float *B_data);

/**
 * @brief  设置权重矩阵 Q, R, P
 * @param  mpc     MPC实例指针
 * @param  Q_data  过程误差权重 (n*n)，行优先
 * @param  R_data  控制输入权重 (m*m)，行优先
 * @param  P_data  终端误差权重 (n*n)，行优先
 */
void mpc_set_weights(MPC_Instance *mpc, const float *Q_data, const float *R_data, const float *P_data);

/**
 * @brief  MPC 求解：计算最优控制输出
 * @param  mpc     MPC实例指针
 * @param  Xk      当前状态向量 (n x 1)
 * @param  Xref    参考状态向量 (n x 1)，在预测时域内保持恒定
 * @return float*  指向最优控制序列的第一个元素 (长度 m)
 */
float *mpc_solve(MPC_Instance *mpc, const float *Xk, const float *Xref);

#ifdef __cplusplus
}
#endif

#endif /* MPC_H */
