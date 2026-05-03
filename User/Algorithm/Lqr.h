/**
 * @file           : Lqr.h
 * @brief          : LQR (Linear Quadratic Regulator) 算法头文件
 * @Author         : ciel
 * @Date           : 2026-05-03
 * @note
 * 基于离散线性时不变状态空间模型的 LQR 最优控制器。
 * 适用于嵌入式平台 (STM32H7)，使用 CMSIS-DSP 矩阵运算。
 *
 * 状态空间模型：
 *     X(k+1) = A * X(k) + B * U(k)
 *
 * 代价函数：
 *     J = Σ [ X(k)^T Q X(k) + U(k)^T R U(k) ]
 *
 * 最优控制律：
 *     U(k) = -K * X(k)
 *
 * 其中增益矩阵 K 通过求解离散代数 Riccati 方程 (DARE) 得到：
 *     P = A^T P A - A^T P B (R + B^T P B)^{-1} B^T P A + Q
 *     K = (R + B^T P B)^{-1} B^T P A
 */
#ifndef __LQR_H
#define __LQR_H

#include "stdint.h"
#include "arm_math.h"
#include "FreeRTOS.h"
#include "Kalman_Filter.h"
#ifdef __cplusplus
extern "C" {
#endif

/* LQR 状态维度上限 (用于预分配) */
#define LQR_MAX_STATE_DIM  8U

/* LQR 控制输入维度上限 (用于预分配) */
#define LQR_MAX_INPUT_DIM  4U

/* DARE 迭代默认最大次数 */
#define LQR_DARE_MAX_ITER  200U

/* DARE 迭代默认收敛阈值 */
#define LQR_DARE_TOL       1e-6f

typedef struct
{
    /* ====== 系统模型参数 ====== */
    Matrix A;          /* 状态转移矩阵  (n x n) */
    Matrix B;          /* 控制矩阵      (n x m) */
    Matrix Q;          /* 状态权重矩阵  (n x n)，半正定 */
    Matrix R;          /* 控制权重矩阵  (m x m)，正定 */
    Matrix K;          /* 最优反馈增益  (m x n) */
    Matrix P;          /* Riccati 方程解 (n x n) */

    uint8_t n;         /* 状态维度 */
    uint8_t m;         /* 控制输入维度 */

    uint16_t max_iter; /* DARE 迭代最大次数 */
    float tol;         /* DARE 收敛阈值 */

    /* ====== 运行时临时矩阵 ====== */
    Matrix Error;      /* 状态误差 X - Xref (n x 1) */
    Matrix Uout;       /* 控制输出 (m x 1) */

    /* ====== DARE 迭代缓存矩阵 ====== */
    Matrix Cache_Mat1; /* 通用缓存 (n x n) */
    Matrix Cache_Mat2; /* 通用缓存 (n x m) */
    Matrix Cache_Mat3; /* 通用缓存 (m x m) */
    Matrix Cache_Mat4; /* 通用缓存 (m x n) */
    Matrix BT;         /* B 的转置 (m x n) */
    Matrix AT;         /* A 的转置 (n x n) */
    Matrix BT_P;       /* B^T * P (m x n) */
    Matrix BT_P_B;     /* B^T * P * B (m x m) */
    Matrix BT_P_B_R;   /* R + B^T * P * B (m x m) */
    Matrix BT_P_B_R_inv; /* (R + B^T * P * B)^{-1} (m x m) */
    Matrix BT_P_A;     /* B^T * P * A (m x n) */
    Matrix AT_P;       /* A^T * P (n x n) */
    Matrix AT_P_A;     /* A^T * P * A (n x n) */

    /* ====== 原始数据存储 ====== */
    float *A_data;
    float *B_data;
    float *Q_data;
    float *R_data;
    float *K_data;
    float *P_data;
    float *Error_data;
    float *Uout_data;
    float *Cache_Mat1_data;
    float *Cache_Mat2_data;
    float *Cache_Mat3_data;
    float *Cache_Mat4_data;
    float *BT_data;
    float *AT_data;
    float *BT_P_data;
    float *BT_P_B_data;
    float *BT_P_B_R_data;
    float *BT_P_B_R_inv_data;
    float *BT_P_A_data;
    float *AT_P_data;
    float *AT_P_A_data;

    uint8_t gain_ready;  /* 标志：K 矩阵是否已计算完成 */
    arm_status MatStatus;

} LQR_Instance;

/**
 * @brief  初始化 LQR 实例
 * @param  lqr     LQR实例指针
 * @param  n       状态维度
 * @param  m       控制输入维度
 */
void lqr_init(LQR_Instance *lqr, uint8_t n, uint8_t m);

/**
 * @brief  设置系统模型参数 A 和 B
 * @param  lqr      LQR实例指针
 * @param  A_data   状态转移矩阵数据 (n*n)，行优先
 * @param  B_data   控制矩阵数据 (n*m)，行优先
 */
void lqr_set_model(LQR_Instance *lqr, const float *A_data, const float *B_data);

/**
 * @brief  设置权重矩阵 Q 和 R
 * @param  lqr      LQR实例指针
 * @param  Q_data   状态权重 (n*n)，行优先，半正定
 * @param  R_data   控制权重 (m*m)，行优先，正定
 */
void lqr_set_weights(LQR_Instance *lqr, const float *Q_data, const float *R_data);

/**
 * @brief  设置 DARE 迭代参数
 * @param  lqr      LQR实例指针
 * @param  max_iter 最大迭代次数
 * @param  tol      收敛阈值
 */
void lqr_set_dare_params(LQR_Instance *lqr, uint16_t max_iter, float tol);

/**
 * @brief  求解 DARE 并计算最优增益矩阵 K
 * @param  lqr  LQR实例指针
 * @return arm_status ARM_MATH_SUCCESS 或 ARM_MATH_ARGUMENT_ERROR
 */
arm_status lqr_calculate_gain(LQR_Instance *lqr);

/**
 * @brief  LQR 控制：计算最优控制输出
 * @param  lqr    LQR实例指针（必须先调用 lqr_calculate_gain）
 * @param  Xk     当前状态向量 (长度 n)
 * @param  Xref   参考状态向量 (长度 n)
 * @return float* 指向控制输出向量 (长度 m)，即 lqr->Uout_data
 */
float *lqr_control(LQR_Instance *lqr, const float *Xk, const float *Xref);

#ifdef __cplusplus
}
#endif

#endif /* __LQR_H */
