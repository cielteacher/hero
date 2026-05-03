/**
 * @file           : Lqr.c
 * @brief          : LQR (Linear Quadratic Regulator) 算法实现
 * @Author         : ciel
 * @Date           : 2026-05-03
 * @note
 * 基于离散线性时不变状态空间模型的 LQR 最优控制器。
 * 使用 CMSIS-DSP 矩阵运算库进行矩阵计算。
 *
 * 离散状态空间模型：
 *     X(k+1) = A * X(k) + B * U(k)
 *
 * 最优控制律：
 *     U(k) = -K * (X(k) - Xref)
 *
 * 增益矩阵 K 通过求解离散代数 Riccati 方程 (DARE) 得到：
 *     P = A^T P A - A^T P B (R + B^T P B)^{-1} B^T P A + Q
 *     K = (R + B^T P B)^{-1} B^T P A
 *
 * DARE 求解采用不动点迭代法：
 *     P_{k+1} = A^T P_k A - A^T P_k B (R + B^T P_k B)^{-1} B^T P_k A + Q
 * 从 P_0 = Q 开始迭代，直到 ||P_{k+1} - P_k||_max < tol。
 */
#include "Lqr.h"
#include <string.h>
#include <math.h>

/**
 * @brief 初始化 CMSIS-DSP 矩阵实例并清零
 */
static inline void LQR_Matrix_Init(Matrix *mat, uint16_t rows, uint16_t cols, float *data)
{
    Matrix_Init(mat, rows, cols, data);
    memset(data, 0, sizeof(float) * rows * cols);
}

/**
 * @brief  初始化 LQR 实例
 * @param  lqr     LQR实例指针
 * @param  n       状态维度
 * @param  m       控制输入维度
 */
void lqr_init(LQR_Instance *lqr, uint8_t n, uint8_t m)
{
    if (lqr == NULL || n > LQR_MAX_STATE_DIM || m > LQR_MAX_INPUT_DIM || n == 0 || m == 0)
        return;

    memset(lqr, 0, sizeof(LQR_Instance));

    lqr->n = n;
    lqr->m = m;
    lqr->max_iter = LQR_DARE_MAX_ITER;
    lqr->tol = LQR_DARE_TOL;
    lqr->gain_ready = 0U;

    uint16_t n2 = n * n;
    uint16_t m2 = m * m;
    uint16_t nm = n * m;
    uint16_t mn = m * n;

    /* ====== 分配原始数据内存 ====== */
    lqr->A_data           = (float *)user_malloc(sizeof(float) * n2);
    lqr->B_data           = (float *)user_malloc(sizeof(float) * nm);
    lqr->Q_data           = (float *)user_malloc(sizeof(float) * n2);
    lqr->R_data           = (float *)user_malloc(sizeof(float) * m2);
    lqr->K_data           = (float *)user_malloc(sizeof(float) * mn);
    lqr->P_data           = (float *)user_malloc(sizeof(float) * n2);
    lqr->Error_data       = (float *)user_malloc(sizeof(float) * n);
    lqr->Uout_data        = (float *)user_malloc(sizeof(float) * m);
    lqr->Cache_Mat1_data  = (float *)user_malloc(sizeof(float) * n2);   /* (n x n) */
    lqr->Cache_Mat2_data  = (float *)user_malloc(sizeof(float) * nm);   /* (n x m) */
    lqr->Cache_Mat3_data  = (float *)user_malloc(sizeof(float) * m2);   /* (m x m) */
    lqr->Cache_Mat4_data  = (float *)user_malloc(sizeof(float) * mn);   /* (m x n) */
    lqr->BT_data          = (float *)user_malloc(sizeof(float) * mn);   /* (m x n) */
    lqr->AT_data          = (float *)user_malloc(sizeof(float) * n2);   /* (n x n) */
    lqr->BT_P_data        = (float *)user_malloc(sizeof(float) * mn);   /* (m x n) */
    lqr->BT_P_B_data      = (float *)user_malloc(sizeof(float) * m2);   /* (m x m) */
    lqr->BT_P_B_R_data    = (float *)user_malloc(sizeof(float) * m2);   /* (m x m) */
    lqr->BT_P_B_R_inv_data= (float *)user_malloc(sizeof(float) * m2);   /* (m x m) */
    lqr->BT_P_A_data      = (float *)user_malloc(sizeof(float) * mn);   /* (m x n) */
    lqr->AT_P_data        = (float *)user_malloc(sizeof(float) * n2);   /* (n x n) */
    lqr->AT_P_A_data      = (float *)user_malloc(sizeof(float) * n2);   /* (n x n) */

    /* ====== 初始化矩阵实例 ====== */
    LQR_Matrix_Init(&lqr->A,            n,  n,  lqr->A_data);
    LQR_Matrix_Init(&lqr->B,            n,  m,  lqr->B_data);
    LQR_Matrix_Init(&lqr->Q,            n,  n,  lqr->Q_data);
    LQR_Matrix_Init(&lqr->R,            m,  m,  lqr->R_data);
    LQR_Matrix_Init(&lqr->K,            m,  n,  lqr->K_data);
    LQR_Matrix_Init(&lqr->P,            n,  n,  lqr->P_data);
    LQR_Matrix_Init(&lqr->Error,        n,  1,  lqr->Error_data);
    LQR_Matrix_Init(&lqr->Uout,         m,  1,  lqr->Uout_data);
    LQR_Matrix_Init(&lqr->Cache_Mat1,   n,  n,  lqr->Cache_Mat1_data);
    LQR_Matrix_Init(&lqr->Cache_Mat2,   n,  m,  lqr->Cache_Mat2_data);
    LQR_Matrix_Init(&lqr->Cache_Mat3,   m,  m,  lqr->Cache_Mat3_data);
    LQR_Matrix_Init(&lqr->Cache_Mat4,   m,  n,  lqr->Cache_Mat4_data);
    LQR_Matrix_Init(&lqr->BT,           m,  n,  lqr->BT_data);
    LQR_Matrix_Init(&lqr->AT,           n,  n,  lqr->AT_data);
    LQR_Matrix_Init(&lqr->BT_P,         m,  n,  lqr->BT_P_data);
    LQR_Matrix_Init(&lqr->BT_P_B,       m,  m,  lqr->BT_P_B_data);
    LQR_Matrix_Init(&lqr->BT_P_B_R,     m,  m,  lqr->BT_P_B_R_data);
    LQR_Matrix_Init(&lqr->BT_P_B_R_inv, m,  m,  lqr->BT_P_B_R_inv_data);
    LQR_Matrix_Init(&lqr->BT_P_A,       m,  n,  lqr->BT_P_A_data);
    LQR_Matrix_Init(&lqr->AT_P,         n,  n,  lqr->AT_P_data);
    LQR_Matrix_Init(&lqr->AT_P_A,       n,  n,  lqr->AT_P_A_data);

    /* 计算 A^T 和 B^T 将在 set_model 中完成 */
}

/**
 * @brief  设置系统模型参数 A 和 B，并自动计算转置矩阵
 */
void lqr_set_model(LQR_Instance *lqr, const float *A_data, const float *B_data)
{
    if (lqr == NULL || A_data == NULL || B_data == NULL)
        return;

    uint16_t n2 = lqr->n * lqr->n;
    uint16_t nm = lqr->n * lqr->m;

    memcpy(lqr->A_data, A_data, sizeof(float) * n2);
    memcpy(lqr->B_data, B_data, sizeof(float) * nm);

    /* 计算转置矩阵 */
    arm_mat_trans_f32(&lqr->A, &lqr->AT);
    arm_mat_trans_f32(&lqr->B, &lqr->BT);

    lqr->gain_ready = 0U;
}

/**
 * @brief  设置权重矩阵 Q 和 R
 */
void lqr_set_weights(LQR_Instance *lqr, const float *Q_data, const float *R_data)
{
    if (lqr == NULL || Q_data == NULL || R_data == NULL)
        return;

    uint16_t n2 = lqr->n * lqr->n;
    uint16_t m2 = lqr->m * lqr->m;

    memcpy(lqr->Q_data, Q_data, sizeof(float) * n2);
    memcpy(lqr->R_data, R_data, sizeof(float) * m2);

    lqr->gain_ready = 0U;
}

/**
 * @brief  设置 DARE 迭代参数
 */
void lqr_set_dare_params(LQR_Instance *lqr, uint16_t max_iter, float tol)
{
    if (lqr == NULL)
        return;

    lqr->max_iter = max_iter;
    lqr->tol = tol;
}

/**
 * @brief  求解离散代数 Riccati 方程 (DARE) 并计算最优增益矩阵 K
 *
 * 迭代公式：
 *     P_{k+1} = A^T P_k A - A^T P_k B (R + B^T P_k B)^{-1} B^T P_k A + Q
 *
 * 求解步骤（每次迭代）：
 *   1. BT_P     = B^T * P
 *   2. BT_P_B   = BT_P * B
 *   3. BT_P_B_R = R + BT_P_B
 *   4. 求逆     BT_P_B_R_inv = (BT_P_B_R)^{-1}
 *   5. BT_P_A   = BT_P * A
 *   6. Cache_Mat4 = BT_P_B_R_inv * BT_P_A       (m x n) = K 项
 *   7. Cache_Mat1 = A^T * P * A                   (n x n)
 *                  (先 AT_P = A^T * P, 再 AT_P_A = AT_P * A)
 *   8. Cache_Mat2 = B * Cache_Mat4               (n x n) 用于减法项
 *                  (实际: B * (m x n) = n x n, 但不直接这样算)
 *     更好的分解：
 *   8. Cache_Mat2 = (A^T * P * B) * BT_P_B_R_inv * (B^T * P * A)
 *                  = (AT_P * B) * Cache_Mat4
 *     其中 AT_P_B = AT_P * B 是 (n x m)
 *   9. P_new = Cache_Mat1 - Cache_Mat2 + Q
 *
 * @param  lqr  LQR实例指针
 * @return arm_status ARM_MATH_SUCCESS 或 ARM_MATH_ARGUMENT_ERROR
 */
arm_status lqr_calculate_gain(LQR_Instance *lqr)
{
    if (lqr == NULL)
        return ARM_MATH_ARGUMENT_ERROR;

    uint8_t n = lqr->n;
    uint8_t m = lqr->m;
    uint16_t n2 = n * n;

    /* ====== 初始化 P = Q ====== */
    memcpy(lqr->P_data, lqr->Q_data, sizeof(float) * n2);

    /* 临时矩阵：AT_P_B (n x m)，用于 A^T * P * B */
    float *AT_P_B_data = (float *)user_malloc(sizeof(float) * n * m);
    float *P_new_data  = (float *)user_malloc(sizeof(float) * n2);
    float *Diff_data   = (float *)user_malloc(sizeof(float) * n2);
    Matrix AT_P_B, P_new, Diff;
    Matrix_Init(&AT_P_B, n, m, AT_P_B_data);
    Matrix_Init(&P_new,  n, n, P_new_data);
    Matrix_Init(&Diff,   n, n, Diff_data);

    /* ====== DARE 迭代求解 ====== */
    for (uint16_t iter = 0; iter < lqr->max_iter; iter++)
    {
        /* Step 1: BT_P = B^T * P     (m x n) = (m x n) * (n x n) */
        lqr->MatStatus = arm_mat_mult_f32(&lqr->BT, &lqr->P, &lqr->BT_P);

        /* Step 2: BT_P_B = BT_P * B  (m x m) = (m x n) * (n x m) */
        lqr->MatStatus = arm_mat_mult_f32(&lqr->BT_P, &lqr->B, &lqr->BT_P_B);

        /* Step 3: BT_P_B_R = R + BT_P_B  (m x m) */
        lqr->MatStatus = arm_mat_add_f32(&lqr->R, &lqr->BT_P_B, &lqr->BT_P_B_R);

        /* Step 4: BT_P_B_R_inv = (R + B^T P B)^{-1}  (m x m) */
        lqr->MatStatus = arm_mat_inverse_f32(&lqr->BT_P_B_R, &lqr->BT_P_B_R_inv);
        if (lqr->MatStatus != ARM_MATH_SUCCESS)
        {
            vPortFree(AT_P_B_data);
            vPortFree(P_new_data);
            vPortFree(Diff_data);
            return ARM_MATH_ARGUMENT_ERROR;
        }

        /* Step 5: BT_P_A = BT_P * A  (m x n) = (m x n) * (n x n) */
        lqr->MatStatus = arm_mat_mult_f32(&lqr->BT_P, &lqr->A, &lqr->BT_P_A);

        /* Step 6: Cache_Mat4 = BT_P_B_R_inv * BT_P_A  (m x n) = K */
        lqr->MatStatus = arm_mat_mult_f32(&lqr->BT_P_B_R_inv, &lqr->BT_P_A, &lqr->Cache_Mat4);

        /* Step 7: AT_P = A^T * P     (n x n) = (n x n) * (n x n) */
        lqr->MatStatus = arm_mat_mult_f32(&lqr->AT, &lqr->P, &lqr->AT_P);

        /* Step 7b: AT_P_A = AT_P * A  (n x n) */
        lqr->MatStatus = arm_mat_mult_f32(&lqr->AT_P, &lqr->A, &lqr->AT_P_A);

        /* Step 8: AT_P_B = AT_P * B  (n x m) */
        lqr->MatStatus = arm_mat_mult_f32(&lqr->AT_P, &lqr->B, &AT_P_B);

        /* Step 8b: Cache_Mat1 = AT_P_B * Cache_Mat4  (n x n) = (n x m) * (m x n) */
        /* 这就是 A^T P B * (R + B^T P B)^{-1} * B^T P A */
        lqr->MatStatus = arm_mat_mult_f32(&AT_P_B, &lqr->Cache_Mat4, &lqr->Cache_Mat1);

        /* Step 9: P_new = AT_P_A - Cache_Mat1 + Q */
        lqr->MatStatus = arm_mat_sub_f32(&lqr->AT_P_A, &lqr->Cache_Mat1, &Diff);
        lqr->MatStatus = arm_mat_add_f32(&Diff, &lqr->Q, &P_new);

        /* ====== 检查收敛 ====== */
        float max_diff = 0.0f;
        for (uint16_t i = 0; i < n2; i++)
        {
            float d = fabsf(P_new_data[i] - lqr->P_data[i]);
            if (d > max_diff)
                max_diff = d;
        }

        /* 更新 P */
        memcpy(lqr->P_data, P_new_data, sizeof(float) * n2);

        if (max_diff < lqr->tol)
            break;
    }

    /* ====== 计算最优增益 K = BT_P_B_R_inv * BT_P_A ====== */
    /* 需要使用最终的 P 重新计算一次 */
    lqr->MatStatus = arm_mat_mult_f32(&lqr->BT, &lqr->P, &lqr->BT_P);
    lqr->MatStatus = arm_mat_mult_f32(&lqr->BT_P, &lqr->B, &lqr->BT_P_B);
    lqr->MatStatus = arm_mat_add_f32(&lqr->R, &lqr->BT_P_B, &lqr->BT_P_B_R);
    lqr->MatStatus = arm_mat_inverse_f32(&lqr->BT_P_B_R, &lqr->BT_P_B_R_inv);
    lqr->MatStatus = arm_mat_mult_f32(&lqr->BT_P, &lqr->A, &lqr->BT_P_A);
    lqr->MatStatus = arm_mat_mult_f32(&lqr->BT_P_B_R_inv, &lqr->BT_P_A, &lqr->K);

    vPortFree(AT_P_B_data);
    vPortFree(P_new_data);
    vPortFree(Diff_data);

    if (lqr->MatStatus == ARM_MATH_SUCCESS)
        lqr->gain_ready = 1U;

    return lqr->MatStatus;
}

/**
 * @brief  LQR 控制：计算最优控制输出
 *
 * 控制律：
 *     U(k) = -K * (X(k) - Xref)
 *
 * @param  lqr    LQR实例指针（必须先调用 lqr_calculate_gain）
 * @param  Xk     当前状态向量 (长度 n)
 * @param  Xref   参考状态向量 (长度 n)
 * @return float* 指向控制输出向量 (长度 m)，即 lqr->Uout_data
 *                如果 K 未就绪，返回 NULL
 */
float *lqr_control(LQR_Instance *lqr, const float *Xk, const float *Xref)
{
    if (lqr == NULL || Xk == NULL || Xref == NULL || !lqr->gain_ready)
        return NULL;

    uint8_t n = lqr->n;
    uint8_t m = lqr->m;

    /* 计算状态误差 Error = Xk - Xref */
    for (uint8_t i = 0; i < n; i++)
    {
        lqr->Error_data[i] = Xk[i] - Xref[i];
    }

    /* 计算控制输出 Uout = -K * Error */
    lqr->MatStatus = arm_mat_mult_f32(&lqr->K, &lqr->Error, &lqr->Uout);

    /* 取反：U = -K * Error */
    for (uint8_t i = 0; i < m; i++)
    {
        lqr->Uout_data[i] = -lqr->Uout_data[i];
    }

    return lqr->Uout_data;
}
