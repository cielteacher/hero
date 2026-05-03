/**
 * @file           : MPC.cpp
 * @brief          : MPC (Model Predictive Control) 算法实现
 * @Author         : ciel
 * @Date           : 2026-05-02 15:44
 * @note
 * 基于线性时不变离散状态空间模型的无约束 MPC 控制器。
 * 使用 CMSIS-DSP 矩阵运算库进行矩阵计算。
 *
 * 离散状态空间模型：
 *     X(k+1) = A * X(k) + B * U(k)
 *     Y(k)   = X(k)   (C = I, D = 0)
 *
 * 预测方程（展开形式）：
 *     X_pred = Psi * U_seq + Phi * Xk
 *     其中：
 *       Psi = [B,          0,          ..., 0      ]
 *             [A*B,        B,          ..., 0      ]
 *             [A^2*B,      A*B,        ..., 0      ]
 *             [...,        ...,        ..., ...    ]
 *             [A^(Np-1)*B, A^(Np-2)*B, ..., B      ]
 *
 *       Phi = [A; A^2; A^3; ...; A^Np]
 *
 * 代价函数矩阵形式：
 *     J = E^T * Q_bar * E + U^T * R_bar * U
 *     其中 E = Xref_aug - Phi*Xk - Psi*U
 *
 * 无约束解析解：
 *     U = (Psi^T * Q_bar * Psi + R_bar)^{-1} * Psi^T * Q_bar * (Xref_aug - Phi*Xk)
 *
 * 矩阵内存布局说明：
 *     所有矩阵数据按行优先存储。
 *     为适配 STM32H7 + CMSIS-DSP，使用 pvPortMalloc 分配内存。
 *     Hessian 矩阵求逆使用 CMSIS-DSP 单精度求逆。
 */
#include "MPC.h"
#include <cstring>

/**
 * @brief 初始化 CMSIS-DSP 矩阵实例并清零
 */
static inline void MPC_Matrix_Init(Matrix *mat, uint16_t rows, uint16_t cols, float *data)
{
    Matrix_Init(mat, rows, cols, data);
    memset(data, 0, sizeof(float) * rows * cols);
}

/**
 * @brief  构建 Psi 矩阵（预测矩阵）
 * @note   Psi 描述了控制输入序列对预测状态的影响
 *         行块 i (0~Np-1): [A^(i)*B, A^(i-1)*B, ..., A*B, B, 0, ..., 0]
 *         实际构建时按列块递推：
 *         列块 j (0~Np-1): 对应 U(k+j) 对各步预测状态的影响
 *         Psi(i,j) = A^(i-j-1)*B  if i > j
 *                    B             if i == j
 *                    0             if i < j
 *
 * @param  mpc  MPC实例指针
 */
static void MPC_Build_Psi(MPC_Instance *mpc)
{
    uint8_t n = mpc->n;
    uint8_t m = mpc->m;
    uint8_t Np = mpc->Np;

    /* 临时矩阵：A_power = A^k (从 A^0 = I 开始递推) */
    float *A_power_data = (float *)user_malloc(sizeof(float) * n * n);
    float *temp_data = (float *)user_malloc(sizeof(float) * n * n);
    float *AB_data = (float *)user_malloc(sizeof(float) * n * m);

    Matrix A_power, temp_mat, AB;

    Matrix_Init(&A_power, n, n, A_power_data);
    Matrix_Init(&temp_mat, n, n, temp_data);
    Matrix_Init(&AB, n, m, AB_data);

    /* Psi 的列块 j 对应控制输入 U(k+j) */
    for (uint8_t j = 0; j < Np; j++)
    {
        /* 对于行块 i >= j 的部分：
         * Psi(i, j) = A^(i - j) * B  (当 i == j 时为 B) */

        /* 从 A^0 = I 开始 */
        memset(A_power_data, 0, sizeof(float) * n * n);
        for (uint8_t diag = 0; diag < n; diag++)
            A_power_data[diag * n + diag] = 1.0f;

        for (uint8_t i = j; i < Np; i++)
        {
            /* 计算 A^(i-j) * B */
            if (i == j)
            {
                /* A^0 * B = B */
                memcpy(AB_data, mpc->B_data, sizeof(float) * n * m);
            }
            else
            {
                /* A_power = A_power * A */
                arm_mat_mult_f32(&A_power, &mpc->A, &temp_mat);
                memcpy(A_power_data, temp_data, sizeof(float) * n * n);

                /* AB = A_power * B */
                arm_mat_mult_f32(&A_power, &mpc->B, &AB);
            }

            /* 写入 Psi 的 (i, j) 块 */
            uint16_t row_offset = i * n;
            uint16_t col_offset = j * m;
            for (uint16_t r = 0; r < n; r++)
            {
                for (uint16_t c = 0; c < m; c++)
                {
                    mpc->Psi_data[(row_offset + r) * (Np * m) + col_offset + c] = AB_data[r * m + c];
                }
            }
        }
    }

    vPortFree(A_power_data);
    vPortFree(temp_data);
    vPortFree(AB_data);
}

/**
 * @brief  构建 Phi 矩阵（自由响应矩阵）
 * @note   Phi 描述了初始状态 Xk 在无控制输入下的自由响应
 *         Phi = [A; A^2; A^3; ...; A^Np]
 *
 * @param  mpc  MPC实例指针
 */
static void MPC_Build_Phi(MPC_Instance *mpc)
{
    uint8_t n = mpc->n;
    uint8_t Np = mpc->Np;

    float *A_power_data = (float *)user_malloc(sizeof(float) * n * n);
    float *temp_data = (float *)user_malloc(sizeof(float) * n * n);

    Matrix A_power, temp_mat;
    Matrix_Init(&A_power, n, n, A_power_data);
    Matrix_Init(&temp_mat, n, n, temp_data);

    /* 从 A^1 开始 */
    memcpy(A_power_data, mpc->A_data, sizeof(float) * n * n);

    for (uint8_t i = 0; i < Np; i++)
    {
        /* 写入 Phi 的第 i 行块 = A^(i+1) */
        uint16_t row_offset = i * n;
        for (uint16_t r = 0; r < n; r++)
        {
            for (uint16_t c = 0; c < n; c++)
            {
                mpc->Phi_data[(row_offset + r) * n + c] = A_power_data[r * n + c];
            }
        }

        /* A_power = A_power * A (计算下一个幂次) */
        if (i < Np - 1)
        {
            arm_mat_mult_f32(&A_power, &mpc->A, &temp_mat);
            memcpy(A_power_data, temp_data, sizeof(float) * n * n);
        }
    }

    vPortFree(A_power_data);
    vPortFree(temp_data);
}

/**
 * @brief  构建 Q_bar 块对角权重矩阵
 * @note   Q_bar = blkdiag(Q, Q, ..., Q, P)
 *         最后一个块使用终端权重 P
 *
 * @param  mpc  MPC实例指针
 */
static void MPC_Build_Q_bar(MPC_Instance *mpc)
{
    uint8_t n = mpc->n;
    uint8_t Np = mpc->Np;
    uint16_t total_size = Np * n;

    memset(mpc->Q_bar_data, 0, sizeof(float) * total_size * total_size);

    for (uint8_t i = 0; i < Np; i++)
    {
        float *src;
        if (i < Np - 1)
            src = mpc->Q_data;
        else
            src = mpc->P_data;  /* 终端权重 */

        uint16_t offset = i * n;
        for (uint16_t r = 0; r < n; r++)
        {
            for (uint16_t c = 0; c < n; c++)
            {
                mpc->Q_bar_data[(offset + r) * total_size + (offset + c)] = src[r * n + c];
            }
        }
    }
}

/**
 * @brief  构建 R_bar 块对角权重矩阵
 * @note   R_bar = blkdiag(R, R, ..., R)
 *
 * @param  mpc  MPC实例指针
 */
static void MPC_Build_R_bar(MPC_Instance *mpc)
{
    uint8_t m = mpc->m;
    uint8_t Np = mpc->Np;
    uint16_t total_size = Np * m;

    memset(mpc->R_bar_data, 0, sizeof(float) * total_size * total_size);

    for (uint8_t i = 0; i < Np; i++)
    {
        uint16_t offset = i * m;
        for (uint16_t r = 0; r < m; r++)
        {
            for (uint16_t c = 0; c < m; c++)
            {
                mpc->R_bar_data[(offset + r) * total_size + (offset + c)] = mpc->R_data[r * m + c];
            }
        }
    }
}

/**
 * @brief  预计算 Hessian 矩阵 H = Psi^T * Q_bar * Psi + R_bar
 * @note   此矩阵在模型/权重不变时只需计算一次
 *
 * @param  mpc  MPC实例指针
 */
static void MPC_Precompute_Hessian(MPC_Instance *mpc)
{
    /*
     * 步骤1: Cache_Mat1 = Q_bar * Psi  (Np*n x Np*m)
     * 步骤2: Cache_Mat2 = Psi^T * Cache_Mat1  (Np*m x Np*m)
     * 步骤3: Hessian = Cache_Mat2 + R_bar
     */

    /* Cache_Mat1 = Q_bar * Psi */
    mpc->MatStatus = arm_mat_mult_f32(&mpc->Q_bar, &mpc->Psi, &mpc->Cache_Mat1);

    /* Cache_Mat2 = Psi^T * Cache_Mat1 */
    /* 先转置 Psi 到 Cache_Mat3 */
    mpc->MatStatus = arm_mat_trans_f32(&mpc->Psi, &mpc->Cache_Mat3);

    mpc->MatStatus = arm_mat_mult_f32(&mpc->Cache_Mat3, &mpc->Cache_Mat1, &mpc->Cache_Mat2);

    /* Hessian = Cache_Mat2 + R_bar */
    mpc->MatStatus = arm_mat_add_f32(&mpc->Cache_Mat2, &mpc->R_bar, &mpc->Hessian);
}

/**
 * @brief  初始化 MPC 实例
 */
void mpc_init(MPC_Instance *mpc, float dt, uint8_t Np, uint8_t n, uint8_t m)
{
    if (mpc == NULL || Np > MPC_MAX_HORIZON || n > MPC_MAX_STATE_DIM || m > MPC_MAX_INPUT_DIM)
        return;

    memset(mpc, 0, sizeof(MPC_Instance));

    mpc->dt = dt;
    mpc->Np = Np;
    mpc->n = n;
    mpc->m = m;

    uint16_t n2 = n * n;
    uint16_t nm = n * m;
    uint16_t m2 = m * m;
    uint16_t Np_n = Np * n;
    uint16_t Np_m = Np * m;

    /* ====== 分配原始数据内存 ====== */
    mpc->A_data          = (float *)user_malloc(sizeof(float) * n2);
    mpc->B_data          = (float *)user_malloc(sizeof(float) * nm);
    mpc->Q_data          = (float *)user_malloc(sizeof(float) * n2);
    mpc->R_data          = (float *)user_malloc(sizeof(float) * m2);
    mpc->P_data          = (float *)user_malloc(sizeof(float) * n2);
    mpc->Xk_data         = (float *)user_malloc(sizeof(float) * n);
    mpc->Psi_data        = (float *)user_malloc(sizeof(float) * Np_n * Np_m);
    mpc->Phi_data        = (float *)user_malloc(sizeof(float) * Np_n * n);
    mpc->Q_bar_data      = (float *)user_malloc(sizeof(float) * Np_n * Np_n);
    mpc->R_bar_data      = (float *)user_malloc(sizeof(float) * Np_m * Np_m);
    mpc->Hessian_data    = (float *)user_malloc(sizeof(float) * Np_m * Np_m);
    mpc->Hessian_inv_data= (float *)user_malloc(sizeof(float) * Np_m * Np_m);
    mpc->Gradient_data   = (float *)user_malloc(sizeof(float) * Np_m);
    mpc->U_opt_data      = (float *)user_malloc(sizeof(float) * Np_m);
    mpc->Xref_aug_data   = (float *)user_malloc(sizeof(float) * Np_n);
    mpc->FreeResp_data   = (float *)user_malloc(sizeof(float) * Np_n);
    mpc->Error_data      = (float *)user_malloc(sizeof(float) * Np_n);
    mpc->Cache_Mat1_data = (float *)user_malloc(sizeof(float) * Np_n * Np_m);
    mpc->Cache_Mat2_data = (float *)user_malloc(sizeof(float) * Np_m * Np_m);
    mpc->Cache_Mat3_data = (float *)user_malloc(sizeof(float) * Np_m * Np_n);

    /* ====== 初始化矩阵实例 ====== */
    MPC_Matrix_Init(&mpc->A,          n,      n,      mpc->A_data);
    MPC_Matrix_Init(&mpc->B,          n,      m,      mpc->B_data);
    MPC_Matrix_Init(&mpc->Q,          n,      n,      mpc->Q_data);
    MPC_Matrix_Init(&mpc->R,          m,      m,      mpc->R_data);
    MPC_Matrix_Init(&mpc->P,          n,      n,      mpc->P_data);
    MPC_Matrix_Init(&mpc->Xk,         n,      1,      mpc->Xk_data);
    MPC_Matrix_Init(&mpc->Psi,        Np_n,   Np_m,   mpc->Psi_data);
    MPC_Matrix_Init(&mpc->Phi,        Np_n,   n,      mpc->Phi_data);
    MPC_Matrix_Init(&mpc->Q_bar,      Np_n,   Np_n,   mpc->Q_bar_data);
    MPC_Matrix_Init(&mpc->R_bar,      Np_m,   Np_m,   mpc->R_bar_data);
    MPC_Matrix_Init(&mpc->Hessian,    Np_m,   Np_m,   mpc->Hessian_data);
    MPC_Matrix_Init(&mpc->Hessian_inv,Np_m,   Np_m,   mpc->Hessian_inv_data);
    MPC_Matrix_Init(&mpc->Gradient,   Np_m,   1,      mpc->Gradient_data);
    MPC_Matrix_Init(&mpc->U_opt,      Np_m,   1,      mpc->U_opt_data);
    MPC_Matrix_Init(&mpc->Xref_aug,   Np_n,   1,      mpc->Xref_aug_data);
    MPC_Matrix_Init(&mpc->FreeResp,   Np_n,   1,      mpc->FreeResp_data);
    MPC_Matrix_Init(&mpc->Error,      Np_n,   1,      mpc->Error_data);
    MPC_Matrix_Init(&mpc->Cache_Mat1, Np_n,   Np_m,   mpc->Cache_Mat1_data);
    MPC_Matrix_Init(&mpc->Cache_Mat2, Np_m,   Np_m,   mpc->Cache_Mat2_data);
    MPC_Matrix_Init(&mpc->Cache_Mat3, Np_m,   Np_n,   mpc->Cache_Mat3_data);
}

/**
 * @brief  设置系统模型参数 A 和 B，并自动重建预计算矩阵
 */
void mpc_set_model(MPC_Instance *mpc, const float *A_data, const float *B_data)
{
    if (mpc == NULL || A_data == NULL || B_data == NULL)
        return;

    uint16_t n2 = mpc->n * mpc->n;
    uint16_t nm = mpc->n * mpc->m;

    memcpy(mpc->A_data, A_data, sizeof(float) * n2);
    memcpy(mpc->B_data, B_data, sizeof(float) * nm);
}

/**
 * @brief  设置权重矩阵 Q、R、P
 */
void mpc_set_weights(MPC_Instance *mpc, const float *Q_data, const float *R_data, const float *P_data)
{
    if (mpc == NULL || Q_data == NULL || R_data == NULL || P_data == NULL)
        return;

    uint16_t n2 = mpc->n * mpc->n;
    uint16_t m2 = mpc->m * mpc->m;

    memcpy(mpc->Q_data, Q_data, sizeof(float) * n2);
    memcpy(mpc->R_data, R_data, sizeof(float) * m2);
    memcpy(mpc->P_data, P_data, sizeof(float) * n2);
}

/**
 * @brief  MPC 求解：计算最优控制输出
 *
 * 求解流程：
 *   1. 构建 Psi (预测矩阵) 和 Phi (自由响应矩阵)
 *   2. 构建 Q_bar 和 R_bar 权重块对角矩阵
 *   3. 计算 Hessian = Psi^T * Q_bar * Psi + R_bar
 *   4. 计算 Hessian 的逆矩阵 (使用单精度求逆)
 *   5. 构建增广参考 Xref_aug = [Xref; Xref; ...; Xref]
 *   6. 计算自由响应 FreeResp = Phi * Xk
 *   7. 计算误差 E = Xref_aug - FreeResp
 *   8. 计算梯度 G = Psi^T * Q_bar * E
 *   9. 计算最优控制 U = Hessian_inv * G
 *   10. 返回 U 的前 m 个元素作为当前控制量
 *
 * @param  mpc   MPC实例指针
 * @param  Xk    当前状态向量 (长度 n)
 * @param  Xref  参考状态向量 (长度 n)，在预测时域内保持恒定
 * @return float* 指向最优控制序列第一个元素 (长度 m)，即 mpc->U_opt_data
 */
float *mpc_solve(MPC_Instance *mpc, const float *Xk, const float *Xref)
{
    if (mpc == NULL || Xk == NULL || Xref == NULL)
        return NULL;

    uint8_t n = mpc->n;
    uint8_t m = mpc->m;
    uint8_t Np = mpc->Np;
    uint16_t Np_n = Np * n;
    uint16_t Np_m = Np * m;

    /* 更新当前状态 */
    memcpy(mpc->Xk_data, Xk, sizeof(float) * n);

    /* ====== Step 1: 构建预测矩阵 Psi 和 Phi ====== */
    MPC_Build_Psi(mpc);
    MPC_Build_Phi(mpc);

    /* ====== Step 2: 构建权重块对角矩阵 Q_bar 和 R_bar ====== */
    MPC_Build_Q_bar(mpc);
    MPC_Build_R_bar(mpc);

    /* ====== Step 3-4: 计算 Hessian 矩阵及其逆 ====== */
    MPC_Precompute_Hessian(mpc);

    /* 使用 CMSIS-DSP 单精度求逆 */
    mpc->MatStatus = arm_mat_inverse_f32(&mpc->Hessian, &mpc->Hessian_inv);

    /* ====== Step 5: 构建增广参考轨迹 ====== */
    /* Xref_aug = [Xref; Xref; ...; Xref]  (Np*n x 1) */
    for (uint8_t i = 0; i < Np; i++)
    {
        memcpy(&mpc->Xref_aug_data[i * n], Xref, sizeof(float) * n);
    }

    /* ====== Step 6: 计算自由响应 FreeResp = Phi * Xk ====== */
    mpc->MatStatus = arm_mat_mult_f32(&mpc->Phi, &mpc->Xk, &mpc->FreeResp);

    /* ====== Step 7: 计算误差 E = Xref_aug - FreeResp ====== */
    mpc->MatStatus = arm_mat_sub_f32(&mpc->Xref_aug, &mpc->FreeResp, &mpc->Error);

    /* ====== Step 8: 计算梯度 G = Psi^T * Q_bar * E ====== */
    /* 先计算 Cache_Mat1 (重用为向量) = Q_bar * E */
    /* Q_bar 是 Np_n x Np_n, E 是 Np_n x 1, 结果也是 Np_n x 1 */
    /* 这里需要一个 Np_n x 1 的临时向量 */
    float *QbE_data = (float *)user_malloc(sizeof(float) * Np_n);
    Matrix QbE;
    Matrix_Init(&QbE, Np_n, 1, QbE_data);
    mpc->MatStatus = arm_mat_mult_f32(&mpc->Q_bar, &mpc->Error, &QbE);

    /* 然后计算 G = Psi^T * QbE */
    /* Psi^T 是 Np_m x Np_n, QbE 是 Np_n x 1, 结果是 Np_m x 1 */
    float *PsiT_data = (float *)user_malloc(sizeof(float) * Np_m * Np_n);
    Matrix PsiT;
    Matrix_Init(&PsiT, Np_m, Np_n, PsiT_data);
    mpc->MatStatus = arm_mat_trans_f32(&mpc->Psi, &PsiT);
    mpc->MatStatus = arm_mat_mult_f32(&PsiT, &QbE, &mpc->Gradient);

    vPortFree(QbE_data);
    vPortFree(PsiT_data);

    /* ====== Step 9: 计算最优控制 U = Hessian_inv * G ====== */
    mpc->MatStatus = arm_mat_mult_f32(&mpc->Hessian_inv, &mpc->Gradient, &mpc->U_opt);

    /* ====== Step 10: 返回第一个控制量 ====== */
    return mpc->U_opt_data;
}
