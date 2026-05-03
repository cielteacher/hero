/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : Config.c
 * @brief          : Configuare the Robot Functions
 * @author         : Yan Yuanbin
 * @date           : 2023/05/21
 * @version        : v1.0
 ******************************************************************************
 * @attention      : To be perfected
 ******************************************************************************
 */
/* USER CODE END Header */
// 卡尔曼滤波器
// 预测
// 先验状态估计 = 状态转移矩阵 *上次状态估计 + 控制矩阵 * 控制输入 
// 先验误差协方差矩阵 = 状态转移矩阵 * 上次误差协方差矩阵 * 状态转移矩阵的转置 + 预测过程噪声协方差
// 校正
// 卡尔曼增益 = 先验误差协方差 * 观测矩阵的转置 / (观测矩阵 * 先验误差协方差 * 观测矩阵的转置 + 测量噪声协方差)
// 后验估计 = 先验状态估计 + 卡尔曼增益 * (测量状态 - 观测矩阵的转置 * 先验状态估计)
// 后验误差协方差矩阵 = (单位矩阵 - 卡尔曼增益 * 观测矩阵) * 先验误差协方差矩阵
// 在非线性系统中，

#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H
#ifdef __cplusplus
extern "C"
{
}
#endif
/* include  */
#include "RMLibHead.h"
#include "arm_math.h"
#include "math.h"
#include "stdbool.h"
/* define */
/**
 * @brief the value of local gravity acceleration
 */

#define VAL_LIMIT(x, min, max) \
do                             \
{                              \
  if ((x) > (max))             \
  {                            \
    (x) = (max);               \
  }                            \
  else if ((x) < (min))        \
  {                            \
    (x) = (min);               \
  }                            \
} while (0U)

#define GravityAccel 9.799f

#define Angle_to_rad 0.01745329f

#define Rad_to_angle 57.2957732f

/**
 * @brief Euler's Number
 */
#define Euler_Number 2.718281828459045f

/**
 * @brief radian system rotation degrees system , 180.f/PI
 */
#define RadiansToDegrees 57.295779513f

/**
 * @brief degrees system rotation radian system , PI/180.f
 */
#define DegreesToRadians 0.01745329251f

/* Vision reslove constants -------------------------------------------------*/

/**
 * @brief  Decision Marking mode
 *         0: select the minimum yaw armor
 *         1: select the minimum distance armor
 */
#define Yaw_Distance_Decision 0

/**
 * @brief ballistic coefficient
 * @note  17mm: 0.038
 *        42mm: 0.019
 */
#define Bullet_Coefficient 0.038f

/**
 * @brief the half width of little armor
 */
#define LittleArmor_HalfWidth 0.07f

/**
 * @brief the half width of Large armor
 */
#define LargeArmor_HalfWidth 0.1175f

/* IMU reslove constants ---------------------------------------------------*/
/**
 * @brief the flag of bmi088 Calibration
 *        0: DISABLE
 *        1: ENABLE
 */
#define IMU_Calibration_ENABLE 0U

/**
 * @brief the index of pitch angle update
 */
#define IMU_ANGLE_INDEX_PITCH 2U
/**
 * @brief the index of yaw angle update
 */
#define IMU_ANGLE_INDEX_YAW 0U
/**
 * @brief the index of roll angle update
 */
#define IMU_ANGLE_INDEX_ROLL 1U

/**
 * @brief the index of pitch gyro update
 */
#define IMU_GYRO_INDEX_PITCH 0U
/**
 * @brief the index of yaw gyro update
 */
#define IMU_GYRO_INDEX_YAW 2U
/**
 * @brief the index of roll gyro update
 */
#define IMU_GYRO_INDEX_ROLL 1U

/**
 * @brief the index of pitch accel update
 */
#define IMU_ACCEL_INDEX_PITCH 0U
/**
 * @brief the index of yaw accel update
 */
#define IMU_ACCEL_INDEX_YAW 2U
/**
 * @brief the index of roll accel update
 */
#define IMU_ACCEL_INDEX_ROLL 1U

/* Remote reslove constants -----------------------------------------------*/
/**
 * @brief the flag of remote control receive frame data
 * @note  0: CAN
 *        1: USART
 */
#define REMOTE_FRAME_USART_CAN 0U

#define user_malloc pvPortMalloc

/**
 * @brief macro definition of the matrix calculation.
 */
#define Matrix arm_matrix_instance_f32
#define Matrix_64 arm_matrix_instance_f64
#define Matrix_Init arm_mat_init_f32
#define Matrix_Add arm_mat_add_f32
#define Matrix_Subtract arm_mat_sub_f32
#define Matrix_Multiply arm_mat_mult_f32
#define Matrix_Transpose arm_mat_trans_f32
#define Matrix_Inverse arm_mat_inverse_f32
#define Matrix_Inverse_64 arm_mat_inverse_f64

  /* Exported types ------------------------------------------------------------*/
  /**
   * @brief typedef structure that contains the information  for the Chi Square Test.
   */
  typedef struct
  {
    bool TestFlag;                 /*!< Enable/Disable Flag */
    Matrix ChiSquare_Matrix;       /*!< chi square test matrix */
    float ChiSquare_Data[1];       /*!< chi square test matrix data */
    float ChiSquareTestThresholds; /*!< chi square test matrix Thresholds */
    uint8_t ChiSquareCnt;          /*!< chi square test count */
    bool Result;                   /*!< chi square test result */
  } ChiSquareTest_Typedef;

  typedef struct KF_Info_TypeDef
  {
    uint16_t sizeof_float, sizeof_double; /*!< size of float/double */

    uint8_t Xhat_Size; /*!< state vector dimension */
    uint8_t U_Size;    /*!< control vector dimension */
    uint8_t Z_Size;    /*!< measurement vector dimension */

    float dt;              /*!< update cycle */
    float *MeasuredVector; /*!< external measure vector pointer */
    float *ControlVector;  /*!< external control vector pointer */

    ChiSquareTest_Typedef ChiSquareTest; /*!< Chi Square Test */

    /**
     * @brief Instance structure for the floating-point matrix structure.
     */
    struct
    {
      Matrix Xhat;            /*!< posteriori estimate Matrix */
      Matrix Xhatminus;       /*!< priori estimate Matrix */
      Matrix U;               /*!< control vector */
      Matrix Z;               /*!< measurement vector */
      Matrix B;               /*!< control Matrix */
      Matrix A, AT;           /*!< state transition Matrix */
      Matrix H, HT;           /*!< measurement Matrix */
      Matrix P;               /*!< posteriori covariance Matrix */
      Matrix Pminus;          /*!< priori covariance Matrix */
      Matrix Q;               /*!< process noise covariance Matrix */
      Matrix R;               /*!< measurement noise covariance Matrix */
      Matrix K;               /*!< kalman gain Matrix */
      Matrix K_denominator;   /*!< K_denominator Matrix (K_denominator = H Pminus HT + R) */
      Matrix Cache_Matrix[2]; /*!< calculate cache Matrix */
      Matrix Cache_Vector[2]; /*!< calculate cache vector */
    } Mat;

    arm_status MatStatus; /*!< Error status. */

    /**
     * @brief Instance structure for the floating-point matrix data pointer.
     */
    struct
    {
      float *Xhat, *Xhatminus; /*!< posteriori/priori estimate matrix memory pointer */
      float *U;                /*!< control vector memory pointer */
      float *Z;                /*!< measurement vector memory pointer */
      float *B;                /*!< control matrix memory pointer */
      float *A, *AT;           /*!< state transition matrix memory pointer */
      float *H, *HT;           /*!< measurement matrix memory pointer */
      float *P;                /*!< posteriori covariance matrix memory pointer */
      float *Pminus;           /*!< priori covariance matrix memory pointer */
      float *Q;                /*!< process noise covariance matrix memory pointer */
      float *R;                /*!< measurement noise covariance matrix memory pointer */
      float *K;                /*!< kalman gain matrix memory pointer */
      float *K_denominator;    /*!< K_denominator matrix memory pointer */
      float *Cache_Matrix[2];  /*!< calculate cache matrix memory pointer */
      float *Cache_Vector[2];  /*!< calculate cache vector memory pointer */
    } Data;

    uint8_t SkipStep1 : 1; /*!< flag to skip the first step of kalman filter updating */
    uint8_t SkipStep2 : 1; /*!< flag to skip the second step of kalman filter updating */
    uint8_t SkipStep3 : 1; /*!< flag to skip the third step of kalman filter updating */
    uint8_t SkipStep4 : 1; /*!< flag to skip the fourth step of kalman filter updating */
    uint8_t SkipStep5 : 1; /*!< flag to skip the fifth step of kalman filter updating */
    uint8_t reserve : 3;

    /**
     * @brief user functions that can replace any step of kalman filter updating.
     */
    void (*User_Function0)(struct KF_Info_TypeDef *KF);
    void (*User_Function1)(struct KF_Info_TypeDef *KF);
    void (*User_Function2)(struct KF_Info_TypeDef *KF);
    void (*User_Function3)(struct KF_Info_TypeDef *KF);
    void (*User_Function4)(struct KF_Info_TypeDef *KF);
    void (*User_Function5)(struct KF_Info_TypeDef *KF);
    void (*User_Function6)(struct KF_Info_TypeDef *KF);

    float *Output; /*!< kalman filter output */

  } KalmanFilter_Info_TypeDef;

  /* Exported functions prototypes ---------------------------------------------*/
  /**
   * @brief Initializes the kalman filter according to the specified parameters in the KalmanFilter_Info_TypeDef.
   */
  extern void Kalman_Filter_Init(KalmanFilter_Info_TypeDef *KF, uint8_t Xhat_Size, uint8_t U_Size, uint8_t Z_Size);
  /**
   * @brief Update the Kalman Filter according to the specified parameters in the KalmanFilter_Info_TypeDef.
   */
  extern float * Kalman_Filter_Update(KalmanFilter_Info_TypeDef *KF);

#endif 
