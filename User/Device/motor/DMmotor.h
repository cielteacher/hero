#ifndef TYPE_MOON_DMMOTOR_H
#define TYPE_MOON_DMMOTOR_H
#include "motor.h" 
/* 电机总数 */
#define DM_MOTOR_CNT 2
/*达妙电机类型枚举*/
typedef enum
{
  DM_4310,
  DM_8009,
  DM_3507,
  NO_DM_TYPE = 0
} DM_MOTOR_TYPE;
/**
 * @brief 达妙电机限位.
 */
typedef struct
{
  float P_MAX;
  float V_MAX;
  float T_MAX;
} DM_Motor_Param_Range_Typedef;

/**
 * @brief typedef structure that contains the data for the DM Motor Device.
 */
typedef struct
{
  int id;                   /*电机id后四位*/
  uint8_t State;            /*!< Motor Message */
  uint16_t P_int;           /*!< Motor Positon  uint16 */
  uint16_t V_int;           /*!< Motor Velocity uint16 */
  uint16_t T_int;           /*!< Motor Torque   uint16 */
  float Position;           /*!< Motor Positon  */
  float Velocity;           /*!< Motor Velocity */
  float Torque;             /*!< Motor Torque   */
  float Temperature_MOS;    /*!< Motor Temperature_MOS   */
  float Temperature_Rotor;  /*!< Motor Temperature_Rotor */
} DM_Motora_Normal_Data;


typedef struct
{
  uint16_t MechanicalAngle; /*!< Motor encoder angle */
  int16_t Current;         /*!< Motor electric current 单位mA*/
  int16_t Velocity;        /*!< Motor rotate velocity (RPM)减速前*/
  uint8_t Temperature;     /*!< Motor Temperature */

  int16_t round;                //!<@brief 圈数
  uint16_t Last_MechanicalAngle; /*!< previous Motor encoder angle */
  int32_t Continuous_Mechanical_angle;
  float Angle;         /*!< Motor angle (degree) */
  float Angle_DEG;     /*!< Motor angle (degree) */
  float SpeedFilter;   //!<@brief 转速 滤波后值
  float CurrentFilter; //!<@brief 转矩电流
} DM_Motor_Data_DJI;

typedef struct
{
  DM_Motora_Normal_Data Normal_Data;
  DM_Motor_Data_DJI DJI_data;
} DM_Motor_Date;
/**
 * @brief 电机设置
 */
typedef struct
{
  DM_MOTOR_TYPE type;                       // 电机类型
  DMMOTOR_CONTROL_TYPE Control_Mode;        // 控制方式
  CAN_Init_Config_s CANFrame;               // CAN配置
  DM_Motor_Param_Range_Typedef Param_Range; // 电机限值
} DM_Motor_Info_Typedef;
// 电机实例
typedef struct
{
  DM_MOTOR_TYPE type;
  DMMOTOR_CONTROL_TYPE Control_Mode;
  CANInstance *CAN_instance;                // CAN实例
  DM_Motor_Param_Range_Typedef Param_Range; // 电机参数限制
  DM_Motor_Date Data;                       // 电机数据
  volatile uint8_t dm_motor_online_flag;
  volatile uint32_t dmmotor_last_rx_time_ms;  
} DM_Motor_Instance;

/**
 * @brief typedef structure that contains the control information for the DM Motor Device .
 */
typedef struct
{
  float Position;
  float Velocity;
  float KP;
  float KD;
  float Torque;
  float Angle;
} DM_Motor_Contorl_info;

extern void DMMotordisable(DM_Motor_Instance *motor);
extern void DMMotorEnable(DM_Motor_Instance *motor);
extern void DMMotorClear(DM_Motor_Instance *instance);
extern DM_Motor_Instance *DM_Motor_Init(DM_Motor_Info_Typedef *_config);

extern void DM_Motor_DJI_CAN_TxMessage(DM_Motor_Instance *DM_Motor, int16_t *txbuffer);
extern void DM_Motor_MIT_CAN_TxMessage(DM_Motor_Instance *DM_Motor, float pos, float vel, float kp, float kd, float tor);
extern void DM_Motor_AliveCheck(void);

#endif // TYPE_MOON_DMMOTOR
