#ifndef REFEREE_H
#define REFEREE_H
/* Includes ------------------------------------------------------------------*/
#include "RMLibHead.h"
#include "RMQueue.h"
#include "bsp_usart.h"
#include "stdbool.h"
#include "stdint.h"
#include "bsp_dwt.h"
/* defines ----------------------------------------------------------*/
#define REFEREE_RXDATA_LENGTH 136U          //裁判系统数据最大长度
//数据帧长度定义
#define FrameHeader_Length    5U   //帧头长度
#define CMDID_Length          2U   //命令ID长度
#define CRC16_Length          2U   // 帧尾CRC16校验长度
//命令ID定义
#define Referee_Command_ID_GAME_STATUS  0x0001U // 比赛状态数据
#define Referee_Command_ID_EVENT_SELF_DATA 0x0101U // 场地事件数据
#define Referee_Command_ID_EVENT_SELF_REFEREE_WARNING 0x0104U // 裁判警告数据
#define Referee_Command_ID_ROBOT_STATUS 0x0201U               // 机器人状态数据
#define Referee_Command_ID_ROBOT_POWER_HEAT 0x0202U           // 机器人热量数据
#define Referee_Command_ID_ROBOT_BUFF 0x0204U                // 机器人增益数据
#define Referee_Command_ID_ROBOT_BOOSTER 0x0207U             // 机器人发射机构状态数据
#define Referee_Command_ID_INTERACTION 0x0301U               // 交互数据
#define Referee_Command_ID_INTERACTION_ROBOT_RECEIVE_CLIENT_REMOTE_CONTROL 0x304U // 键鼠遥控数据
/* structures -------------------------------------------------------*/
//貌似条件编译写错误位置了，因该写在结构体定义里的
#pragma pack(1)
#ifdef Referee_Command_ID_GAME_STATUS
/**
 * @brief typedef structure that contains the information of game status, id: 0x0001U
 */
typedef struct
{
    /**
     * @brief the type of game,
              1:RMUC,
              2:RMUT,
              3:RMUA,
              4:RMUL,3v3,
              5:RMUL,1v1,
     */
    uint8_t game_type : 4;
    uint8_t game_progress : 4;  /*!< the progress of game */
    uint16_t stage_remain_time; /*!< remain time of real progress */
    uint64_t SyncTimeStamp;     /*!< unix time */

} game_status_t;
#endif
#ifdef Referee_Command_ID_EVENT_SELF_DATA
/**
 * @brief typedef structure that contains the information of site event data, id: 0x0101U
 */
typedef struct
{
    /**
     * @brief the event of site
                bits 0-2:
              bit 0:  Status of the supply zone that does not overlap with the exchange zone, 1 for occupied
              bit 1:  Status of the supply zone that overlaps with the exchange zone, 1 for occupied
              bit 2:  Status of the supply zone, 1 for occupied (applicable only to RMUL)

                bits 3-5: Status of the energy mechanism:
              bit 3:    Status of the small energy mechanism, 1 for activated
              bit 4:    Status of the large energy mechanism, 1 for activated
              bit 5-6:  Status of the central highland, 1 for occupied by own side, 2 for occupied by the opponent
              bit 7-8:  Status of the trapezoidal highland, 1 for occupied
              bit 9-17: Time of the opponent's last dart hit on the own side's outpost or base (0-420, default is 0 at the start)
                bit 18-20:  Specific target of the opponent's last dart hit on the own side's outpost or base, default is 0 at the start,
                            1 for hitting the outpost, 2 for hitting the fixed target on the base, 3 for hitting the random fixed target on the base,
                            4 for hitting the random moving target on the base
                bit 21-22: Status of the central gain point, 0 for unoccupied, 1 for occupied by own side,
                           2 for occupied by the opponent, 3 for occupied by both sides (applicable only to RMUL)
              bit 23-31: Reserved

    */
    uint32_t event_data;
} event_data_t;
#endif
#ifdef Referee_Command_ID_EVENT_SELF_REFEREE_WARNING
/**
 * @brief typedef structure that contains the warning  of Referee , id: 0x0104U
 */
typedef struct
{
    /**
   * @brief the type of game,
            1: Double Yellow Card,
            2: Yellow Card,
            3:RMUA,
            4:RMUL,3v3,
            5:RMUL,1v1,
   */
    uint8_t level;
    uint8_t offending_robot_id;
    uint8_t count;
} referee_warning_t;
#endif
#ifdef Referee_Command_ID_ROBOT_STATUS
/**
 * @brief typedef structure that contains the information of robot status, id: 0x0201U
 */
typedef struct
{
    /**
     * @brief robot id
     *             0: robot none
     *             1: red hero
     *             2: red engineer
     *         3/4/5: red infantry
     *             6: red aerial
     *             7: red sentry
     *             8: red dart
     *             9: red radar station
     *           101: blue hero
     *           102: blue engineer
     *   103/104/105: blue infantry
     *           106: blue aerial
     *           107: blue sentry
     *           108: blue dart
     *           109: blue radar station
     */
    uint8_t robot_id;
    uint8_t robot_level;
    uint16_t current_HP;
    uint16_t maximum_HP;

    uint16_t shooter_barrel_cooling_value;
    uint16_t shooter_barrel_heat_limit;
    uint16_t chassis_power_limit;

    uint8_t mains_power_gimbal_output : 1;
    uint8_t mains_power_chassis_output : 1;
    uint8_t mains_power_shooter_output : 1;
} robot_status_t;

#endif
#ifdef Referee_Command_ID_ROBOT_POWER_HEAT
/**
 * @brief typedef structure that contains the information of power heat data, id: 0x0202U
 */
typedef struct
{
    uint16_t buffer_energy;
    uint16_t shooter_17mm_1_barrel_heat;
    uint16_t shooter_17mm_2_barrel_heat;
    uint16_t shooter_42mm_barrel_heat;
} power_heat_data_t;
#endif
#ifdef Referee_Command_ID_ROBOT_BUFF
/**
 * @brief typedef structure that contains the information of robot buff data, id: 0x0204U
 */
typedef struct
{
    uint8_t recovery_buff;
    uint8_t cooling_buff;
    uint8_t defence_buff;
    uint8_t vulnerability_buff;
    uint16_t attack_buff;
    uint8_t remaining_energy;
} buff_t;
#endif
#ifdef Referee_Command_ID_ROBOT_BOOSTER
/**
 * @brief typedef structure that contains the information of real shoot data, id: 0x0207U
 */
typedef struct
{
    uint8_t bullet_type;
    uint8_t shooter_number;
    uint8_t launching_frequency;
    float initial_speed;
} shoot_data_t;
#endif
#ifdef Referee_Command_ID_INTERACTION
/**
 * @brief typedef structure that contains the information of custom controller interactive, id: 0x0301U
 */
typedef struct
{
    uint16_t data_cmd_id;
    uint16_t sender_id;
    uint16_t receiver_id;
    uint8_t user_data[113];
} robot_interaction_data_t;
#endif
#ifdef Referee_Command_ID_INTERACTION_ROBOT_RECEIVE_CLIENT_REMOTE_CONTROL
/**
 * @brief typedef structure that contains the information of client transmit data, id: 0x0303U
 */
typedef struct
{
    /**
     * @brief target position coordinate, is 0 when transmit target robot id
     */
    float target_position_x;
    float target_position_y;
    float target_position_z;

    uint8_t commd_keyboard;
    uint16_t target_robot_ID; /* is 0 when transmit position data */
} ext_robot_command_t;
#endif
#pragma pack()
typedef struct
{
    RMQueue_Handle rx_queue;           // 循环队列存放数据
    USARTInstance *USART_instance;      // 实例对应的uart_handle
    volatile uint8_t referee_online_flag; // 在线标志
    volatile uint32_t referee_last_rx_ms;// 上次接收数据的时间戳
    game_status_t referee_game_status; // 比赛状态数据
    event_data_t referee_event_data; // 场地事件数据
    referee_warning_t referee_warning_data; // 裁判警告数据
    robot_status_t referee_robot_status;   // 机器人状态数据
    power_heat_data_t referee_power_heat_data; // 机器人热量数据
    buff_t referee_buff_data;                  // 机器人增益数据
    shoot_data_t referee_shoot_data;               // 机器人发射机构状态数据
    robot_interaction_data_t referee_interaction_data; // 交互数据
    ext_robot_command_t referee_mouse_key_cmd; // 键鼠指令
} Referee_instance_t;

/* marco--------------------------------------------------------------*/
void Referee_Init(void);
void Referee_AliveCheck(void);
//extern
extern Referee_instance_t Referee_instance;
#endif // REFEREE_H
