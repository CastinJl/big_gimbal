#ifndef RM_REFEREE_H
#define RM_REFEREE_H

#include "usart.h"
#include "referee_protocol.h"
#include "robot_def.h"
#include "bsp_usart.h"
#include "FreeRTOS.h"

extern uint8_t UI_Seq;

// 用于图传遥控器数据读取,遥控器数据是一个大小为2的数组
#define LAST 1
#define TEMP 0
// 获取按键操作
#define KEY_PRESS 0
#define KEY_STATE 1
#define KEY_PRESS_WITH_CTRL 1
#define KEY_PRESS_WITH_SHIFT 2

// 检查接收值是否出错
#define VT_CH_VALUE_MIN ((uint64_t)364)
#define VT_CH_VALUE_OFFSET ((uint16_t)1024)
#define VT_CH_VALUE_MAX ((uint64_t)1684)

// 对应key[x][0~16],获取对应的键;例如通过key[KEY_PRESS][Key_W]获取W键是否按下,后续改为位域后删除
#define Key_W 0
#define Key_S 1
#define Key_D 2
#define Key_A 3
#define Key_Shift 4
#define Key_Ctrl 5
#define Key_Q 6
#define Key_E 7
#define Key_R 8
#define Key_F 9
#define Key_G 10
#define Key_Z 11
#define Key_X 12
#define Key_C 13
#define Key_V 14
#define Key_B 15

//挡位定义
typedef enum {
    MODE_SW_C = 0,
    MODE_SW_N = 1,
    MODE_SW_S = 2
} Mode_sw_Gate;

typedef union
{
    struct // 用于访问键盘状态
    {
        uint16_t w : 1;
        uint16_t s : 1;
        uint16_t a : 1;
        uint16_t d : 1;
        uint16_t shift : 1;
        uint16_t ctrl : 1;
        uint16_t q : 1;
        uint16_t e : 1;
        uint16_t r : 1;
        uint16_t f : 1;
        uint16_t g : 1;
        uint16_t z : 1;
        uint16_t x : 1;
        uint16_t c : 1;
        uint16_t v : 1;
        uint16_t b : 1;
    };
    uint16_t keys; // 用于memcpy而不需要进行强制类型转换
} Key_vt;

typedef struct
{
    struct
    {
        /*-- 帧头 --*/
        uint8_t sof_1;           // 0xA9
        uint8_t sof_2;           // 0x53
        /*-- 摇杆通道 --*/
        int16_t ch_right_x;  // 通道0 右摇杆水平 364-1024-1684
        int16_t ch_right_y;  // 通道1 右摇杆竖直 364-1024-1684
        int16_t ch_left_y;   // 通道2 左摇杆竖直 364-1024-1684
        int16_t ch_left_x;   // 通道3 左摇杆水平 364-1024-1684
        /*-- 控制按钮 --*/
        uint64_t mode_sw;      // 挡位 C:0 N:1 S:2
        uint64_t pause;        // 暂停接收端 OFF:0 ON:1
        uint64_t fn_left;      // 自定义左键 OFF:0 ON:1
        uint64_t fn_right;     // 自定义右键 OFF:0 ON:1
        uint64_t wheel;       // 拨轮 364-1024-1684
        uint64_t trigger;      // 扳机键 OFF:0 ON:1
    } vt;
    struct
    {
        /*-- 鼠标数据 --*/
        int16_t x;         // X轴 向左负值
        int16_t y;         // Y轴 向后负值
        int16_t z;         // Z轴（滚轮） 向后负值
        uint8_t left;    //鼠标左键 OFF:0 ON:1
        uint8_t right;   //鼠标右键 OFF:0 ON:1
        uint8_t middle;  //鼠标中键 OFF:0 ON:1
    } mouse;
    /*-- 键盘按键 --*/
    //uint16_t key;                // 0~65535
    Key_vt key[3]; // 改为位域后的键盘索引,空间减少8倍,速度增加16~倍
    uint16_t key_count[3][16];
    /*-- CRC校验 --*/
    uint16_t crc16;

}VT_ctrl_t;

#pragma pack(1)
typedef struct
{
    uint8_t Robot_Color;		// 机器人颜色
    uint16_t Robot_ID;			// 本机器人ID
    uint16_t Cilent_ID;			// 本机器人对应的客户端ID
    uint16_t Receiver_Robot_ID; // 机器人车间通信时接收者的ID，必须和本机器人同颜色
} referee_id_t;

// 此结构体包含裁判系统接收数据以及UI绘制与机器人车间通信的相关信息
typedef struct
{
    referee_id_t referee_id;

    xFrameHeader FrameHeader; // 接收到的帧头信息
    uint16_t CmdID;
    ext_game_state_t GameState;							   // 0x0001
    ext_game_result_t GameResult;						   // 0x0002
    ext_game_robot_HP_t GameRobotHP;					   // 0x0003
    ext_event_data_t EventData;							   // 0x0101
    //ext_supply_projectile_action_t SupplyProjectileAction; // 0x0102
    ext_game_robot_state_t GameRobotState;				   // 0x0201
    ext_power_heat_data_t PowerHeatData;				   // 0x0202
    ext_game_robot_pos_t GameRobotPos;					   // 0x0203
    ext_buff_musk_t BuffMusk;							   // 0x0204
    //aerial_robot_energy_t AerialRobotEnergy;			   // 0x0205
    ext_robot_hurt_t RobotHurt;							   // 0x0206
    ext_shoot_data_t ShootData;							   // 0x0207
    ext_projectile_allowance_t ProjectileAllowance;        // 0x0208

    // 自定义交互数据的接收
    Communicate_ReceiveData_t ReceiveData;

    uint8_t init_flag;

} referee_info_t;

// 模式是否切换标志位，0为未切换，1为切换，static定义默认为0
typedef struct
{
    uint32_t chassis_flag : 1;
    uint32_t friction_flag : 1;
    uint32_t Buffer_energy_flag : 1;
    uint32_t auto_aim_mode_flag : 1;
    uint32_t auto_aim_fire_suggestion_flag : 1;
    uint32_t cap_energy_flag : 1;
    uint32_t bullet_number_flag : 1;
    uint32_t load_flag : 1;
    uint32_t beat_flag : 1;// 被击中数据变化标志位
} Referee_Interactive_Flag_t;

// 此结构体为图传链路传输的数据
typedef struct
{
    VT_ctrl_t vt_ctrl[2];

    referee_id_t referee_id;
    xFrameHeader FrameHeader; // 接收到的帧头信息
    uint16_t CmdID;

    vision_transfer_t RemoteControl;                       // 0x304
    custom_robot_data_t CustomControl;                     // 0x302

    uint8_t init_flag;

} VT_info_t;

// 此结构体包含UI绘制与机器人车间通信的需要的其他非裁判系统数据
typedef struct
{
    Referee_Interactive_Flag_t Referee_Interactive_Flag;
    // 为UI绘制以及交互数据所用
    chassis_mode_e chassis_mode;			 // 底盘模式
    friction_mode_e friction_mode;			 // 摩擦轮
    uint8_t auto_aim_mode;                        // 自瞄是否开启
    uint8_t auto_aim_fire_suggestion;                        // 自瞄是否建议开火
    loader_mode_e loader_mode;               // 发射模式  单连发
    uint8_t SuperCapEnergy;//超级电容可用能量：0-100%
    uint16_t bullet_number;//剩余子弹数量
    uint8_t armor_beat_id;//被击中部位ID   需要在裁判系统设置   规定前0  逆时针递增  左1 后2 右3


    // 上一次的模式，用于flag判断
    chassis_mode_e chassis_last_mode;
    friction_mode_e friction_last_mode;
    uint8_t auto_aim_last_mode;
    uint8_t auto_aim_fire_last_suggestion;
    loader_mode_e loader_last_mode;
    uint8_t SuperCapLastEnergy;//超级电容可用能量：0-100%
    uint16_t bullet_number_last;
    uint8_t armor_beat_id_last;

} Referee_Interactive_info_t;

#pragma pack()

/**
 * @brief 裁判系统通信初始化,该函数会初始化裁判系统串口,开启中断
 *
 * @param referee_usart_handle 串口handle,C板一般用串口6
 * @return referee_info_t* 返回裁判系统反馈的数据,包括热量/血量/状态等
 */
referee_info_t *RefereeInit(UART_HandleTypeDef *referee_usart_handle);

/**
 * @brief UI绘制和交互数的发送接口,由UI绘制任务和多机通信函数调用
 * @note 内部包含了一个实时系统的延时函数,这是因为裁判系统接收CMD数据至高位10Hz
 *
 * @param send 发送数据首地址
 * @param tx_len 发送长度
 */
void RefereeSend(uint8_t *send, uint16_t tx_len);

#endif // !REFEREE_H
