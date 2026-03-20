/**
 * @file robot_def.h
 * @author NeoZeng neozng1@hnu.edu.cn
 * @author Even
 * @version 0.1
 * @date 2022-12-02
 *
 * @copyright Copyright (c) HNU YueLu EC 2022 all rights reserved
 *
 */
#pragma once // 可以用#pragma once代替#ifndef ROBOT_DEF_H(header guard)
#ifndef ROBOT_DEF_H
#define ROBOT_DEF_H

#include "ins_task.h"
#include "master_process.h"
#include "stdint.h"

/* 开发板类型定义,烧录时注意不要弄错对应功能;修改定义后需要重新编译,只能存在一个定义! */
// #define ONE_BOARD // 单板控制整车
// #define CHASSIS_BOARD //底盘板
#define GIMBAL_BOARD  //云台板

#define VISION_USE_VCP  // 使用虚拟串口发送视觉数据
// #define VISION_USE_UART // 使用串口发送视觉数据

/* 机器人重要参数定义,注意根据不同机器人进行修改,浮点数需要以.0或f结尾,无符号以u结尾 */
// 云台参数
#define YAW_CHASSIS_ALIGN_RAD -1.237// 云台和底盘对齐指向相同方向时的电机position值,若对云台有机械改动需要修改
#define YAW_ECD_GREATER_THAN_4096 0 // ALIGN_ECD值是否大于4096,是为1,否为0;用于计算云台偏转角度
#define MAX_FF_SPEED_RAD 20
#define PITCH_HORIZON_ECD 3742    // 云台处于水平位置时编码器值,若对云台有机械改动需要修改
#define PITCH_MAX_ANGLE 35           // 云台竖直方向最大角度 (注意反馈如果是陀螺仪，则填写陀螺仪的角度)
#define PITCH_MIN_ANGLE -21          // 云台竖直方向最小角度 (注意反馈如果是陀螺仪，则填写陀螺仪的角度)

#define PITCH_MAX_RELATIVE_ANGLE 123        // 云台相对底盘最大角度
#define PITCH_MIN_RELATIVE_ANGLE 80          // 云台相对底盘最小角度

// 发射参数
#define ONE_BULLET_DELTA_ANGLE 40    // 发射一发弹丸拨盘转动的距离,由机械设计图纸给出
#define REDUCTION_RATIO_LOADER 27.142857f //  = 拨盘电机的减速比 * 拨盘齿数 / 电机轴齿数，其中电机减速比：3508-19.0f 2006-36.0f
#define HEAT_PER_SHOT 10       // 每发子弹热量增量（根据实际弹药参数调整）
#define COOLING_RATE 20        // 每秒冷却热量（单位：热量单位/秒）
#define ONE_BULLET_DELTA_ANGLES (360.0f / 9.0f) // 每发弹丸对应角度（8发/圈）
#define NUM_PER_CIRCLE 9            // 拨盘一圈的装载量
#define TOTAL_BULLET 500    // 发射一发弹丸拨盘转动的距离,由机械设计图纸给出

// 机器人底盘修改的参数,单位为m(米)
#define WHEEL_BASE 0.1f              // 纵向轴距(前进后退方向)
#define TRACK_WIDTH 0.1f              // 横向轮距(左右平移方向)
#define CENTER_GIMBAL_OFFSET_X 0    // 云台旋转中心距底盘几何中心的距离,前后方向,云台位于正中心时默认设为0
#define CENTER_GIMBAL_OFFSET_Y 0    // 云台旋转中心距底盘几何中心的距离,左右方向,云台位于正中心时默认设为0
#define RADIUS_WHEEL 0.08f            // 轮子半径
#define REDUCTION_RATIO_WHEEL 19.0f // 电机减速比,因为编码器量测的是转子的速度而不是输出轴的速度故需进行转换

#define GYRO2GIMBAL_DIR_YAW 1   // 陀螺仪数据相较于云台的yaw的方向,1为相同,-1为相反
#define GYRO2GIMBAL_DIR_PITCH 1 // 陀螺仪数据相较于云台的pitch的方向,1为相同,-1为相反
#define GYRO2GIMBAL_DIR_ROLL 1  // 陀螺仪数据相较于云台的roll的方向,1为相同,-1为相反

// 检查是否出现主控板定义冲突,只允许一个开发板定义存在,否则编译会自动报错
#if (defined(ONE_BOARD) && defined(CHASSIS_BOARD)) || \
    (defined(ONE_BOARD) && defined(GIMBAL_BOARD)) ||  \
    (defined(CHASSIS_BOARD) && defined(GIMBAL_BOARD))
#error Conflict board definition! You can only define one board type.
#endif

#pragma pack(1) // 压缩结构体,取消字节对齐,下面的数据都可能被传输
/* -------------------------基本控制模式和数据类型定义-------------------------*/
typedef struct {
    float heat_limit;       // 热量上限
    float cooling_rate;     // 冷却速率(/s)
    float heat_per_bullet;  // 每发子弹热量(固定10)
    float current_heat;     // 当前热量
    float shoot_rate;       // 当前射速(发/秒)
    float last_heat_time; // 上次热量更新时间
    uint8_t overheat_flag;  // 超热标志
} HeatControl_t;

typedef enum {
    ST_CLOSE,     // 关机状态
    ST_OPEN       // 工作状态
} FSM_State;

typedef struct {
    float* buf;         // 队列存储数组
    int cap;            // 队列容量
    int size;           // 当前大小
    int front, rear;    // 头尾指针
} CurrentQueue;

typedef struct {
    FSM_State state;            // 当前状态
    CurrentQueue current_q;      // 电流差值队列
    float q_sum;                // 队列总和
    float last_q_sum;                // 队列总和
    float error_q_sum;                // 队列总和

    float now_heat;             // 当前热量
    uint32_t total_ammo;        // 总弹药计数
    float q_threshold;          // 触发阈值（可动态调整）
} HeatDetector;
typedef struct {
    // 状态跟踪
    uint8_t last_trigger_state;   // 上一次触发状态
    uint32_t last_trigger_time;// 最后一次触发时间戳

    // 动态参数
    float noise_floor;         // 噪声基线
    float dynamic_threshold;   // 动态触发阈值

    // 统计
    uint32_t trigger_count;    // 总触发次数
} TriggerDetector;
/**
 * @brief 这些枚举类型和结构体会作为CMD控制数据和各应用的反馈数据的一部分
 *
 */
// 机器人状态
typedef enum
{
    ROBOT_STOP = 0,
    ROBOT_READY,
} Robot_Status_e;

// 应用状态
typedef enum
{
    APP_OFFLINE = 0,
    APP_ONLINE,
    APP_ERROR,
} App_Status_e;


// 底盘模式设置
/**
 * @brief 后续考虑修改为云台跟随底盘,而不是让底盘去追云台,云台的惯量比底盘小.
 *
 */
typedef enum
{
    CHASSIS_ZERO_FORCE = 0,    // 电流零输入
        CHASSIS_ROTATE,            // 小陀螺模式
        CHASSIS_NO_FOLLOW,         // 不跟随，允许全向平移
        CHASSIS_FOLLOW_GIMBAL_YAW, // 跟随模式，底盘叠加角度环控制
        CHASSIS_SIDEWAYS,          // 侧向
} chassis_mode_e;

// 云台模式设置
typedef enum
{
    GIMBAL_ZERO_FORCE = 0, // 电流零输入
    GIMBAL_FREE_MODE,      // 云台自由运动模式,即与底盘分离(底盘此时应为NO_FOLLOW)反馈值为电机total_angle;似乎可以改为全部用IMU数据?
    GIMBAL_GYRO_MODE,      // 云台陀螺仪反馈模式,反馈值为陀螺仪pitch,total_yaw_angle,底盘可以为小陀螺和跟随模式
} gimbal_mode_e;

// 发射模式设置
typedef enum
{
    SHOOT_OFF = 0,
    SHOOT_ON,
} shoot_mode_e;
typedef enum
{
    FRICTION_OFF = 0, // 摩擦轮关闭
    FRICTION_ON,      // 摩擦轮开启
} friction_mode_e;

typedef enum
{
    LID_OPEN , // 弹舱盖打开
    LID_CLOSE,    // 弹舱盖关闭
} lid_mode_e;

typedef enum
{
    LOAD_STOP = 0,  // 停止发射
    LOAD_REVERSE,   // 反转
    LOAD_1_BULLET,  // 单发
    LOAD_3_BULLET,  // 三发
    LOAD_BURSTFIRE, // 连发
} loader_mode_e;
typedef enum
{
    HEAT_OPEN , // 热控打开
    HEAT_CLOSE, // 热控关闭
}heat_mode_e;
// 功率限制,从裁判系统获取,是否有必要保留?
typedef struct
{
    uint16_t chassis_power_mx; // 功率限制
} Chassis_Power_Data_s;

// 电容信息
typedef struct
{
    int16_t cap_vol;
} Cap_Data_s;
/* ----------------CMD应用发布的控制数据,应当由gimbal/chassis/shoot订阅---------------- */
/**
 * @brief 对于双板情况,遥控器和pc在云台,裁判系统在底盘
 *
 */
// cmd发布的底盘控制数据,由chassis订阅
typedef struct
{
    // 控制部分
    float vx;           // 前进方向速度
    float vy;           // 横移方向速度
    float offset_angle; // 底盘和归中位置的夹角  单位rad
    chassis_mode_e chassis_mode;
    uint8_t cap_open;
    uint8_t heat_open; //发射热控标志位，1为热控开，0为热控关
    // UI部分
    friction_mode_e friction_mode;
    uint8_t auto_aim_mode;
    uint8_t auto_aim_fire_suggestion;
    uint8_t is_running_fire; //单发、连发标志位，1为连发，0为单发
    uint8_t ui_refresh;
    uint8_t chassis_rotate_reverse_flag;
} Chassis_Ctrl_Cmd_s;

// cmd发布的云台控制数据,由gimbal订阅
typedef struct
{ // 云台角度控制
    float yaw;
    float pitch;
    float chassis_rotate_wz;
    float shoot_rate; // 连续发射的射频,unit per s,发/秒
    float offset_angle; // 底盘和归中位置的夹角
    float real_wz; // 底盘和归中位置的夹角
    float auto_aim_mode;
    float vision_yaw_speed_ff;
    float vision_pitch_speed_ff;
    gimbal_mode_e gimbal_mode;
} Gimbal_Ctrl_Cmd_s;

// cmd发布的发射控制数据,由shoot订阅
typedef struct
{
    shoot_mode_e shoot_mode;
    loader_mode_e loader_mode;
    lid_mode_e lid_mode;
    friction_mode_e friction_mode;
    heat_mode_e heat_mode;
    Bullet_Speed_e bullet_speed; // 弹速枚举
    float shoot_rate; // 连续发射的射频,unit per s,发/秒

    uint16_t heat;       // 实时热量
    uint16_t heat_limit;  // 热量上限
    uint16_t heat_cool;  // 热量每秒冷却值
    uint8_t robot_level;
    uint8_t aim_flag;
} Shoot_Ctrl_Cmd_s;

/* ----------------gimbal/shoot/chassis发布的反馈数据----------------*/
/**
 * @brief 由cmd订阅,其他应用也可以根据需要获取.
 *
 */

typedef struct
{
#if defined(CHASSIS_BOARD) || defined(GIMBAL_BOARD) // 非单板的时候底盘还将imu数据回传(若有必要)
    // attitude_t chassis_imu_data;
#endif
    float bullet_speed;
    uint8_t robot_level;
    uint16_t heat_limit;
    uint8_t dead_state;
    uint16_t heat;
    float real_wz;
    Enemy_Color_e enemy_color;   // 0 for blue, 1 for red
    uint16_t cooling_speed;
} Chassis_Upload_Data_s;


typedef struct
{
    attitude_t gimbal_imu_data;
    float yaw_motor_single_round_angle;
    float yaw_motor_sum_round_angle;

} Gimbal_Upload_Data_s;

typedef struct
{
    // code to go here
    // ...
    uint8_t stalled_flag;  //堵转标志
    uint8_t over1;
    uint8_t over2;
    uint8_t over3;

} Shoot_Upload_Data_s;

#pragma pack() // 开启字节对齐,结束前面的#pragma pack(1)

#endif // !ROBOT_DEF_H