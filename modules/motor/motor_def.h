/**
 * @file motor_def.h
 * @author neozng
 * @brief  电机通用的数据结构定义
 * @version beta
 * @date 2022-11-01
 *
 * @copyright Copyright (c) 2022 HNU YueLu EC all rights reserved
 *
 */

#ifndef MOTOR_DEF_H
#define MOTOR_DEF_H

#include "controller.h"
#include "stdint.h"
#include "bsp_can.h"

#define LIMIT_MIN_MAX(x, min, max) (x) = (((x) <= (min)) ? (min) : (((x) >= (max)) ? (max) : (x)))

/**
 * @brief 闭环类型,如果需要多个闭环,则使用或运算
 *        例如需要速度环和电流环: CURRENT_LOOP|SPEED_LOOP
 */
typedef enum
{
    CURRENT_LOOP            = 0b0000,  // 开环
    SPEED_LOOP              = 0b0001,  // bit0速度环
    ANGLE_LOOP              = 0b0010,  // bit1角度环
    ANGLE_SPEED_CASCADE     = 0b0011,  // 串级
    ANGLE_SPEED_PARALLEL    = 0b0111,  // bit3并级标志
} Closeloop_Type_e;

typedef enum
{
    FEEDFORWARD_NONE = 0b00,
    CURRENT_FEEDFORWARD = 0b01,
    SPEED_FEEDFORWARD = 0b10,
    CURRENT_AND_SPEED_FEEDFORWARD = CURRENT_FEEDFORWARD | SPEED_FEEDFORWARD,
} Feedfoward_Type_e;

/* 反馈来源设定,若设为OTHER_FEED则需要指定数据来源指针,详见Motor_Controller_s*/
typedef enum
{
    MOTOR_FEED = 0,
    OTHER_FEED,
} Feedback_Source_e;

/* 电机正反转标志 */
typedef enum
{
    MOTOR_DIRECTION_NORMAL = 0,
    MOTOR_DIRECTION_REVERSE = 1
} Motor_Reverse_Flag_e;

/* 反馈量正反标志 */
typedef enum
{
    FEEDBACK_DIRECTION_NORMAL = 0,
    FEEDBACK_DIRECTION_REVERSE = 1
} Feedback_Reverse_Flag_e;
typedef enum
{
    MOTOR_STOP = 0,
    MOTOR_ENALBED = 1,
} Motor_Working_Type_e;

typedef enum
{
    NO_POWER_LIMIT = 0,
    POWER_LIMIT_ON = 1,
} Power_Limit_Type_e;

/* 电机控制设置,包括闭环类型,反转标志和反馈来源 */
typedef struct
{
    Closeloop_Type_e loop_type;              // 闭环类型
    Motor_Reverse_Flag_e motor_reverse_flag;       // 是否反转
    Feedback_Reverse_Flag_e feedback_reverse_flag; // 反馈是否反向
    Feedback_Source_e angle_feedback_source;       // 角度反馈类型
    Feedback_Source_e speed_feedback_source;       // 速度反馈类型
    Feedfoward_Type_e feedforward_flag;            // 前馈标志
    Power_Limit_Type_e power_limit_flag;            //功率限制标志

} Motor_Control_Setting_s;

/* 电机控制方式枚举 */
typedef enum
{
    CONTROL_TYPE_NONE = 0,
    CURRENT_CONTROL,
    VOLTAGE_CONTROL,
} Motor_Control_Type_e;


/* 电机控制器,包括其他来源的反馈数据指针,3环控制器和电机的参考输入*/
// 后续增加前馈数据指针
typedef struct
{
    float *other_angle_feedback_ptr; // 其他反馈来源的反馈数据指针
    float *other_speed_feedback_ptr;
    float *angle_ref_ptr;            // 并级角度环目标值指针
    float *speed_ref_ptr;            // 并级速度环目标值指针
    float *speed_feedforward_ptr;
    float *current_feedforward_ptr;

    PIDInstance current_PID;
    PIDInstance speed_PID;
    PIDInstance angle_PID;

    float pid_ref;     // 外部设定值,串级控制时作为最外环输入
    float control_out; // 停机处理和限幅后的最终电机控制命令

    float motor_power_max;     //每个电机分配的功率上限
    float motor_power_predict; //根据模型预测的电机功率
    float motor_power_scale;   //电机功率缩放比例
} Motor_Controller_s;

/* 电机类型枚举 */
typedef enum
{
    MOTOR_TYPE_NONE = 0,
    GM6020,
    M3508,
    M2006,
    LK9025,
    HT04,
    DM4310,
} Motor_Type_e;

/**
 * @brief 电机控制器初始化结构体,包括三环PID的配置以及两个反馈数据来源指针
 *        如果不需要某个控制环,可以不设置对应的pid config
 *        需要其他数据来源进行反馈闭环,不仅要设置这里的指针还需要在Motor_Control_Setting_s启用其他数据来源标志
 */
typedef struct
{
    float *other_angle_feedback_ptr; // 角度反馈数据指针,注意电机使用total_angle
    float *other_speed_feedback_ptr; // 速度反馈数据指针,单位为angle per sec

    float *angle_ref_ptr; // 并级角度环目标值指针,单位应与角度PID反馈一致
    float *speed_ref_ptr; // 并级速度环目标值指针,单位应与速度PID反馈一致

    float *speed_feedforward_ptr;   // 速度前馈数据指针
    float *current_feedforward_ptr; // 电流前馈数据指针

    PID_Init_Config_s current_PID;
    PID_Init_Config_s speed_PID;
    PID_Init_Config_s angle_PID;
} Motor_Controller_Init_s;

/* 用于初始化CAN电机的结构体,各类电机通用 */
typedef struct
{
    Motor_Controller_Init_s controller_param_init_config;
    Motor_Control_Setting_s controller_setting_init_config;
    Motor_Type_e motor_type;
    CAN_Init_Config_s can_init_config;
    Motor_Control_Type_e motor_control_type;
    float DM_T_MAX;
    float DM_V_MAX;
    float DM_P_MAX;
} Motor_Init_Config_s;

#endif // !MOTOR_DEF_H
