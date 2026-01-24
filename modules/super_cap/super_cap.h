#ifndef SUPER_CAP_H
#define SUPER_CAP_H

#include "bsp_can.h"
#include "controller.h"

#pragma pack(1)
typedef struct
{
    int16_t input_vol; // 输入电压
    int16_t cap_vol;   // 电容电压
    int16_t input_cur; // 输入电流
    int16_t power_set; // 设定功率
} SuperCap_Msg_s;
#pragma pack()

/* 超级电容实例 */
typedef struct
{
    CANInstance *can_ins;   // CAN实例
    SuperCap_Msg_s cap_msg; // 超级电容信息
    PIDInstance buffer_pid; // 对缓冲功率进行闭环
} SuperCapInstance;

/* 超级电容初始化配置 */
typedef struct
{
    CAN_Init_Config_s can_config;
    PID_Init_Config_s buffer_config_pid;
} SuperCap_Init_Config_s;

/**
 * @brief 初始化超级电容
 *
 * @param supercap_config 超级电容初始化配置
 * @return SuperCapInstance* 超级电容实例指针
 */
SuperCapInstance *SuperCapInit(SuperCap_Init_Config_s *supercap_config);

/**
 * @brief 发送超级电容控制信息
 *
 * @param instance 超级电容实例
 * @param data 超级电容控制信息
 */
void SuperCapSend(SuperCapInstance *instance, uint8_t *data);

/**
 * @brief 发送超级电容目标功率
 *
 * @param instance 超级电容实例
 * @param power_set 超级电容目标功率
 */
void SuperCapSetPower(SuperCapInstance *instance, float power_set);

#endif // !SUPER_CAP_Hd
