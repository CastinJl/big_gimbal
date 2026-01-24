//
// Created by ydwzxcv on 2025/2/27.
//

#ifndef BASIC_FRAMEWORK_POWER_H
#define BASIC_FRAMEWORK_POWER_H


#include "usart.h"
#include "referee_protocol.h"
#include "robot_def.h"
#include "bsp_usart.h"
#include "FreeRTOS.h"
#include "remote_control.h"




typedef struct
{
        float voltage;
        float current;
        float power;
        float energy;
} Power_data;

// 此结构体包含裁判系统接收数据以及UI绘制与机器人车间通信的相关信息

/**
 * @brief 裁判系统通信初始化,该函数会初始化裁判系统串口,开启中断
 *
 * @param power_usart_handle 串口handle,C板一般用串口6
 * @return referee_info_t* 返回裁判系统反馈的数据,包括热量/血量/状态等
 */
Power_data *PowerInit(UART_HandleTypeDef *power_usart_handle);

/**
 * @brief UI绘制和交互数的发送接口,由UI绘制任务和多机通信函数调用
 * @note 内部包含了一个实时系统的延时函数,这是因为裁判系统接收CMD数据至高位10Hz
 *
 * @param send 发送数据首地址
 * @param tx_len 发送长度
 */
void PowerSend(uint8_t *send, uint16_t tx_len);

#endif //BASIC_FRAMEWORK_POWER_H

