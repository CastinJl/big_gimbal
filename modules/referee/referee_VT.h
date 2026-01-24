//
// Created by zyx12 on 2025/4/12.
//

#ifndef UAV_REFEREE_VT_H
#define UAV_REFEREE_VT_H
#include "rm_referee.h"
/**
 * @brief 图传链路通信初始化,该函数会初始化裁判系统串口,开启中断
 *
 * @param referee_usart_handle 串口handle,C板一般用串口1（丝印为uart2）
 * @return VT_info_t* 返回图传链路发来的数据,包括键鼠、自定义控制器
 */
VT_info_t *VTInit(UART_HandleTypeDef *VT_usart_handle);

#endif //UAV_REFEREE_VT_H
