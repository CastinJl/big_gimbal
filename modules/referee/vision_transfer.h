// #ifndef VISION_TRANSFER_H
// #define VISION_TRANSFER_H
//
// #include "usart.h"
// #include "referee_protocol.h"
// #include "robot_def.h"
// #include "bsp_usart.h"
// #include "FreeRTOS.h"
// #include "remote_control.h"
//
// #pragma pack(1)
//
// // 此结构体包含裁判系统接收数据以及UI绘制与机器人车间通信的相关信息
// typedef struct
// {
//
//     xFrameHeader FrameHeader; // 接收到的帧头信息
//     uint16_t CmdID;
//
//     vision_transfer_t  VisionTransfer;
//     Communicate_ReceiveData_t ReceiveData;
//
//     uint8_t init_flag;
//
// } referee_info_vt_t;
//
// #pragma pack()
//
// /**
//  * @brief 裁判系统通信初始化,该函数会初始化裁判系统串口,开启中断
//  *
//  * @param vt_usart_handle 串口handle,C板一般用串口6
//  * @return referee_info_t* 返回裁判系统反馈的数据,包括热量/血量/状态等
//  */
// VT_ctrl_t *VTRefereeInit(UART_HandleTypeDef *referee_usart_handle);
//
// /**
//  * @brief UI绘制和交互数的发送接口,由UI绘制任务和多机通信函数调用
//  * @note 内部包含了一个实时系统的延时函数,这是因为裁判系统接收CMD数据至高位10Hz
//  *
//  * @param send 发送数据首地址
//  * @param tx_len 发送长度
//  */
// void VTRefereeSend(uint8_t *send, uint16_t tx_len);
//
// #endif // !REFEREE_H
