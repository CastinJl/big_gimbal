//
// Created by zyx12 on 2025/4/12.
//
#include "referee_VT.h"

/**
 * @file rm_referee.C
 * @author kidneygood (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2022-11-18
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "rm_referee.h"
#include "string.h"
#include "crc_ref.h"
#include "bsp_usart.h"
#include "task.h"
#include "daemon.h"
#include "bsp_log.h"
#include "cmsis_os.h"

#define VT_RE_RX_BUFFER_SIZE 255u // 图传接收缓冲区大小

static USARTInstance *VT_usart_instance; // 图传串口实例
static DaemonInstance *VT_daemon;		 // 图传守护进程
static VT_info_t VT_info;			     // 图传数据
uint8_t FLAG_CRC;
/**
 * @brief  读取图传链路数据,中断中读取保证速度
 * @param  buff: 读取到的裁判系统原始数据
 * @retval 是否对正误判断做处理
 * @attention  在此判断帧头和CRC校验,无误再写入数据，不重复判断帧头
 */
static void VTJudgeReadData(uint8_t *buff)
{
    uint16_t judge_length; // 统计一帧数据长度
    if (buff == NULL)	   // 空数据包，则不作任何处理
        return;
    // 写入帧头数据(5-byte),用于判断是否开始存储裁判数据
    memcpy(&VT_info.FrameHeader, buff, LEN_HEADER);
    // 判断帧头数据(0)是否为0xA5
    if (buff[SOF] == REFEREE_SOF)
    {
        FLAG_CRC = 3;
        // 帧头CRC8校验
        if (Verify_CRC8_Check_Sum(buff, LEN_HEADER) == TRUE)
        {
            FLAG_CRC = 1;
            // 统计一帧数据长度(byte),用于CR16校验
            judge_length = buff[DATA_LENGTH] + LEN_HEADER + LEN_CMDID + LEN_TAIL;
            // 帧尾CRC16校验
            if (Verify_CRC16_Check_Sum(buff, judge_length) == TRUE)
            {
                FLAG_CRC = 5;
                // 2个8位拼成16位int
                VT_info.CmdID = (buff[6] << 8 | buff[5]);
                // 解析数据命令码,将数据拷贝到相应结构体中(注意拷贝数据的长度)
                // 第8个字节开始才是数据 data=7
                switch (VT_info.CmdID)
                {
                case ID_remote_control: // 0x0304
                    memcpy(&VT_info.RemoteControl, (buff + DATA_Offset), LEN_remote_control);
                    break;
                case ID_custom_control: // 0x0302
                    memcpy(&VT_info.CustomControl, (buff + DATA_Offset), LEN_custom_control);
                    break;
                }
            }
        }
        // 首地址加帧长度,指向CRC16下一字节,用来判断是否为0xA5,从而判断一个数据包是否有多帧数据
        if (*(buff + sizeof(xFrameHeader) + LEN_CMDID + VT_info.FrameHeader.DataLength + LEN_TAIL) == 0xA5)
        { // 如果一个数据包出现了多帧数据,则再次调用解析函数,直到所有数据包解析完毕
            VTJudgeReadData(buff + sizeof(xFrameHeader) + LEN_CMDID + VT_info.FrameHeader.DataLength + LEN_TAIL);
        }
    }
    else if(buff[0] == 0xA9 && buff[1] == 0x53)
    {
        if (Verify_CRC16_Check_Sum(buff, 21) == TRUE)//遥控帧帧尾CRC校验
        {
            FLAG_CRC = 2;
            VT_info.vt_ctrl[TEMP].vt.sof_1 = buff[0];
            VT_info.vt_ctrl[TEMP].vt.sof_2 = buff[1];
            VT_info.vt_ctrl[TEMP].vt.ch_right_x =((buff[2] | (buff[3] << 8)) & 0x07ff) - VT_CH_VALUE_OFFSET;                             //通道0
            VT_info.vt_ctrl[TEMP].vt.ch_right_y =(((buff[3] >> 3) | (buff[4] << 5)) & 0x07ff) - VT_CH_VALUE_OFFSET;                      //通道1
            VT_info.vt_ctrl[TEMP].vt.ch_left_y =(((buff[4] >> 6) | (buff[5] << 2) | (buff[6] << 10)) & 0x07ff) - VT_CH_VALUE_OFFSET;     //通道2
            VT_info.vt_ctrl[TEMP].vt.ch_left_x =(((buff[6] >> 1) | (buff[7] << 7)) & 0x07ff) - VT_CH_VALUE_OFFSET;                       //通道3

            // 控制按钮
            VT_info.vt_ctrl[TEMP].vt.mode_sw = ((buff[7] >> 4) & 0x0003);                //挡位键
            VT_info.vt_ctrl[TEMP].vt.pause = ((buff[7] >> 6) & 0x0001);                  //暂停键
            VT_info.vt_ctrl[TEMP].vt.fn_left = ((buff[7] >> 7) & 0x0001);                //自定义左键
            VT_info.vt_ctrl[TEMP].vt.fn_right = (buff[8] & 0x0001);                      //自定义右键
            VT_info.vt_ctrl[TEMP].vt.wheel =(((buff[8] >> 1) | (buff[9] << 7)) & 0x07ff);// 拨轮
            VT_info.vt_ctrl[TEMP].vt.trigger = ((buff[9] >> 4) & 0x0001);                //扳机键

            // 鼠标解析
            VT_info.vt_ctrl[TEMP].mouse.x = (buff[10] | (buff[11] << 8) & 0xffff); //X轴
            VT_info.vt_ctrl[TEMP].mouse.y = (buff[12] | (buff[13] << 8) & 0xffff); //Y轴
            VT_info.vt_ctrl[TEMP].mouse.z = (buff[14] | (buff[15] << 8) & 0xffff); //Z滚轮
            VT_info.vt_ctrl[TEMP].mouse.left = (buff[16] & 0x0003);                //左键
            VT_info.vt_ctrl[TEMP].mouse.right = (buff[16] >> 2 & 0x0003);          //右键
            VT_info.vt_ctrl[TEMP].mouse.middle = ((buff[16] >> 4) & 0x0003);       //中键

            //  位域的按键值解算,直接memcpy即可,注意小端低字节在前,即lsb在第一位,msb在最后
            *(uint16_t *) &VT_info.vt_ctrl[TEMP].key[KEY_PRESS] = (uint16_t) ((buff[17] | buff[18] <<8) & 0xffff);
            // if (VT_info.vt_ctrl[TEMP].key[KEY_PRESS].ctrl) // ctrl键按下
            //     VT_info.vt_ctrl[TEMP].key[KEY_PRESS_WITH_CTRL] = VT_info.vt_ctrl[TEMP].key[KEY_PRESS];
            // else
            //     memset(&VT_info.vt_ctrl[TEMP].key[KEY_PRESS_WITH_CTRL], 0, sizeof(Key_vt));
            // if (VT_info.vt_ctrl[TEMP].key[KEY_PRESS].shift) // shift键按下
            //     VT_info.vt_ctrl[TEMP].key[KEY_PRESS_WITH_SHIFT] = VT_info.vt_ctrl[TEMP].key[KEY_PRESS];
            // else
            //     memset(&VT_info.vt_ctrl[TEMP].key[KEY_PRESS_WITH_SHIFT], 0, sizeof(Key_vt));

            uint16_t key_now = VT_info.vt_ctrl[TEMP].key[KEY_PRESS].keys,               // 当前按键是否按下
            key_last = VT_info.vt_ctrl[LAST].key[KEY_PRESS].keys;                      // 上一次按键是否按下
            // key_with_ctrl = VT_info.vt_ctrl[TEMP].key[KEY_PRESS_WITH_CTRL].keys,        // 当前ctrl组合键是否按下
            // key_with_shift = VT_info.vt_ctrl[TEMP].key[KEY_PRESS_WITH_SHIFT].keys,      //  当前shift组合键是否按下
            // key_last_with_ctrl = VT_info.vt_ctrl[LAST].key[KEY_PRESS_WITH_CTRL].keys,   // 上一次ctrl组合键是否按下
            // key_last_with_shift = VT_info.vt_ctrl[LAST].key[KEY_PRESS_WITH_SHIFT].keys; // 上一次shift组合键是否按下

            for (uint16_t i = 0, j = 0x1; i < 16; j <<= 1, i++) {
                // if (i == 4 || i == 5) // 4,5位为ctrl和shift,直接跳过
                //     continue;
                // 如果当前按键按下,上一次按键没有按下,且ctrl和shift组合键没有按下,则按键按下计数加1(检测到上升沿)
                //if ((key_now & j) && !(key_last & j) && !(key_with_ctrl & j) && !(key_with_shift & j))
                if ((key_now & j) && !(key_last & j))
                    VT_info.vt_ctrl[TEMP].key_count[KEY_PRESS][i]++;
                // // 当前ctrl组合键按下,上一次ctrl组合键没有按下,则ctrl组合键按下计数加1(检测到上升沿)
                // if ((key_with_ctrl & j) && !(key_last_with_ctrl & j))
                //     VT_info.vt_ctrl[TEMP].key_count[KEY_PRESS_WITH_CTRL][i]++;
                // // 当前shift组合键按下,上一次shift组合键没有按下,则shift组合键按下计数加1(检测到上升沿)
                // if ((key_with_shift & j) && !(key_last_with_shift & j))
                //     VT_info.vt_ctrl[TEMP].key_count[KEY_PRESS_WITH_SHIFT][i]++;
            }

            memcpy(&VT_info.vt_ctrl[LAST], &VT_info.vt_ctrl[TEMP], sizeof(VT_ctrl_t)); // 保存上一次的数据,用于按键持续按下和切换的判断
        }
    }
}

/*裁判系统串口接收回调函数,解析数据 */
static void VTRxCallback()
{
    DaemonReload(VT_daemon);
    VTJudgeReadData(VT_usart_instance->recv_buff);
    //FLAG_CRC++;
}
// 裁判系统丢失回调函数,重新初始化裁判系统串口
static void VTLostCallback(void *arg)
{
    VT_info.vt_ctrl->vt.wheel = 0;
    VT_info.vt_ctrl[TEMP].key->w = 0;
    VT_info.vt_ctrl[TEMP].key->s = 0;
    VT_info.vt_ctrl[TEMP].key->a = 0;
    VT_info.vt_ctrl[TEMP].key->d = 0;
    VT_info.vt_ctrl[TEMP].mouse.left = 0;

    USARTServiceInit(VT_usart_instance);
    LOGWARNING("[rm_vt] lost vt data");
}

/* 裁判系统通信初始化 */
VT_info_t *VTInit(UART_HandleTypeDef *VT_usart_handle)
{
    USART_Init_Config_s conf;
    conf.module_callback = VTRxCallback;
    conf.usart_handle = VT_usart_handle;
    conf.recv_buff_size = VT_RE_RX_BUFFER_SIZE; // mx 255(u8)
    VT_usart_instance = USARTRegister(&conf);

    Daemon_Init_Config_s daemon_conf = {
            .callback = VTLostCallback,
            .owner_id = VT_usart_instance,
            .reload_count = 30, // 0.3s没有收到数据,则认为丢失,重启串口接收
    };
    VT_daemon = DaemonRegister(&daemon_conf);

    return &VT_info;
}