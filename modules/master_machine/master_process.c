/**
 * @file master_process.c
 * @author neozng
 * @brief  module for recv&send vision data
 * @version beta
 * @date 2022-11-03
 * @todo 增加串口调试助手协议支持，包括 vofa 和 serial debug
 * @copyright Copyright (c) 2022
 *
 */
#include "master_process.h"
#include "seasky_protocol.h"
#include "daemon.h"
#include "bsp_log.h"
#include "robot_def.h"
#include "crc16.h"
#include "crc8.h"
static RecievePacket_t recv_data;
static SendPacket_t send_data;
static DaemonInstance *vision_daemon_instance;


void VisionSetFlag(uint8_t enemy_color)
{
    send_data.mode = enemy_color;

}


void VisionSetAltitude(float yaw, float pitch,float roll)
{
    send_data.yaw = yaw;
    send_data.pitch = roll;
    send_data.roll = pitch;
}

void VisionSetFireDelay(float fire_delay_ms)
{
    send_data.fire_delay_ms = fire_delay_ms;

}
void VisionSetBulletSpeed(float bullet_speed)
{
    send_data.bullet_speed = bullet_speed;

}
/**
 * @brief 离线回调函数，由 daemon task 在 daemon.c 中调用
 * @attention 串口开启 DMA 接收后同时发送，可能因 __HAL_LOCK() 导致死锁，无法再次
 *            进入接收中断。通过 daemon 判断数据是否更新，并重新初始化服务恢复通信。
 *
 * @param id vision_usart_instance 的地址，此处未使用。
 */
static void VisionOfflineCallback(void *id)
{
#ifdef VISION_USE_UART
    USARTServiceInit(vision_usart_instance);
#endif // !VISION_USE_UART

    memset(&recv_data, 0, sizeof(recv_data));
    recv_data.distance = -1.0f;

    LOGWARNING("[vision] vision offline, restart communication.");
}

#ifdef VISION_USE_UART

#include "bsp_usart.h"

static USARTInstance *vision_usart_instance;

/**
 * @brief 接收解包回调函数，由 bsp_usart.c 中的串口接收回调调用
 * @todo 1. 提高可读性，为 get_protocol_info 的第四个参数增加 float 类型的缓冲区
 *       2. 添加标志位解码
 */
static void DecodeVision()
{
    uint16_t flag_register;
    DaemonReload(vision_daemon_instance); // 喂狗
    get_protocol_info(vision_usart_instance->recv_buff, &flag_register, (uint8_t *)&recv_data.pitch);
    // TODO: code to resolve flag_register;
}

Vision_Recv_s *VisionInit(UART_HandleTypeDef *_handle)
{
    USART_Init_Config_s conf;
    conf.module_callback = DecodeVision;
    conf.recv_buff_size = VISION_RECV_SIZE;
    conf.usart_handle = _handle;
    vision_usart_instance = USARTRegister(&conf);

    // 为 master process 注册 daemon，用于判断视觉通信是否离线
    Daemon_Init_Config_s daemon_conf = {
        .callback = VisionOfflineCallback, // 离线时调用，重新启动串口接收
        .owner_id = vision_usart_instance,
        .reload_count = 10,
    };
    vision_daemon_instance = DaemonRegister(&daemon_conf);

    return &recv_data;
}

/**
 * @brief 发送函数
 *
 *
 */
void VisionSend()
{
    // 缓冲区和长度变量使用 static，避免函数退出后失效，保证 DMA 发送完成
    // 使用结构体组织数据时，需要特别注意内存布局
    static uint16_t flag_register;
    static uint8_t send_buff[VISION_SEND_SIZE];
    static uint16_t tx_len;
    // TODO: code to set flag_register
    flag_register = 30 << 8 | 0b00000001;
    // 将数据转换为 seasky 协议数据包
    get_protocol_send_data(0x02, flag_register, &send_data.yaw, 3, send_buff, &tx_len);
    USARTSend(vision_usart_instance, send_buff, tx_len, USART_TRANSFER_DMA); // 向视觉端发送串口数据
    // DMASTOP 会同时停止发送和接收，可能导致无法再次进入接收中断
    // 可在发送完成中断中重新启动 DMA 接收，或使用中断发送避免干扰 DMA 接收
    // 也可使用 daemon 在通信离线后恢复 DMA 接收
}

#endif // VISION_USE_UART

#ifdef VISION_USE_VCP

#include "bsp_usb.h"
static uint8_t *vis_recv_buff; // 接收到的原始数据

static void DecodeVision(uint16_t recv_len)
{
    if (recv_len >= 32 && vis_recv_buff[0] == 0xff) {
        if (vis_recv_buff[31] == 0x0d) {
            if (vis_recv_buff[30] == crc_8(vis_recv_buff, 30)) {
                memcpy(&recv_data, vis_recv_buff, 32);
                DaemonReload(vision_daemon_instance);
            }
        }
    }
}


RecievePacket_t *VisionInit(void)
{
    //    UNUSED(_handle);
    USB_Init_Config_s conf = {.rx_cbk = DecodeVision};
    vis_recv_buff = USBInit(conf);

    Daemon_Init_Config_s daemon_conf = {
            .callback = VisionOfflineCallback,
            .owner_id = NULL,
            .reload_count = 5, // 50ms
    };
    vision_daemon_instance = DaemonRegister(&daemon_conf);

    return &recv_data;
}

void VisionSend()
{
    static uint8_t send_buffer[24]={0};

    send_data.header = 0xff;
    send_data.tail = 0x0d;
    // send_data.mode = 3;
    //    VisionSetFlag(data->detect_color);
    //    VisionSetAim(data->aim_x,data->aim_y,data->aim_z);
  //  send_data.bullet_speed = ;
    send_data.checksum = crc_8(&send_data.header,sizeof(send_buffer)-2);

    memcpy(send_buffer,&send_data,24/*sizeof(send_data)*/);

    USBTransmit(send_buffer, sizeof(send_buffer));
}

#endif // VISION_USE_VCP
