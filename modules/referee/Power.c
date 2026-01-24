// #include "vision_transfer.h"
// #include "string.h"
// #include "crc_ref.h"
// #include "bsp_usart.h"
// #include "task.h"
// #include "daemon.h"
// #include "bsp_log.h"
// #include "cmsis_os.h"
// #include "remote_control.h"
// #include "Power.h"
//
// #define Pr_RX_BUFFER_SIZE 128u // 裁判系统接收缓冲区大小
//
// static USARTInstance *power_usart_instance; // 裁判系统串口实例
// static DaemonInstance *power_daemon;		  // 裁判系统守护进程
// static Power_data power_info;			  // 裁判系统数据
//
// /**
//  * @brief  读取裁判数据,中断中读取保证速度
//  * @param  buff: 读取到的裁判系统原始数据
//  * @retval 是否对正误判断做处理
//  * @attention  在此判断帧头和CRC校验,无误再写入数据，不重复判断帧头
//  */
// static void JudgeReadData(uint8_t *buff)
// {
//     static char packet[Pr_RX_BUFFER_SIZE];
//     static int index = 0;
//
//     for(int i=0; i<power_usart_instance->recv_buff_size; i++){
//         uint8_t ch = buff[i];
//
//         // 检测帧尾（换行符）
//         if(ch == '\n') {
//             packet[index] = '\0';
//
//             // 匹配电功率计协议格式
//             if(sscanf(packet, "Voltage:%f,Current:%f,Power:%f,Energy:%f",
//                       &power_info.voltage, &power_info.current,
//                       &power_info.power, &power_info.energy) == 4)
//
//             index = 0;
//         }
//             // 缓冲区保护
//         else if(index < sizeof(packet)-1) {
//             packet[index++] = ch;
//         }
//     }
// }
// static void PowerRxCallback()
// {
//     DaemonReload(power_daemon);
//     JudgeReadData(power_usart_instance->recv_buff);
// }
// // 裁判系统丢失回调函数,重新初始化裁判系统串口
// static void PowerLostCallback(void *arg)
// {
//     USARTServiceInit(power_usart_instance);
//     LOGWARNING("[rm_ref] lost power data");
// }
//
// /* 裁判系统通信初始化 */
// Power_data *PowerInit(UART_HandleTypeDef *power_usart_handle)
// {
//     USART_Init_Config_s conf;
//     conf.module_callback = PowerRxCallback;
//     conf.usart_handle = power_usart_handle;
//     conf.recv_buff_size = Pr_RX_BUFFER_SIZE; // mx 255(u8)
//     power_usart_instance = USARTRegister(&conf);
//
//     Daemon_Init_Config_s daemon_conf = {
//             .callback = PowerLostCallback,
//             .owner_id = power_usart_instance,
//             .reload_count = 30, // 0.3s没有收到数据,则认为丢失,重启串口接收
//     };
//     power_daemon = DaemonRegister(&daemon_conf);
//
//     return &power_info;
// }
// /**
//  * @brief 裁判系统数据发送函数
//  * @param
//  */
// void PowerSend(uint8_t *send, uint16_t tx_len)
// {
//     USARTSend(power_usart_instance, send, tx_len, USART_TRANSFER_DMA);
//     osDelay(115);
// }
