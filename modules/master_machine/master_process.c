/**
 * @file master_process.c
 * @author neozng
 * @brief  module for recv&send vision data
 * @version beta
 * @date 2022-11-03
 * @todo 澧炲姞瀵逛覆鍙ｈ皟璇曞姪鎵嬪崗璁殑鏀寔,鍖呮嫭vofa鍜宻erial debug
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

//void VisionSetFlag(Enemy_Color_e enemy_color, Work_Mode_e work_mode, Bullet_Speed_e bullet_speed)
//{
//    send_data.enemy_color = enemy_color;
//    send_data.work_mode = work_mode;
//    send_data.bullet_speed = bullet_speed;
//}
void VisionSetFlag(uint8_t enemy_color)
{
    send_data.mode = enemy_color;

}

//void VisionSetAltitude(float yaw, float pitch)
//{
//    send_data.yaw = yaw;
//    send_data.pitch = pitch;
//}
void VisionSetAltitude(float yaw, float pitch,float roll)
{
    send_data.yaw = yaw;
    send_data.pitch = pitch;
    send_data.roll = roll;
}

/**
 * @brief 绂荤嚎鍥炶皟鍑芥暟,灏嗗湪daemon.c涓daemon task璋冪敤
 * @attention 鐢变簬HAL搴撶殑璁捐闂,涓插彛寮€鍚疍MA鎺ユ敹涔嬪悗鍚屾椂鍙戦€佹湁姒傜巼鍑虹幇__HAL_LOCK()瀵艰嚧鐨勬閿?浣垮緱鏃犳硶
 *            杩涘叆鎺ユ敹涓柇.閫氳繃daemon鍒ゆ柇鏁版嵁鏇存柊,閲嶆柊璋冪敤鏈嶅姟鍚姩鍑芥暟浠ヨВ鍐虫闂.
 *
 * @param id vision_usart_instance鐨勫湴鍧€,姝ゅ娌＄敤.
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
 * @brief 鎺ユ敹瑙ｅ寘鍥炶皟鍑芥暟,灏嗗湪bsp_usart.c涓usart rx callback璋冪敤
 * @todo  1.鎻愰珮鍙鎬?灏唃et_protocol_info鐨勭鍥涗釜鍙傛暟澧炲姞涓€涓猣loat绫诲瀷buffer
 *        2.娣诲姞鏍囧織浣嶈В鐮?
 */
static void DecodeVision()
{
    uint16_t flag_register;
    DaemonReload(vision_daemon_instance); // 鍠傜嫍
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

    // 涓簃aster process娉ㄥ唽daemon,鐢ㄤ簬鍒ゆ柇瑙嗚閫氫俊鏄惁绂荤嚎
    Daemon_Init_Config_s daemon_conf = {
        .callback = VisionOfflineCallback, // 绂荤嚎鏃惰皟鐢ㄧ殑鍥炶皟鍑芥暟,浼氶噸鍚覆鍙ｆ帴鏀?
        .owner_id = vision_usart_instance,
        .reload_count = 10,
    };
    vision_daemon_instance = DaemonRegister(&daemon_conf);

    return &recv_data;
}

/**
 * @brief 鍙戦€佸嚱鏁?
 *
 * @param send 寰呭彂閫佹暟鎹?
 *
 */
void VisionSend()
{
    // buff鍜宼xlen蹇呴』涓簊tatic,鎵嶈兘淇濊瘉鍦ㄥ嚱鏁伴€€鍑哄悗涓嶈閲婃斁,浣垮緱DMA姝ｇ‘瀹屾垚鍙戦€?
    // 鏋愭瀯鍚庣殑闄烽槺闇€瑕佺壒鍒敞鎰?
    static uint16_t flag_register;
    static uint8_t send_buff[VISION_SEND_SIZE];
    static uint16_t tx_len;
    // TODO: code to set flag_register
    flag_register = 30 << 8 | 0b00000001;
    // 灏嗘暟鎹浆鍖栦负seasky鍗忚鐨勬暟鎹寘
    get_protocol_send_data(0x02, flag_register, &send_data.yaw, 3, send_buff, &tx_len);
    USARTSend(vision_usart_instance, send_buff, tx_len, USART_TRANSFER_DMA); // 鍜岃瑙夐€氫俊浣跨敤IT,闃叉鍜屾帴鏀朵娇鐢ㄧ殑DMA鍐茬獊
    // 姝ゅ涓篐AL璁捐鐨勭己闄?DMASTOP浼氬仠姝㈠彂閫佸拰鎺ユ敹,瀵艰嚧鍐嶄篃鏃犳硶杩涘叆鎺ユ敹涓柇.
    // 涔熷彲鍦ㄥ彂閫佸畬鎴愪腑鏂腑閲嶆柊鍚姩DMA鎺ユ敹,浣嗚緝涓哄鏉?鍥犳,姝ゅ浣跨敤IT鍙戦€?
    // 鑻ヤ娇鐢ㄤ簡daemon,鍒欎篃鍙互浣跨敤DMA鍙戦€?
}

#endif // VISION_USE_UART

#ifdef VISION_USE_VCP

#include "bsp_usb.h"
static uint8_t *vis_recv_buff; //鎺ユ敹鍒扮殑鍘熷鏁版嵁

static void DecodeVision(uint16_t recv_len)
{
    //uint16_t flag_register;
    //get_protocol_info(vis_recv_buff, &flag_register, (uint8_t *)&recv_data.pitch);
    //    // TODO: code to resolve flag_register;
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
    //    UNUSED(_handle); // 浠呬负浜嗘秷闄よ鍛?
    USB_Init_Config_s conf = {.rx_cbk = DecodeVision};
    vis_recv_buff = USBInit(conf);

    // 涓簃aster process娉ㄥ唽daemon,鐢ㄤ簬鍒ゆ柇瑙嗚閫氫俊鏄惁绂荤嚎
    Daemon_Init_Config_s daemon_conf = {
            .callback = VisionOfflineCallback, // 绂荤嚎鏃惰皟鐢ㄧ殑鍥炶皟鍑芥暟,浼氶噸鍚覆鍙ｆ帴鏀?
            .owner_id = NULL,
            .reload_count = 5, // 50ms
    };
    vision_daemon_instance = DaemonRegister(&daemon_conf);

    return &recv_data;
}

void VisionSend()
{
    //    static uint16_t flag_register;
    //    static uint8_t send_buff[VISION_SEND_SIZE];
    //    static uint16_t tx_len;
    //    // TODO: code to set flag_register
    //    flag_register = 30 << 8 | 0b00000001;
    //    // 灏嗘暟鎹浆鍖栦负seasky鍗忚鐨勬暟鎹寘
    //    get_protocol_send_data(0x02, flag_register, &send_data.yaw, 3, send_buff, &tx_len);
    //    USBTransmit(send_buff, tx_len);
    static uint8_t send_buffer[24]={0};

    send_data.header = 0xff;
    send_data.tail = 0x0d;
    // send_data.mode = 3;
    //    VisionSetFlag(data->detect_color);
    //    VisionSetAim(data->aim_x,data->aim_y,data->aim_z);
    send_data.empty1 = 0;
    send_data.empty2 = 0;
    send_data.checksum = crc_8(&send_data.header,sizeof(send_buffer)-2);
    // send_buffer[0] = send_data.header;
    //send_buffer[15] = send_data.tail;
    //send_buffer[14] = send_data.checksum;
    //send_buffer[1] = send_data.mode;
    //send_buffer[2] = send_data.roll;
    //send_buffer[6] = send_data.pitch;
    //send_buffer[10] = send_data.yaw;
    memcpy(send_buffer,&send_data,24/*sizeof(send_data)*/);

    USBTransmit(send_buffer, sizeof(send_buffer));
}
//static uint8_t *vis_recv_buff; //鎺ユ敹鍒扮殑鍘熷鏁版嵁

/*static void DecodeVision(uint16_t recv_len)
{
//    uint16_t flag_register;
//    get_protocol_info(vis_recv_buff, &flag_register, (uint8_t *)&recv_data.pitch);
//    // TODO: code to resolve flag_register;
    if(vis_recv_buff[0]==0xA5)
    {
        if(VerifyCRC16CheckSum(vis_recv_buff,sizeof(recv_data)))
        {
            memcpy(&recv_data,vis_recv_buff,sizeof(recv_data));
        }
    }
}

/* 瑙嗚閫氫俊鍒濆鍖?*/
/*RecievePacket_t *VisionInit(void)
{
//    UNUSED(_handle); // 浠呬负浜嗘秷闄よ鍛?
    USB_Init_Config_s conf = {.rx_cbk = DecodeVision};
    vis_recv_buff = USBInit(conf);

    // 涓簃aster process娉ㄥ唽daemon,鐢ㄤ簬鍒ゆ柇瑙嗚閫氫俊鏄惁绂荤嚎
    Daemon_Init_Config_s daemon_conf = {
            .callback = VisionOfflineCallback, // 绂荤嚎鏃惰皟鐢ㄧ殑鍥炶皟鍑芥暟,浼氶噸鍚覆鍙ｆ帴鏀?
            .owner_id = NULL,
            .reload_count = 5, // 50ms
    };
    vision_daemon_instance = DaemonRegister(&daemon_conf);

    return &recv_data;
}

void VisionSend()
{
//    static uint16_t flag_register;
//    static uint8_t send_buff[VISION_SEND_SIZE];
//    static uint16_t tx_len;
//    // TODO: code to set flag_register
//    flag_register = 30 << 8 | 0b00000001;
//    // 灏嗘暟鎹浆鍖栦负seasky鍗忚鐨勬暟鎹寘
//    get_protocol_send_data(0x02, flag_register, &send_data.yaw, 3, send_buff, &tx_len);
//    USBTransmit(send_buff, tx_len);
    static uint8_t send_buffer[24]={0};

    send_data.header = 0x5A;
//    VisionSetFlag(data->detect_color);
//    VisionSetAim(data->aim_x,data->aim_y,data->aim_z);
    send_data.checksum = crc_16(&send_data.header,sizeof(send_data)-2);

    memcpy(send_buffer,&send_data,sizeof(send_data));
    USBTransmit(send_buffer, sizeof(send_data));
}*/
#endif // VISION_USE_VCP
