#ifndef MASTER_PROCESS_H
#define MASTER_PROCESS_H

#include "bsp_usart.h"
#include "seasky_protocol.h"

#define VISION_RECV_SIZE 18u // 褰撳墠涓哄浐瀹氬€?36瀛楄妭
#define VISION_SEND_SIZE 36u

#pragma pack(1)
typedef enum {
    NO_FIRE = 0,
    AUTO_FIRE = 1,
    AUTO_AIM = 2
} Fire_Mode_e;

typedef enum {
    NO_TARGET = 0,
    TARGET_CONVERGING = 1,
    READY_TO_FIRE = 2
} Target_State_e;

typedef enum {
    NO_TARGET_NUM = 0,
    HERO1 = 1,
    ENGINEER2 = 2,
    INFANTRY3 = 3,
    INFANTRY4 = 4,
    INFANTRY5 = 5,
    OUTPOST = 6,
    SENTRY = 7,
    BASE = 8
} Target_Type_e;

typedef struct {
    Fire_Mode_e fire_mode;
    Target_State_e target_state;
    Target_Type_e target_type;

    float pitch;
    float yaw;
} Vision_Recv_s;

typedef enum {
    //	COLOR_NONE = 0,
    //	COLOR_BLUE = 1,
    //	COLOR_RED = 2,
    ENEMY_COLOR_RED = 0,
    ENEMY_COLOR_BLUE = 1,
} Enemy_Color_e;

typedef enum {
    VISION_MODE_AIM = 0,
    VISION_MODE_SMALL_BUFF = 1,
    VISION_MODE_BIG_BUFF = 2
} Work_Mode_e;

typedef enum {
    BULLET_SPEED_NONE = 0,
    BIG_AMU_10 = 10,
    SMALL_AMU_15 = 15,
    BIG_AMU_16 = 16,
    SMALL_AMU_18 = 18,
    SMALL_AMU_30 = 30,
} Bullet_Speed_e;

typedef struct {
    Enemy_Color_e enemy_color;
    Work_Mode_e work_mode;
    Bullet_Speed_e bullet_speed;

    float yaw;
    float pitch;
    float roll;
} Vision_Send_s;

typedef __packed struct {
    uint8_t header;//0xff
    uint8_t mode;
    float roll;
    float pitch;
    float yaw;
    uint8_t checksum;
    uint8_t tail;//0x0d
    float aim_x;
    float aim_y;
    float aim_z;
    uint8_t detect_color: 1;
    uint8_t reserved: 7;
} SendPacket_t;

typedef __packed struct {
    uint8_t header; //= 0xff;
    uint8_t mode;
    float pitch;
    float yaw;
    float distance;
    uint8_t checksum;
    uint8_t tail; //=0x0d;
    float x;
    float y;
    float z;
    float vx;
    float vy;
    float vz;
    float v_yaw;
    float r1;
    float r2;
    float dz;
    uint8_t tracking: 1;
    uint8_t id: 3;          // 0-outpost 6-guard 7-base
    uint8_t armors_num: 3;  // 2-balance 3-outpost 4-normal
    uint8_t reserved: 1;
} RecievePacket_t;
#pragma pack()

/**
 * @brief 璋冪敤姝ゅ嚱鏁板垵濮嬪寲鍜岃瑙夌殑涓插彛閫氫俊
 *
 * @param handle 鐢ㄤ簬鍜岃瑙夐€氫俊鐨勪覆鍙andle(C鏉夸笂涓€鑸负USART1,涓濆嵃涓篣SART2,4pin)
 */
//Vision_Recv_s *VisionInit(UART_HandleTypeDef *_handle);
RecievePacket_t *VisionInit(void);

/**
 * @brief 鍙戦€佽瑙夋暟鎹?
 *
 */
void VisionSend();

/**
 * @brief 璁剧疆瑙嗚鍙戦€佹爣蹇椾綅
 *
 * @param enemy_color
 * @param work_mode
 * @param bullet_speed
 */
//void VisionSetFlag(Enemy_Color_e enemy_color, Work_Mode_e work_mode, Bullet_Speed_e bullet_speed);
void VisionSetFlag(Enemy_Color_e enemy_color);

/**
 * @brief 璁剧疆鍙戦€佹暟鎹殑濮挎€侀儴鍒?
 *
 * @param yaw
 * @param pitch
 */
//void VisionSetAltitude(float yaw, float pitch, float roll);
void VisionSetAltitude(float yaw, float pitch,float roll);

void VisionSetAim(float aim_x, float aim_y,float aim_z);
#endif // !MASTER_PROCESS_H