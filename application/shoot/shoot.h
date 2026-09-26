#ifndef SHOOT_H
#define SHOOT_H
#include "robot_def.h"
typedef struct {
    float left_current_rise_delta;    // 左摩擦轮绝对电流相对基线的上升量
    float right_current_rise_delta;   // 右摩擦轮绝对电流相对基线的上升量
    float left_current_recover_delta; // 左轮低于“基线 + 此值”视为恢复
    float right_current_recover_delta;// 右轮低于“基线 + 此值”视为恢复
    float rise_confirm_ms;      // 电流持续上升多久才接受为弹丸脉冲
    float recover_confirm_ms;   // 电流持续恢复多久才确认本次发弹有效
    float timeout_ms;           // 从视觉发弹指令开始等待弹丸的最长时间
} Shoot_Bullet_Detect_Config_s;
typedef enum {
    BULLET_DETECT_IDLE = 0, //空闲状态
    BULLET_DETECT_WAIT_RISE,//等待电流上升
    BULLET_DETECT_WAIT_RECOVER,//等待电流回落
} Bullet_Detect_State_e;

typedef struct {
    Bullet_Detect_State_e state;
    loader_mode_e loader_mode;
    float trigger_time_ms;//起始时间
    float rise_candidate_time_ms;
    float recover_candidate_time_ms;
    float baseline_speed;
} Bullet_Detect_State_s;
/**
 * @brief 发射初始化,会被RobotInit()调用
 * 
 */
void ShootInit();

/**
 * @brief 发射任务
 * 
 */
void ShootTask();


#endif // SHOOT_H
