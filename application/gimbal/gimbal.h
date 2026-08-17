#ifndef GIMBAL_H
#define GIMBAL_H

#define YAW_PROFILE_DEFAULT_DT            0.001f
#define YAW_PROFILE_MIN_DT                0.0001f
#define YAW_PROFILE_MAX_DT                0.0100f

#define YAW_VEL_MAX 30.0f
#define YAW_ACC_MAX 150.0f   //最大扭矩/J   DM4310额定扭矩3N*m
#define PITCH_VEL_MAX 30.0f
#define PITCH_ACC_MAX 150.0f

typedef struct {
    // --- 轨迹规划状态（平滑后的参考值） ---
    float ref_pos;         // 期望位置 (rad)
    float ref_vel;         // 期望速度 (rad/s)
    float ref_acc;         // 期望加速度 (rad/s²)

    // --- 中间计算变量（便于调试打印） ---
    float torque_ff;       // 前馈力矩 (N*m)

    // --- 最终输出 ---
    float out_current;     // 发送给电调的电流指令
}Gimbal_GrayFF;


typedef struct {
    float pos_f;   // 平滑后目标位置 (rad)
    float vel_ref; // 期望角速度 (rad/s)
    float acc_ref; // 期望角加速度 (rad/s²)
    float test_current;
} RefTraj_t;
/**
 * @brief 初始化云台,会被RobotInit()调用
 * 
 */
void GimbalInit();

/**
 * @brief 云台任务
 * 
 */
void GimbalTask();

#endif // GIMBAL_H
