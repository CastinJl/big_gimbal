#ifndef GIMBAL_H
#define GIMBAL_H


typedef struct {
    // --- 轨迹规划状态（平滑后的参考值） ---
    float ref_pos;         // 期望位置 (rad)
    float ref_vel;         // 期望速度 (rad/s)
    float ref_acc;         // 期望加速度 (rad/s²)

    // --- 反馈值（来自传感器） ---
    float fbk_pos;         // 实际位置 (rad)
    float fbk_vel;         // 实际角速度 (rad/s)

    // --- 中间计算变量（便于调试打印） ---
    float torque_ff;       // 前馈力矩 (N*m)
    float torque_p;        // 位置补偿力矩 (N*m)
    float torque_v;        // 速度补偿力矩 (N*m)

    // --- 最终输出 ---
    float out_current;     // 发送给电调的电流指令
}Gimbal_GrayFF;

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
