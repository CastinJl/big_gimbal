#ifndef GIMBAL_H
#define GIMBAL_H

#define YAW_PROFILE_DEFAULT_DT            0.001f
#define YAW_PROFILE_MIN_DT                0.0001f
#define YAW_PROFILE_MAX_DT                0.0100f

#define YAW_VEL_MAX 5.0f
#define YAW_ACC_MAX 50.0f   //最大扭矩/J   DM4310额定扭矩3N*m
#define PITCH_VEL_MAX 5.0f
#define PITCH_ACC_MAX 50.0f


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
