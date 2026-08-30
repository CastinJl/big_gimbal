#ifndef GIMBAL_H
#define GIMBAL_H

#include "CTC_Control.h"
#include "dmmotor.h"
#include "ins_task.h"
#include "robot_def.h"

#define YAW_VEL_MAX 5.0f
#define YAW_ACC_MAX 50.0f   //最大扭矩/J   DM4310额定扭矩3N*m
#define PITCH_VEL_MAX 5.0f
#define PITCH_ACC_MAX 50.0f

extern attitude_t *gimbal_IMU_data; // 云台IMU数据
extern DMMotorInstance *yaw_motor;
extern DMMotorInstance  *pitch_motor;
extern Gimbal_Upload_Data_s gimbal_feedback_data;
extern CTC_Controller yaw_cmd;
extern CTC_Controller pitch_cmd;
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
