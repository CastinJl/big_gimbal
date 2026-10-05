//
// Created by sun30 on 2026/8/22.
//
#include "stm32f4xx.h"
#include "arm_math.h"
#include "CTC_Control.h"
#include <math.h>
#include <string.h>
#include "gimbal.h"
#include "robot_def.h"

float Torque_ff_Yaw(CTC_Controller *ff)
{
    // ref_pos/ref_vel/ref_acc 由GimbalTask通过本地轨迹生成 + 视觉覆盖填好
    ff->torque_ff = YAW_J * ff->ref_acc + YAW_B * ff->ref_vel + YAW_TC * tanhf(Friction_Smooth * ff->ref_vel);

    ff->out_current = ff->torque_ff/YAW_KU;
    return ff->out_current;
}


float Torque_ff_Pitch(CTC_Controller *ff,float theta)
{
    // ref_pos/ref_vel/ref_acc 由GimbalTask通过本地轨迹生成 + 视觉覆盖填好
    ff->torque_ff = PITCH_J * ff->ref_acc + PITCH_B * ff->ref_vel + PITCH_G * arm_cos_f32(theta+PITCH_PHI) + PITCH_TC * tanhf(Friction_Smooth * ff->ref_vel);

    ff->out_current = ff->torque_ff/PITCH_KU;
    return ff->out_current;
}

uint8_t CTC_Pitch_Limit(float speed_cmd, float *safe_angle) {
    float encoder = gimbal_feedback_data.pitch_motor_single_round_angle;
    float imu = gimbal_feedback_data.gimbal_imu_data.Pitch;

    if (gimbal_feedback_data.pitch_motor_lost_flag) return 0;

    if (encoder >= PITCH_MAX_ECD && speed_cmd > 0.01f) {
        float max_allow_delta = PITCH_MAX_ECD - encoder;
        *safe_angle = imu + max_allow_delta;
        return 1;
    }
    if (encoder < PITCH_MIN_ECD && speed_cmd < -0.01f) {
        float min_allow_delta = PITCH_MIN_ECD - encoder;
        *safe_angle = imu + min_allow_delta;
        return 1;
    }
    return 0;
}


void CTC_Pitch_Control(uint8_t is_speed_mode, float speed_cmd, uint8_t need_update) {
    float safe_angle;
    uint8_t limit_hit = CTC_Pitch_Limit(speed_cmd, &safe_angle);

    // 触发限位
    if (limit_hit) {
        DMMotorLoopType(pitch_motor, ANGLE_SPEED_CASCADE);
        pitch_cmd.ref_pos = safe_angle;
        pitch_cmd.ref_vel = 0.0f;
        pitch_cmd.ref_acc = 0.0f;
    }else if (is_speed_mode) {
        DMMotorLoopType(pitch_motor, SPEED_LOOP);
        pitch_cmd.ref_vel = speed_cmd;
        pitch_cmd.ref_acc = 0.0f;
    }else {
        DMMotorLoopType(pitch_motor, ANGLE_SPEED_CASCADE);
        if (need_update) pitch_cmd.ref_pos = gimbal_IMU_data->Pitch;
        pitch_cmd.ref_vel = 0.0f;
        pitch_cmd.ref_acc = 0.0f;
    }
}


void CTC_Yaw_Control(uint8_t is_speed_mode, float speed_cmd, uint8_t need_update) {
    if (is_speed_mode) {
        DMMotorLoopType(yaw_motor, SPEED_LOOP);
        yaw_cmd.ref_vel = speed_cmd;
        yaw_cmd.ref_acc = 0.0f;
    }else {
        DMMotorLoopType(yaw_motor, ANGLE_SPEED_CASCADE);
        if (need_update) yaw_cmd.ref_pos = gimbal_IMU_data->YawTotalAngle;
        yaw_cmd.ref_vel = 0.0f;
        yaw_cmd.ref_acc = 0.0f;
    }
}


static void ESO(CTC_Controller *control) {

}

