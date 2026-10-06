//
// Created by sun30 on 2026/8/22.
//

#ifndef STEER_GIMBAL_CTC_CONTROL_H
#define STEER_GIMBAL_CTC_CONTROL_H
#include <stdint.h>

#define Friction_Smooth 100.0f
#define YAW_J  0.02552f    // kg*m^2
#define YAW_B  0.044349312f  // N*m*s/rad
#define YAW_TC 0.14761474f  // N*m
#define YAW_KU 0.78064207f   // N*m/unit
#define PITCH_J 0.003f
#define PITCH_B 0.019479f
#define PITCH_G 0.08003f
#define PITCH_KU 0.21299f
#define PITCH_TC 0.0876f
#define FF_GAIN 1.0f  // 前馈增益，建议 0.85~0.95

typedef struct {
    float ref_pos;         // 期望位置 (rad)
    float ref_vel;         // 期望速度 (rad/s)
    float ref_acc;         // 期望加速度 (rad/s²)
    float torque_ff;       // 前馈力矩 (N*m)
    float out_current;     // 发送给电调的电流指令

    float eso_bandwidth;            //带宽
    float eso_gain;                 //eso增益
    float eso_limit;                //eso输出限幅

    float eso_z1;
    float eso_z2;
    float eso_z3;
    uint8_t eso_initialized;

}CTC_Controller;

float Torque_ff_Yaw(CTC_Controller *ff);
float Torque_ff_Pitch(CTC_Controller *ff,float theta);

void CTC_Pitch_Control(uint8_t is_speed_mode, float speed_cmd, uint8_t need_update);
void CTC_Yaw_Control(uint8_t is_speed_mode, float speed_cmd, uint8_t need_update);

#endif //STEER_GIMBAL_CTC_CONTROL_H
