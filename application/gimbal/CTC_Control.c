//
// Created by sun30 on 2026/8/22.
//
#include "stm32f4xx.h"
#include "arm_math.h"
#include "CTC_Control.h"
#include <math.h>
#include <string.h>


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
    ff->torque_ff = PITCH_J * ff->ref_acc + PITCH_B * ff->ref_vel + PITCH_G * arm_cos_f32(theta) + PITCH_TC * tanhf(Friction_Smooth * ff->ref_vel);

    ff->out_current = ff->torque_ff/PITCH_KU;
    return ff->out_current;
}


static void ESO(CTC_Controller *control) {

}

