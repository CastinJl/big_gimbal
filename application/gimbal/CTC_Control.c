#include "stm32f4xx.h"
#include "arm_math.h"
#include "CTC_Control.h"
#include "robot_def.h"
#include "motor_def.h"


static CTC_Gimbal_Feedback_s ctc_feedback;
CTC_Controller yaw_cmd;
CTC_Controller pitch_cmd;

CTC_Controller* CTC_Get_Yaw_Cmd(void) {
    return &yaw_cmd;
}

CTC_Controller* CTC_Get_Pitch_Cmd(void) {
    return &pitch_cmd;
}

void CTC_Set_Gimbal_Feedback(const CTC_Gimbal_Feedback_s *feedback) {
    if (feedback != NULL) {
        ctc_feedback = *feedback;
    }
}

static void ESO_pitch(CTC_Controller *control) {
    const float dt = 0.001f;
    const float theta = ctc_feedback.Pitch;
    const float omega = ctc_feedback.pitch_gyro;
    const float tau_input = ctc_feedback.pitch_motor_control_out;
    const float w0 = control->eso_bandwidth;

    if (control->eso_enabled == 0U || control == NULL || w0 <= 0.0f) {
        control->eso_initialized = 0U;
        control->eso_z1 = 0.0f;
        control->eso_z2 = 0.0f;
        control->eso_z3 = 0.0f;
        return;
    }

    if (control->eso_initialized == 0U) {
        control->eso_z1 = theta;
        control->eso_z2 = omega;
        control->eso_z3 = 0.0f;
        control->eso_initialized = 1U;
        return;
    }

    float e = theta - control->eso_z1;
    float z1_dot = control->eso_z2 + 3.0f * w0 * e;
    float z2_dot = (tau_input - PITCH_B * control->eso_z2) / PITCH_J +
                   control->eso_z3 + 3.0f * w0 * w0 * e;
    float z3_dot = w0 * w0 * w0 * e;

    control->eso_z1 += dt * z1_dot;
    control->eso_z2 += dt * z2_dot;
    control->eso_z3 += dt * z3_dot;

    float tau_eso = -control->eso_gain * PITCH_J * control->eso_z3;
    if (control->eso_limit > 0.0f) {
        tau_eso = fminf(fmaxf(tau_eso, -control->eso_limit), control->eso_limit);
    }

    control->torque_ff += tau_eso;
}

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
    ff->torque_ff = PITCH_J * ff->ref_acc + PITCH_B * ff->ref_vel + PITCH_G * arm_cos_f32(theta+1.0106) + PITCH_TC * tanhf(Friction_Smooth * ff->ref_vel);

   ESO_pitch(ff);

    ff->out_current = ff->torque_ff/PITCH_KU;
    return ff->out_current;
}

uint8_t CTC_Pitch_Limit(float speed_cmd, float *safe_angle) {
    float encoder = ctc_feedback.pitch_motor_angle;
    float imu = ctc_feedback.Pitch;

    if (ctc_feedback.pitch_motor_lost_flag) return 0;

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


uint8_t CTC_Pitch_Control(uint8_t is_speed_mode, float speed_cmd, uint8_t need_update) {
    float safe_angle;
    uint8_t limit_hit = CTC_Pitch_Limit(speed_cmd, &safe_angle);

    // 触发限位
    if (limit_hit) {
        pitch_cmd.ref_pos = safe_angle;
        pitch_cmd.ref_vel = 0.0f;
        pitch_cmd.ref_acc = 0.0f;
        return ANGLE_SPEED_CASCADE;
    }
    if (is_speed_mode) {
        pitch_cmd.ref_vel = speed_cmd;
        pitch_cmd.ref_acc = 0.0f;
        return SPEED_LOOP;
    }

    if (need_update) pitch_cmd.ref_pos = ctc_feedback.Pitch;
    pitch_cmd.ref_vel = 0.0f;
    pitch_cmd.ref_acc = 0.0f;
    return ANGLE_SPEED_CASCADE;
}


uint8_t CTC_Yaw_Control(uint8_t is_speed_mode, float speed_cmd, uint8_t need_update) {
    if (is_speed_mode) {
        yaw_cmd.ref_vel = speed_cmd;
        yaw_cmd.ref_acc = 0.0f;
        return SPEED_LOOP;
    }
    if (need_update) yaw_cmd.ref_pos = ctc_feedback.YawTotalAngle;
    yaw_cmd.ref_vel = 0.0f;
    yaw_cmd.ref_acc = 0.0f;
    return ANGLE_SPEED_CASCADE;
}
