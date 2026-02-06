#include "gimbal.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "ins_task.h"
#include "message_center.h"
#include "general_def.h"
#include "bmi088.h"
#include "vofa.h"
#include "dmmotor.h"

#define VISION_VELOCITY_LPF 0.4f
static attitude_t *gimba_IMU_data; // 云台IMU数据
static DMMotorInstance *yaw_motor;
static DMMotorInstance  *pitch_motor;



static Publisher_t *gimbal_pub;                   // 云台应用消息发布者(云台反馈给cmd)
static Subscriber_t *gimbal_sub;                  // cmd控制消息订阅者
static Gimbal_Upload_Data_s gimbal_feedback_data; // 回传给cmd的云台状态信息
static Gimbal_Ctrl_Cmd_s gimbal_cmd_recv;         // 来自cmd的控制信息
static float gyro_yaw,gyro_pitch;
static float yaw_speed_deg;
static float gravity_current = 0;
static float yaw_current = 0;
float yaw_speed_feedforward,yaw_feedforward;
float pitch_set,pitch_get,yaw_set;
static float theta;
static float sys_id_target_speed;
uint32_t motor_model_count;

static float last_yaw_ref = 0;
static float dt = 0.001f; // 任务运行周期1ms，也可用 DWT 获取真实 dt
static float vision_kv = 1.0f;


//#include <math.h>

// --- 系统辨识配置宏 ---
#define SYS_ID_MODE_ON      1
#define SYS_ID_MIN_FREQ     0.5f    // 起始频率
#define SYS_ID_MAX_FREQ     100.0f  // 终止频率 (建议拉高到200Hz)
#define SYS_ID_DURATION     60.0f   // 扫频时间

// --- 幅值配置 (自动渐变) ---
#define SYS_ID_START_AMP    1.0f    // 低频时的幅值 (小一点，安全)
#define SYS_ID_END_AMP      5.0f    // 高频时的幅值 (大一点，抗噪)

static float sys_id_time = 0.0f;

float Gimbal_SysID_Generate_Chirp(float dt) {
    // 1. 超时检测
    if (sys_id_time > SYS_ID_DURATION) {
        return 0.0f; // 扫频结束，输出0
    }

    // --- 计算当前时刻的进度 (0.0 ~ 1.0) ---
    float progress = sys_id_time / SYS_ID_DURATION;

    // --- A. 计算动态幅值 (关键改进) ---
    // 幅值随时间线性增加: Amp(t) = Start + (End - Start) * progress
    float current_amp = SYS_ID_START_AMP + (SYS_ID_END_AMP - SYS_ID_START_AMP) * progress;

    // --- B. 计算当前相位 ---
    // 线性频率增长率 k
    float k = (SYS_ID_MAX_FREQ - SYS_ID_MIN_FREQ) / SYS_ID_DURATION;

    // 累计相位公式: phase = 2*pi * (f_start * t + 0.5 * k * t^2)
    float phase_accum = 2.0f * PI * (SYS_ID_MIN_FREQ * sys_id_time + 0.5f * k * sys_id_time * sys_id_time);

    // 对 2*PI 取模，防止 float 精度溢出 (一定要加这个!)
    float phase_wrapped = fmodf(phase_accum, 2.0f * PI);

    // --- C. 生成正弦波 ---
    float output = current_amp * arm_sin_f32(phase_wrapped);

    // 更新时间
    sys_id_time += dt;
    return output;
}


// // --- 系统辨识配置宏 ---
// #define SYS_ID_MODE_ON  1
// #define SYS_ID_AMP      2.0f    // 尝试加大一点，或者分段测试
// #define SYS_ID_MIN_FREQ 0.5f    // 稍微降低起点，获取更好的直流特性
// #define SYS_ID_MAX_FREQ 200.0f  // 【关键】提升到 200Hz，捕捉高频极点
// #define SYS_ID_DURATION 60.0f   // 拉长时间，保证扫频够慢，减少瞬态影响
//
// static float sys_id_time = 0.0f;
//
// float Gimbal_SysID_Generate_Chirp(float dt) {
//     if (sys_id_time > SYS_ID_DURATION) {
//         return 0.0f;
//     }
//
//     // 1. 计算当前的瞬时频率 (Linear Chirp)
//     // f(t) = f_start + k * t
//     float k = (SYS_ID_MAX_FREQ - SYS_ID_MIN_FREQ) / SYS_ID_DURATION;
//
//     // 2. 计算累计相位 (Phase = Integral of 2*pi*f(t) dt)
//     // 公式: phase = 2*pi * (f_start * t + 0.5 * k * t^2)
//     float phase_accum = 2.0f * PI * (SYS_ID_MIN_FREQ * sys_id_time + 0.5f * k * sys_id_time * sys_id_time);
//
//     // 3. 【关键改进】对 2*PI 取模，防止 float 精度溢出导致波形畸变
//     // 注意：arm_sin_f32 通常接受弧度。如果用的是查表法，确保输入范围正确。
//     float phase_wrapped = fmodf(phase_accum, 2.0f * PI);
//
//     // 4. 生成输出
//     float output = SYS_ID_AMP * arm_sin_f32(phase_wrapped);
//
//     sys_id_time += dt;
//     return output;
// }


float Yaw_FF_Filter(float input) {
    // --- 调参区域 ---
    // 系数 alpha (0~1)：
    // 0.1: 超强滤波 (非常平滑，滞后明显)
    // 0.3: 推荐值 (平衡点)
    // 0.8: 弱滤波 (响应快，可能还有点响)
    const float alpha = 0.6f;
    // ---------------

    // 使用 static 变量，函数退出后数值依然保留，下次进来还能接着用
    // 相当于自带了记忆功能
    static float last_output = 0.0f;
    static uint8_t first_run = 1;

    // 第一次运行时的特殊处理，防止从0开始缓慢爬升
    if (first_run) {
        last_output = input;
        first_run = 0;
        return input;
    }

    // 核心公式: Output = a * Input + (1-a) * Last
    last_output = alpha * input + (1.0f - alpha) * last_output;

    return last_output;
}


void GimbalInit() {
    gimba_IMU_data = INS_Init(); // IMU先初始化,获取姿态数据指针赋给yaw电机的其他数据来源
    // YAW
    Motor_Init_Config_s yaw_config = {
            .can_init_config = {
                    .can_handle = &hcan1,
                    .tx_id = 0x10, //
                    .rx_id = 0x1b
            },
            .controller_param_init_config = {
                    .angle_PID = {
                            .Kp = 30,//50//30
                            .Ki = 0.0f,//0
                            .Kd = 0.0f,//0.0f,//0.005f
                            .DeadBand = 0.0f,
                            .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                            .IntegralLimit = 500,
                            .MaxOut = 20//500//300,

                    },
                    .speed_PID = {
                            .Kp = 2.2f,//10
                            .Ki = 0,
                            .Kd = 0.0f, //0.005f
                            .Improve = PID_Integral_Limit | PID_Derivative_On_Measurement | PID_DerivativeFilter,                            .IntegralLimit = 10000,
                            .MaxOut = 10,
                            .Derivative_LPF_RC = 0.0f
                           // .Output_LPF_RC = 0.003f
                    },
                    .other_angle_feedback_ptr = &gimba_IMU_data->YawTotalAngle,
                    // 还需要增加角速度额外反馈指针,注意方向,ins_task.md中有c板的bodyframe坐标系说明
                    // .other_speed_feedback_ptr = &yaw_speed_deg,
                    .other_speed_feedback_ptr = &gimba_IMU_data->Gyro[2],
                    .current_feedforward_ptr = &yaw_current,
                    .speed_feedforward_ptr = &yaw_speed_feedforward,

            },
            .controller_setting_init_config = {
                    .angle_feedback_source = OTHER_FEED,
                    .speed_feedback_source = OTHER_FEED,
                    // .outer_loop_type = ANGLE_LOOP,
                    // .close_loop_type = ANGLE_LOOP | SPEED_LOOP ,
                    // .outer_loop_type = CURRENT_LOOP,
                    .close_loop_type = CURRENT_LOOP,
                    .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
                    //.feedforward_flag = SPEED_FEEDFORWARD,
            },
            .motor_type = DM4310,
            .DM_P_MAX = 3.14159,
            .DM_V_MAX = 30,
            .DM_T_MAX = 10
    };
    // PITCH
    Motor_Init_Config_s pitch_config = {
            .can_init_config = {
                    .can_handle = &hcan2,
                    .tx_id = 0x09,  //slave id
                    .rx_id = 0x1a,  //MASTER ID
            },
            .controller_param_init_config = {
                    .angle_PID = {
                            .Kp = 50.0f,//两个都可以50
                            .Ki = 0.0f,//1,//0
                            .Kd = 0.01f,//0.0f
                            .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                            .IntegralLimit = 0.3f,
                            .MaxOut = 5,

                    },
                    .speed_PID = {
                            .Kp = 2.0f,//白头2.25，绿头2.0（应该也可以2.25
                            .Ki = 0.0f,//可以不给，没有阻尼测试
                            .Kd = 0.001f,//白头0.005,绿头0.001
                            .Improve = PID_Integral_Limit | PID_Derivative_On_Measurement,
                            .IntegralLimit = 10000,
                            .MaxOut = 10,
                            //.Derivative_LPF_RC = 0.02f

                    },
                    .other_angle_feedback_ptr = &gimba_IMU_data->Pitch,//&gimba_IMU_data->Pitch,
                    // 还需要增加角速度额外反馈指针,注意方向,ins_task.md中有c板的bodyframe坐标系说明
                    .other_speed_feedback_ptr = &gimba_IMU_data->Gyro[0],//(&gimba_IMU_data->Gyro[0]),
                    .current_feedforward_ptr = &gravity_current,
            },
            .controller_setting_init_config = {
                    .angle_feedback_source = OTHER_FEED,
                    .speed_feedback_source = OTHER_FEED,
                    // .outer_loop_type = CURRENT_LOOP,
                    .outer_loop_type = ANGLE_LOOP,
                    .close_loop_type = ANGLE_LOOP | SPEED_LOOP ,
                    .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
                    .feedforward_flag = CURRENT_FEEDFORWARD,
            },
            .motor_type = DM4310,
            .DM_P_MAX = 12.5f,
            .DM_V_MAX = 30,
            .DM_T_MAX = 10
    };
    // 电机对total_angle闭环,上电时为零,会保持静止,收到遥控器数据再动
    yaw_motor = DMMotorInit(&yaw_config);
    pitch_motor = DMMotorInit(&pitch_config);

    gimbal_pub = PubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    gimbal_sub = SubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
}



/* 机器人云台控制核心任务,后续考虑只保留IMU控制,不再需要电机的反馈 */
void GimbalTask() {
    //红点激光
    //HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_SET);
    // 获取云台控制数据
    // 后续增加未收到数据的处理
    SubGetMessage(gimbal_sub, &gimbal_cmd_recv);
    // if(gimba_IMU_data->Pitch > 20.0f)
    // {
    //     pitch_motor->motor_controller.speed_PID.Kp = 1.6f;
    //     // pitch_motor->motor_controller.speed_PID.Kd = 0.005f;
    // }
    // else
    // {
    //     pitch_motor->motor_controller.speed_PID.Kp = 2.0f;
    // }
    // if(gimbal_cmd_recv.auto_aim_mode)
    // {
    //     yaw_motor->motor_controller.speed_PID.Kp = 300.0f; //560.0f
    //     yaw_motor->motor_controller.speed_PID.Kd = 0.00f;
    // }
    // else
    // {
    //     yaw_motor->motor_controller.speed_PID.Kp = 400.00f;
    //
    //     yaw_motor->motor_controller.speed_PID.Kd = 0.15f;
    //     yaw_motor->motor_controller.angle_PID.Kp = 20.0f;
    //     yaw_motor->motor_controller.angle_PID.Ki = 0.0f;
    //     yaw_motor->motor_controller.angle_PID.Kd = 0.0f;
    // }
    // @todo:现在已不再需要电机反馈,实际上可以始终使用IMU的姿态数据来作为云台的反馈,yaw电机的offset只是用来跟随底盘
    // 根据控制模式进行电机反馈切换和过渡,视觉模式在robot_cmd模块就已经设置好,gimbal只看yaw_ref和pitch_ref
    switch (gimbal_cmd_recv.gimbal_mode) {
        {
        case GIMBAL_ZERO_FORCE:
            DMMotorStop(yaw_motor);
            DMMotorStop(pitch_motor);

            break;
            // 使用陀螺仪的反馈,底盘根据yaw电机的offset跟随云台或视觉模式采用
        case GIMBAL_GYRO_MODE: // 后续只保留此模式
            // DMMotorEnable(yaw_motor);
            // DMMotorEnable(pitch_motor);
            // float yaw_target_speed = (gimbal_cmd_recv.yaw - last_yaw_ref) * PI / 180 / dt;
            // last_yaw_ref = gimbal_cmd_recv.yaw;
            // float chassis_comp_speed = -gimbal_cmd_recv.real_wz;
            // //速度前馈限幅
            // if (yaw_target_speed > MAX_FF_SPEED_RAD) {
            //     yaw_target_speed = MAX_FF_SPEED_RAD;
            // } else if (yaw_target_speed < -MAX_FF_SPEED_RAD) {
            //     yaw_target_speed = -MAX_FF_SPEED_RAD;
            // }
            // //yaw_target_speed = Yaw_FF_Filter(yaw_target_speed);   //速度前馈滤波，代价是响应速度
            // // 速度前馈
            // yaw_speed_feedforward = yaw_target_speed * vision_kv;
            // DMMotorSetRef(yaw_motor, gimbal_cmd_recv.yaw); // yaw和pitch会在robot_cmd中处理好多圈和单圈
            // pitch_get = gimba_IMU_data->Pitch;
            // // if (motor_model_count > 50) {
            // //     motor_model_count = 0;
            // //     pitch_set = generate_angle_setpoint();
            // // }
            // // motor_model_count++;
            // DMMotorSetRef(pitch_motor,gimbal_cmd_recv.pitch);
            // //DMMotorSetRef(pitch_motor,pitch_set);
            // // // DMMotorSetRef(pitch_motor,5);
            // // roll_get = gimba_IMU_data->Roll;


            #if SYS_ID_MODE_ON
                        // --- 系统辨识模式 ---
                        DMMotorEnable(yaw_motor);
                        DMMotorEnable(pitch_motor);
                        //生成激励信号
                        sys_id_target_speed = Gimbal_SysID_Generate_Chirp(dt);
                        DMMotorSetRef(yaw_motor, sys_id_target_speed);
                        DMMotorSetRef(pitch_motor, 0); // Pitch 保持 0
            #endif
            break;


            // 云台自由模式,使用编码器反馈,底盘和云台分离,仅云台旋转,一般用于调整云台姿态(英雄吊射等)/能量机关,
        case GIMBAL_FREE_MODE: // 后续删除,或加入云台追地盘的跟随模式(响应速度更快)
            DMMotorStop(yaw_motor);
            DMMotorStop(pitch_motor);

            //DJIMotorSetRef(yaw_motor, gimbal_cmd_recv.yaw); // yaw和pitch会在robot_cmd中处理好多圈和单圈

            //        big_yaw_angle = chassis_cmd_recv.offset_angle;
            //        big_yaw_speed = big_yaw_motor->measure.speed_rads;
            //
            //        ECMotorSetRef(big_yaw_motor,0);
            //            big_yaw_angle = gimba_IMU_data->YawTotalAngle - (yaw_motor->measure.total_angle - 44);
            //            big_yaw_speed = yaw_speed - yaw_motor->measure.speed_aps * DEGREE_2_RAD;

            //DMMotorSetRef(pitch_motor,gimbal_cmd_recv.pitch);

            break;
        default:
            break;
        }
    }
    // 在合适的地方添加pitch重力补偿前馈力矩
    // 根据IMU姿态/pitch电机角度反馈计算出当前配重下的重力矩
    // ...

    //yaw_speed_feedforward  -= gimbal_cmd_recv.real_wz * 180 / PI;
    theta = gimba_IMU_data->Pitch/180*PI;
    if (theta>0.4) {//0.6
        gravity_current = 0.1f;
        //gravity_current = 0;
    }
    else {
        gravity_current = 0.8383*arm_cos_f32(theta+1.0106)-0.2;
        //gravity_current = 0;
    }
    if(gimbal_cmd_recv.real_wz > 0)
    {
        yaw_current = 0;
        //yaw_current = -27.8095*gimbal_cmd_recv.real_wz*gimbal_cmd_recv.real_wz-18.619*gimbal_cmd_recv.real_wz-2154;
    }
    else if(gimbal_cmd_recv.real_wz < 0)
    {
        yaw_current = 0;
        //yaw_current = 27.8095*gimbal_cmd_recv.real_wz*gimbal_cmd_recv.real_wz-18.619*gimbal_cmd_recv.real_wz+2154;
    }
    else
    {
        yaw_current = 0;
    }
    //    float vofa_send_data[4];
    //    vofa_send_data[0] = pitch_motor->motor_controller.speed_PID.Ref;
    //    vofa_send_data[1] = pitch_motor->motor_controller.speed_PID.Measure;
    //    vofa_send_data[2] = pitch_motor->motor_controller.angle_PID.Ref;
    //    vofa_send_data[3] = pitch_motor->motor_controller.angle_PID.Measure;
    //    vofa_justfloat_output(vofa_send_data, 16, &huart1);
    //    float vofa_send_data[4];
    //    vofa_send_data[0] = yaw_motor->motor_controller.speed_PID.Ref;
    //    vofa_send_data[1] = yaw_motor->motor_controller.speed_PID.Measure;
    //    vofa_send_data[2] = yaw_motor->motor_controller.angle_PID.Ref;
    //    vofa_send_data[3] = yaw_motor->motor_controller.angle_PID.Measure;
    //    vofa_justfloat_output(vofa_send_data, 16, &huart1);

    // if(gimba_IMU_data->Roll > 42)
    // {
    //     // DJIMotorStop(yaw_motor);
    //     DMMotorStop(pitch_motor);
    // }

    // 设置反馈数据,主要是imu和yaw的ecd
    gimbal_feedback_data.gimbal_imu_data = *gimba_IMU_data;
    gimbal_feedback_data.yaw_motor_single_round_angle = yaw_motor->measure.angle_single_round;
    gimbal_feedback_data.yaw_motor_sum_round_angle= yaw_motor->measure.total_angle;

    // 推送消息
    PubPushMessage(gimbal_pub, (void *) &gimbal_feedback_data);
}