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

// --- 系统辨识配置宏 ---
#define SYS_ID_MODE_ON  0       // 1: 开启辨识, 0: 关闭
#define SYS_ID_AMP      1.5f   // 激励振幅 (单位: rad/s，请根据电机响应调整，不要太大)
#define SYS_ID_MIN_FREQ 1.0f    // 起始频率 1Hz
#define SYS_ID_MAX_FREQ 50.0f   // 终止频率 50Hz
#define SYS_ID_DURATION 30.0f   // 扫频持续时间 30秒

// 系统辨识全局变量
static float sys_id_tick = 0.0f;

/**
 * @brief 生成线性扫频信号 (Chirp Signal)
 * @param dt 控制周期 (秒)
 * @return 当前时刻的激励值
 */
float Gimbal_SysID_Generate_Chirp(float dt) {
    if (sys_id_tick > SYS_ID_DURATION) {
        return 0.0f;
    }

    // 线性频率增长公式: f(t) = f_start + (f_end - f_start) * (t / duration)
    float k = (SYS_ID_MAX_FREQ - SYS_ID_MIN_FREQ) / SYS_ID_DURATION;
    float current_freq = SYS_ID_MIN_FREQ + k * sys_id_tick;

    // 激励信号: A * sin(2 * PI * f(t) * t) -> 注意：变频信号相位是频率的积分
    // Phase = 2 * PI * (f_start * t + 0.5 * k * t^2)
    float phase = 2.0f * PI * (SYS_ID_MIN_FREQ * sys_id_tick + 0.5f * k * sys_id_tick * sys_id_tick);

    float output = SYS_ID_AMP * arm_sin_f32(phase);

    sys_id_tick += dt;
    return output;
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
                            .MaxOut = 20,

                    },
                    .speed_PID = {
                            .Kp = 2.1f,
                            .Ki = 60,
                            .Kd = 0.01f, //0.005f
                            .Improve = PID_Integral_Limit | PID_Derivative_On_Measurement | PID_DerivativeFilter,
                            .IntegralLimit = 5,
                            .MaxOut = 10, //即输出扭矩TMAX，可通过达妙上位机读取或修改
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
                    .outer_loop_type = ANGLE_LOOP,
                    .close_loop_type = ANGLE_LOOP | SPEED_LOOP ,
                    // .outer_loop_type = SPEED_LOOP,
                    // .close_loop_type = SPEED_LOOP,
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
                            .Ki = 30.0f,//可以不给，没有阻尼测试
                            .Kd = 0.001f,//白头0.005,绿头0.001
                            .Improve = PID_Integral_Limit | PID_Derivative_On_Measurement,
                            .IntegralLimit = 5,
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
            .DM_P_MAX = 3.14159f,
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
    if (gimbal_cmd_recv.DMMOTOR_Error_Clear_flag == 1) {
        //DMMotorClearError(yaw_motor);
        DMMotorClearError(pitch_motor);
    }

    // @todo:现在已不再需要电机反馈,实际上可以始终使用IMU的姿态数据来作为云台的反馈,yaw电机的offset只是用来跟随底盘
    // 根据控制模式进行电机反馈切换和过渡,视觉模式在robot_cmd模块就已经设置好,gimbal只看yaw_ref和pitch_ref
    switch (gimbal_cmd_recv.gimbal_mode) {
        {
        case GIMBAL_ZERO_FORCE:
            DMMotorStop(yaw_motor);
            DMMotorStop(pitch_motor);
            yaw_motor->motor_controller.speed_PID.ITerm = 0.0f;
            yaw_motor->motor_controller.speed_PID.Iout = 0.0f;
            pitch_motor->motor_controller.speed_PID.ITerm = 0.0f;
            pitch_motor->motor_controller.speed_PID.Iout = 0.0f;
            break;
            // 使用陀螺仪的反馈,底盘根据yaw电机的offset跟随云台或视觉模式采用
        case GIMBAL_GYRO_MODE: // 后续只保留此模式
            DMMotorEnable(yaw_motor);
            DMMotorEnable(pitch_motor);

            // 速度前馈
            if (gimbal_cmd_recv.auto_aim_mode == 1) {
                //yaw_speed_feedforward = gimbal_cmd_recv.vision_yaw_speed_ff*PI/180.0f;
            }
            else {
                yaw_speed_feedforward = 0;

            }

            DMMotorSetRef(yaw_motor, gimbal_cmd_recv.yaw); // yaw和pitch会在robot_cmd中处理好多圈和单圈
            pitch_get = gimba_IMU_data->Pitch;
            DMMotorSetRef(pitch_motor,gimbal_cmd_recv.pitch);
            float yaw_angle_err = yaw_motor->motor_controller.angle_PID.Err;
            // 积分分离
            if (fabs(yaw_angle_err) > 0.5f) {
                // 如果角度误差大于2度，强制把速度环的积分项清零
                yaw_motor->motor_controller.speed_PID.ITerm = 0.0f;
                yaw_motor->motor_controller.speed_PID.Iout = 0.0f;
            }

            // Pitch 同理
            float pitch_angle_err = pitch_motor->motor_controller.angle_PID.Err;
            if (fabs(pitch_angle_err) > 0.5f) { // Pitch较轻，阈值可以小点
                pitch_motor->motor_controller.speed_PID.ITerm = 0.0f;
                pitch_motor->motor_controller.speed_PID.Iout = 0.0f;
            }

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
            yaw_motor->motor_controller.speed_PID.ITerm = 0.0f;
            yaw_motor->motor_controller.speed_PID.Iout = 0.0f;
            pitch_motor->motor_controller.speed_PID.ITerm = 0.0f;
            pitch_motor->motor_controller.speed_PID.Iout = 0.0f;

            break;
        default:
            break;
        }
    }
    // 根据IMU姿态/pitch电机角度反馈计算出当前配重下的重力矩

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

    // 设置反馈数据,主要是imu和yaw的ecd
    gimbal_feedback_data.gimbal_imu_data = *gimba_IMU_data;
    gimbal_feedback_data.yaw_motor_single_round_angle = yaw_motor->measure.angle_single_round;
    gimbal_feedback_data.yaw_motor_sum_round_angle= yaw_motor->measure.total_angle;
    gimbal_feedback_data.pitch_motor_single_round_angle= pitch_motor->measure.angle_single_round;
    gimbal_feedback_data.pitch_motor_lost_flag = pitch_motor->lost_flag;
    // 推送消息
    PubPushMessage(gimbal_pub, (void *) &gimbal_feedback_data);
}