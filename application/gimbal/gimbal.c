#include "gimbal.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "ins_task.h"
#include "message_center.h"
#include "general_def.h"
#include "bmi088.h"
#include "vofa.h"
#include "dmmotor.h"
#include "math.h"
#include "CTC_Control.h"

attitude_t *gimbal_IMU_data; // 云台IMU数据
static DMMotorInstance *yaw_motor;
static DMMotorInstance  *pitch_motor;

static Publisher_t *gimbal_pub;                   // 云台应用消息发布者(云台反馈给cmd)
static Subscriber_t *gimbal_sub;                  // cmd控制消息订阅者
Gimbal_Upload_Data_s gimbal_feedback_data; // 回传给cmd的云台状态信息
static Gimbal_Ctrl_Cmd_s gimbal_cmd_recv;         // 来自cmd的控制信息
static float pitch_current = 0;
static float yaw_current = 0;

static float last_auto_aim_mode;
static uint8_t last_gimbal_mode;
static float last_pid_auto_aim_mode = -1;//为了防止上电不切换pid
static Motor_Init_Config_s yaw_config;
static Motor_Init_Config_s pitch_config;

// --- 系统辨识配置宏 (Pitch 速度参考 chirp 扫频) ---
#define SYS_ID_MODE_ON  0       // 保持 0: yaw chirp 关闭
#define SYS_ID_AMP      0.20f   // 速度参考峰值(rad/s)，1Hz时角度峰值约11.5°
#define SYS_ID_MIN_FREQ 1.0f    // 起始频率(Hz)
#define SYS_ID_MAX_FREQ 5.0f   // 终止频率(Hz)
#define SYS_ID_DURATION 30.0f   // 扫频时长(s)

// Pitch 速度环系统辨识；打开后仅运行速度环，关闭后恢复正常角度控制
#define PITCH_SYS_ID_MODE_ON   0
#define PITCH_SYS_ID_GRAVITY_FF 1  // 1: 保留重力补偿，0: 纯力矩输入（需机械支撑）
#define PITCH_SYS_ID_LIMIT_RAD   (15.0f * DEGREE_2_RAD)  // 角度保护阈值

#if SYS_ID_MODE_ON && PITCH_SYS_ID_MODE_ON
#error "Yaw and pitch system identification modes cannot| be enabled together."
#endif

// 系统辨识全局变量
static float sys_id_tick = 0.0f;
static  float sys_id_target_speed;

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

    // 激励信号: A * sin(2 * PI * f(t) * t) -> 注意：变频信号相位是频率的积分
    // Phase = 2 * PI * (f_start * t + 0.5 * k * t^2)
    float phase = 2.0f * PI * (SYS_ID_MIN_FREQ * sys_id_tick + 0.5f * k * sys_id_tick * sys_id_tick);

    // 3. 对 2*PI 取模，防止 float 精度溢出
    float phase_wrapped = fmodf(phase, 2.0f * PI);

    float output = SYS_ID_AMP * arm_sin_f32(phase_wrapped);

    sys_id_tick += dt;
    return output;
}


void GimbalInit() {
    gimbal_IMU_data = INS_Init(); // IMU先初始化,获取姿态数据指针赋给yaw电机的其他数据来源
    CTC_Controller *yaw_cmd = CTC_Get_Yaw_Cmd();
    CTC_Controller *pitch_cmd = CTC_Get_Pitch_Cmd();

    // YAW
    yaw_config = (Motor_Init_Config_s){
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id = 0x10, //
            .rx_id = 0x1b
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 30.0f,//并行扭矩量纲起始值(原级联30);实车重标定
                .Ki = 0.0f,//0
                .Kd = 0.0f,//0.0f,//0.005f
                .DeadBand = 0.0f,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 50,
                .MaxOut = 100,

            },
            .speed_PID = {
                .Kp = 1.60f,//并行扭矩量纲起始值(原级联2.1);阻尼,实车重标定
                .Ki = 0.0f,
                .Kd = 0.0f, //0.005f
                .Improve = PID_Integral_Limit | PID_Derivative_On_Measurement | PID_DerivativeFilter,
                .IntegralLimit = 5,
                .MaxOut = 100, //即输出扭矩TMAX，可通过达妙上位机读取或修改
                .Derivative_LPF_RC = 0.0f
               // .Output_LPF_RC = 0.003f
            },
            .other_angle_feedback_ptr = &gimbal_IMU_data->YawTotalAngle,
            // 还需要增加角速度额外反馈指针,注意方向,ins_task.md中有c板的bodyframe坐标系说明
            .other_speed_feedback_ptr = &gimbal_IMU_data->Gyro[2],
            //设置目标值
            .angle_ref_ptr = &yaw_cmd->ref_pos,
            .speed_ref_ptr = &yaw_cmd->ref_vel,
            .current_feedforward_ptr = &yaw_current,
        },
        .controller_setting_init_config = {
            .angle_feedback_source = OTHER_FEED,
            .speed_feedback_source = OTHER_FEED,
#if SYS_ID_MODE_ON
            .loop_type = CURRENT_LOOP,
            .feedforward_flag = FEEDFORWARD_NONE,
#else
            .loop_type = ANGLE_SPEED_CASCADE,
            .feedforward_flag = CURRENT_FEEDFORWARD,
#endif
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = DM4310,
        .DM_P_MAX = 3.14159,
        .DM_V_MAX = 30,
        .DM_T_MAX = 10
    };
    // PITCH
    pitch_config = (Motor_Init_Config_s){
        .can_init_config = {
            .can_handle = &hcan2,
            .tx_id = 0x09,  //slave id
            .rx_id = 0x1a,  //MASTER ID
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 27.0f,//并行扭矩量纲起始值(原级联50);实车重标定
                .Ki = 0.0f,
                .Kd = 0.0f,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 0.3f,
                .MaxOut = 5,

            },
            .speed_PID = {
                .Kp = 4.0f,//并行扭矩量纲起始值(原级联2);阻尼,实车重标定
                .Ki = 1.0f,
                .Kd = 0.0f,
                .Improve = PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 5,
                .MaxOut = 10,
                //.Derivative_LPF_RC = 0.02f
            },
            .other_angle_feedback_ptr = &gimbal_IMU_data->Pitch,//&gimba_IMU_data->Pitch,
            // 还需要增加角速度额外反馈指针,注意方向,ins_task.md中有c板的bodyframe坐标系说明
            .other_speed_feedback_ptr = &gimbal_IMU_data->Gyro[0],//(&gimba_IMU_data->Gyro[0]),
            .angle_ref_ptr = &pitch_cmd->ref_pos,
            .speed_ref_ptr = &pitch_cmd->ref_vel,
            .current_feedforward_ptr = &pitch_current,
        },
        .controller_setting_init_config = {
            .angle_feedback_source = OTHER_FEED,
            .speed_feedback_source = OTHER_FEED,
#if PITCH_SYS_ID_MODE_ON
#if PITCH_SYS_ID_GRAVITY_FF
            .feedforward_flag = CURRENT_FEEDFORWARD,
#else
            .feedforward_flag =  FEEDFORWARD_NONE,
#endif
#else
            .feedforward_flag = CURRENT_FEEDFORWARD,
#endif
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
            .loop_type = ANGLE_SPEED_CASCADE,
        },
        .motor_type = DM4310,
        .DM_P_MAX = 3.14159f,
        .DM_V_MAX = 30,
        .DM_T_MAX = 10
    };
    pitch_cmd->eso_initialized=0;
    pitch_cmd->eso_bandwidth = 27.0f;
    pitch_cmd->eso_gain=0.1f;
    pitch_cmd->eso_limit=0.8f;
    // 电机对total_angle闭环,上电时为零,会保持静止,收到遥控器数据再动
    yaw_motor = DMMotorInit(&yaw_config);
    pitch_motor = DMMotorInit(&pitch_config);

    gimbal_pub = PubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    gimbal_sub = SubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
}

/* 遥控模式的串级 PID 参数，Yaw/Pitch 及角度环/速度环分别配置。 */
static Motor_Controller_Init_s yaw_controller_config = {
    .angle_PID = {
        .Kp = 30.0f,//并行扭矩量纲起始值(原级联2.1);阻尼,实车重标定
        .Ki = 0.0f,
        .Kd = 0.0f, //0.005f
        .MaxOut = 100,
    },
    .speed_PID = {
        .Kp = 1.60f,//并行扭矩量纲起始值(原级联30);实车重标定
        .Ki = 0.0f,//0
        .Kd = 0.0f,//0.0f,//0.005f
        .MaxOut = 100,
       },
};

static Motor_Controller_Init_s pitch_controller_config = {
    .angle_PID = {
        .Kp = 27.0f,//并行扭矩量纲起始值(原级联2);阻尼,实车重标定
        .Ki = 0.0f,
        .Kd = 0.0f,
        .MaxOut = 5,
    },
    .speed_PID = {
        .Kp = 1.0f,//并行扭矩量纲起始值(原级联50);实车重标定
        .Ki = 0.0f,
        .Kd = 0.0f,
        .MaxOut = 10
     },
};

static void GimbalUpdate_PIDConfig()
{
    if (last_pid_auto_aim_mode == gimbal_cmd_recv.auto_aim_mode)
        return;

    if (gimbal_cmd_recv.auto_aim_mode) {
        //视觉模式
        PIDInit(&yaw_motor->motor_controller.angle_PID, &yaw_config.controller_param_init_config.angle_PID);
        PIDInit(&yaw_motor->motor_controller.speed_PID, &yaw_config.controller_param_init_config.speed_PID);

        PIDInit(&pitch_motor->motor_controller.angle_PID, &pitch_config.controller_param_init_config.angle_PID);
        PIDInit(&pitch_motor->motor_controller.speed_PID, &pitch_config.controller_param_init_config.speed_PID);

    } else {
        //遥控模式
        PIDInit(&yaw_motor->motor_controller.angle_PID, &yaw_controller_config.angle_PID);
        PIDInit(&yaw_motor->motor_controller.speed_PID, &yaw_controller_config.speed_PID);

        PIDInit(&pitch_motor->motor_controller.angle_PID, &pitch_controller_config.angle_PID);
        PIDInit(&pitch_motor->motor_controller.speed_PID, &pitch_controller_config.speed_PID);

    }
    last_pid_auto_aim_mode = gimbal_cmd_recv.auto_aim_mode;
}


/* 机器人云台控制核心任务,后续考虑只保留IMU控制,不再需要电机的反馈 */
void GimbalTask() {
    //红点激光
    //HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_SET);
    // 获取云台控制数据
    // 后续增加未收到数据的处理
    SubGetMessage(gimbal_sub, &gimbal_cmd_recv);

    CTC_Gimbal_Feedback_s ctc_fb = {
        .Pitch = gimbal_IMU_data->Pitch,
        .YawTotalAngle = gimbal_IMU_data->YawTotalAngle,
        .pitch_gyro = gimbal_IMU_data->Gyro[0],
        .pitch_motor_angle = gimbal_feedback_data.pitch_motor_single_round_angle,
        .pitch_motor_control_out = pitch_motor->motor_controller.control_out,
        .pitch_motor_lost_flag = gimbal_feedback_data.pitch_motor_lost_flag
    };
    CTC_Set_Gimbal_Feedback(&ctc_fb);

    CTC_Controller *yaw_cmd = CTC_Get_Yaw_Cmd();
    CTC_Controller *pitch_cmd = CTC_Get_Pitch_Cmd();
    GimbalUpdate_PIDConfig();
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
            pitch_cmd->eso_enabled = 0U;
            yaw_motor->motor_controller.speed_PID.ITerm = 0.0f;
            yaw_motor->motor_controller.speed_PID.Iout = 0.0f;
            pitch_motor->motor_controller.speed_PID.ITerm = 0.0f;
            pitch_motor->motor_controller.speed_PID.Iout = 0.0f;
            yaw_current = 0.0f;
            yaw_cmd->ref_vel = 0.0f;
            yaw_cmd->ref_acc = 0.0f;
            pitch_cmd->ref_vel = 0.0f;
            pitch_cmd->ref_acc = 0.0f;

            last_gimbal_mode = GIMBAL_ZERO_FORCE;
            break;

            // 使用陀螺仪的反馈,底盘根据yaw电机的offset跟随云台或视觉模式采用
            case GIMBAL_GYRO_MODE: // 后续只保留此模式
            DMMotorEnable(yaw_motor);
            DMMotorEnable(pitch_motor);
#if !(SYS_ID_MODE_ON ||PITCH_SYS_ID_MODE_ON)
            if (gimbal_cmd_recv.auto_aim_mode == 1) {
                pitch_cmd->eso_enabled = 1U;
                // 视觉自瞄: 用视觉给的 yaw_v/yaw_acc
                yaw_cmd->ref_pos = gimbal_cmd_recv.yaw;
                yaw_cmd->ref_vel = gimbal_cmd_recv.vision_yaw_speed_ff;
                yaw_cmd->ref_acc = gimbal_cmd_recv.vision_yaw_acc_ff;
                pitch_cmd->ref_pos = gimbal_cmd_recv.pitch;
                pitch_cmd->ref_vel = gimbal_cmd_recv.vision_pitch_speed_ff;
                pitch_cmd->ref_acc = gimbal_cmd_recv.vision_pitch_acc_ff;

                DMMotorLoopType(yaw_motor, ANGLE_SPEED_PARALLEL);
                DMMotorLoopType(pitch_motor, ANGLE_SPEED_PARALLEL);
            } else {
                pitch_cmd->eso_enabled = 0U;
                const float DEADZONE = 0.01f;
                static uint8_t last_had_input_flag = 0;
                uint8_t had_input_flag = (fabsf(gimbal_cmd_recv.yaw_speed) > DEADZONE || fabsf(gimbal_cmd_recv.pitch_speed) > DEADZONE);
                uint8_t yaw_looptype , pitch_looptype;
                if (had_input_flag) {
                    //有输入，速度单环
                    pitch_looptype = CTC_Pitch_Control(1, gimbal_cmd_recv.pitch_speed, 0);
                    yaw_looptype = CTC_Yaw_Control(1, gimbal_cmd_recv.yaw_speed, 0);
                    DMMotorLoopType(pitch_motor,pitch_looptype);
                    DMMotorLoopType(yaw_motor,yaw_looptype);
                }else {
                    //无输入，角度保持
                    uint8_t need_update = (last_auto_aim_mode == 1 ||last_had_input_flag || last_gimbal_mode == GIMBAL_ZERO_FORCE);
                    pitch_looptype = CTC_Pitch_Control(0, 0.0f, need_update);
                    yaw_looptype = CTC_Yaw_Control(0, 0.0f, need_update);
                    DMMotorLoopType(pitch_motor,pitch_looptype);
                    DMMotorLoopType(yaw_motor,yaw_looptype);
                }
                last_had_input_flag = had_input_flag;
            }
            last_auto_aim_mode = gimbal_cmd_recv.auto_aim_mode;
            last_gimbal_mode = gimbal_cmd_recv.gimbal_mode;

#endif

#if SYS_ID_MODE_ON
            // 辨识输入必须是纯 chirp，不能叠加正常闭环和模型前馈。
            yaw_current = 0.0f;
            sys_id_target_speed = Gimbal_SysID_Generate_Chirp(0.001f);
            DMMotorSetRef(yaw_motor, sys_id_target_speed);
            DMMotorSetRef(pitch_motor, 0.0f);
#elif PITCH_SYS_ID_MODE_ON
            // pitch 辨识:  chirp, 保留重力补偿; yaw 不给前馈
            yaw_current = 0.0f;
            yaw_cmd->ref_pos=0;
            // 开环保护: 角度越限 → 力矩 0
            sys_id_target_speed = (fabsf(gimbal_IMU_data->Pitch) > PITCH_SYS_ID_LIMIT_RAD)
                                      ? 0.0f
                                      : Gimbal_SysID_Generate_Chirp(0.001f);
            pitch_cmd->ref_pos = sys_id_target_speed;
            // DMMotorSetRef(pitch_motor, sys_id_target_speed);
#else

            yaw_current = Torque_ff_Yaw(yaw_cmd);
            pitch_current = Torque_ff_Pitch(pitch_cmd, gimbal_IMU_data->Pitch);
#endif

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
            break;
        }
    }

        // 设置反馈数据,主要是imu和yaw的ecd
        gimbal_feedback_data.gimbal_imu_data = *gimbal_IMU_data;
        gimbal_feedback_data.yaw_motor_single_round_angle = yaw_motor->measure.angle_single_round;
        gimbal_feedback_data.yaw_motor_sum_round_angle= yaw_motor->measure.total_angle;
        gimbal_feedback_data.pitch_motor_single_round_angle= pitch_motor->measure.angle_single_round;
        gimbal_feedback_data.pitch_motor_lost_flag = pitch_motor->lost_flag;
        // 推送消息
        PubPushMessage(gimbal_pub, (void *) &gimbal_feedback_data);
}
