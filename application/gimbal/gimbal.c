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

#define Friction_Smooth 100.0f
#define J  0.02552f    // kg*m^2
#define B  0.0731065f  // N*m*s/rad
#define Tc 0.0125219f  // N*m
#define Ku 0.808712f   // N*m/unit


static attitude_t *gimba_IMU_data; // 云台IMU数据
static DMMotorInstance *yaw_motor;
static DMMotorInstance  *pitch_motor;

static Publisher_t *gimbal_pub;                   // 云台应用消息发布者(云台反馈给cmd)
static Subscriber_t *gimbal_sub;                  // cmd控制消息订阅者
static Gimbal_Upload_Data_s gimbal_feedback_data; // 回传给cmd的云台状态信息
static Gimbal_Ctrl_Cmd_s gimbal_cmd_recv;         // 来自cmd的控制信息
static float gravity_current = 0;
static float yaw_current = 0;
static float theta;
static float sys_id_target_speed;
static Gimbal_GrayFF yaw_gray_ff;

// --- 系统辨识配置宏 ---
#define SYS_ID_MODE_ON  0       // 1: 开启辨识, 0: 关闭
#define SYS_ID_AMP      2.0f   // 激励振幅 (单位: rad/s，请根据电机响应调整，不要太大)
#define SYS_ID_MIN_FREQ 0.5f    // 起始频率 Hz
#define SYS_ID_MAX_FREQ 100.0f   // 终止频率 Hz
#define SYS_ID_DURATION 90.0f   // 扫频持续时间 秒

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

    // 激励信号: A * sin(2 * PI * f(t) * t) -> 注意：变频信号相位是频率的积分
    // Phase = 2 * PI * (f_start * t + 0.5 * k * t^2)
    float phase = 2.0f * PI * (SYS_ID_MIN_FREQ * sys_id_tick + 0.5f * k * sys_id_tick * sys_id_tick);

    // 3. 对 2*PI 取模，防止 float 精度溢出
    float phase_wrapped = fmodf(phase, 2.0f * PI);

    float output = SYS_ID_AMP * arm_sin_f32(phase_wrapped);

    sys_id_tick += dt;
    return output;
}

#define YAW_SINE_TEST_ENABLE 1

static float Sine_Test() {

    static float test_tick = 0.0f;
    float dt = 0.001f;
    test_tick += dt;

    // 参数：振幅 5°(0.087rad)，频率 1Hz(2*PI*1)
    float amp = 15.0f * DEGREE_2_RAD;    // 5度振幅，安全
    float freq = 2.0f;                  // 从1Hz开始，别贪快
    float sine_angle = amp * arm_sin_f32(2.0f * PI * freq * test_tick);

    // 用正弦波覆盖原有的 yaw 指令
    return sine_angle ;
}


// --- 本地参考轨迹生成器 (2阶参考滤波器) ---
// 把目标角度展开成平滑的 ref_pos/ref_vel/ref_acc;无视觉时也用本地生成前馈,
// 视觉自瞄时再由 GimbalTask 用视觉 yaw_v/yaw_acc 覆盖 ref_vel/ref_acc。
typedef struct {
    float pos_f;   // 平滑后目标位置 (rad)
    float vel_ref; // 期望角速度 (rad/s)
    float acc_ref; // 期望角加速度 (rad/s²)
    float fbk_pos;
    float k;
} RefTraj_t;

static RefTraj_t yaw_traj;

static void RefTraj_Step(RefTraj_t *t, float theta_raw_rad, float dt)
{
    if (YAW_SINE_TEST_ENABLE) {
        // 你可以单独定义一个宏
        theta_raw_rad=Sine_Test();

    }
    const float wn = 2.0f * PI * t->k; // 参考带宽(Hz)
    const float zeta = 0.8f;           // 临界阻尼, 无超调
    t->acc_ref = wn * wn * (theta_raw_rad - t->pos_f) - 2.0f * zeta * wn * t->vel_ref;
    t->vel_ref += t->acc_ref * dt;
    t->pos_f   += t->vel_ref * dt;
}

static float Compute_Torque(Gimbal_GrayFF *ff)
{
    ff->fbk_pos = gimba_IMU_data->YawTotalAngle *DEGREE_2_RAD;
    ff->fbk_vel = gimba_IMU_data->Gyro[2];
    // ref_pos/ref_vel/ref_acc 由调用方(GimbalTask)通过本地轨迹生成 + 视觉覆盖填好, 不再从指针读取

    ff->torque_ff = J * ff->ref_acc + B * ff->ref_vel + Tc * tanhf(Friction_Smooth * ff->ref_vel);

    // 并行复用 yaw 电机的 angle_PID / speed_PID 实例(输出都是扭矩,求和), 增益取配置
    ff->torque_p = PIDCalculate(&yaw_motor->motor_controller.angle_PID, ff->fbk_pos, ff->ref_pos);
    ff->torque_v = PIDCalculate(&yaw_motor->motor_controller.speed_PID, ff->fbk_vel, ff->ref_vel);

    ff->out_current = ff->torque_ff /Ku + ff->torque_p + ff->torque_v;
    return ff->out_current;
}


void GimbalInit() {
    gimba_IMU_data = INS_Init(); // IMU先初始化,获取姿态数据指针赋给yaw电机的其他数据来源
    // ref_vel/ref_acc 改由 GimbalTask 本地轨迹生成 + 视觉覆盖填给 yaw_gray_ff, 不再用指针指向视觉字段
    // YAW
    Motor_Init_Config_s yaw_config = {
            .can_init_config = {
                    .can_handle = &hcan1,
                    .tx_id = 0x10, //
                    .rx_id = 0x1b
            },
            .controller_param_init_config = {
                    .angle_PID = {
                            .Kp = 0.0f,//并行扭矩量纲起始值(原级联30);实车重标定
                            .Ki = 0.0f,//0
                            .Kd = 0.0f,//0.0f,//0.005f
                            .DeadBand = 0.0f,
                            .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                            .IntegralLimit = 500,
                            .MaxOut = 100,
                    },
                    .speed_PID = {
                            .Kp = 0.0f,//并行扭矩量纲起始值(原级联2.1);阻尼,实车重标定
                            .Ki = 0.0f,
                            .Kd = 0.0f, //0.005f
                            .Improve = PID_Integral_Limit | PID_Derivative_On_Measurement | PID_DerivativeFilter,
                            .IntegralLimit = 5,
                            .MaxOut = 100, //即输出扭矩TMAX，可通过达妙上位机读取或修改
                            .Derivative_LPF_RC = 0.0f
                           // .Output_LPF_RC = 0.003f
                    },
                    .other_angle_feedback_ptr = &gimba_IMU_data->YawTotalAngle,
                    // 还需要增加角速度额外反馈指针,注意方向,ins_task.md中有c板的bodyframe坐标系说明
                    .other_speed_feedback_ptr = &gimba_IMU_data->Gyro[2],
                    .current_feedforward_ptr = &yaw_current,

            },
            .controller_setting_init_config = {
                    .angle_feedback_source = OTHER_FEED,
                    .speed_feedback_source = OTHER_FEED,
                    .outer_loop_type = ANGLE_LOOP,
                    .close_loop_type = OPEN_LOOP,      //改成并行了。
                    // .outer_loop_type = SPEED_LOOP,
                    // .close_loop_type = SPEED_LOOP,
                    .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
                    .feedforward_flag = CURRENT_FEEDFORWARD,
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

    yaw_traj.k=7.0f;
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
            yaw_current = 0.0f;
            yaw_gray_ff.ref_vel = 0.0f;
            yaw_gray_ff.ref_acc = 0.0f;
            // 同步参考轨迹到当前实际角度, 防止进 GYRO 时猛冲
            yaw_traj.pos_f = gimba_IMU_data->YawTotalAngle * DEGREE_2_RAD;

            break;
            // 使用陀螺仪的反馈,底盘根据yaw电机的offset跟随云台或视觉模式采用
        case GIMBAL_GYRO_MODE: // 后续只保留此模式
            DMMotorEnable(yaw_motor);
            DMMotorEnable(pitch_motor);

            // 本地参考轨迹整形: 把 yaw_ref(主目标) 展开成平滑的 ref_pos; 视觉/无视觉共用平滑位置
            RefTraj_Step(&yaw_traj, gimbal_cmd_recv.yaw * DEGREE_2_RAD, 0.001f);
            yaw_gray_ff.ref_pos = yaw_traj.pos_f;

            if (gimbal_cmd_recv.auto_aim_mode == 1) {
                // 视觉自瞄: 用视觉给的 yaw_v/yaw_acc(更好的外部预测)
                yaw_gray_ff.ref_vel = gimbal_cmd_recv.vision_yaw_speed_ff * DEGREE_2_RAD;
                yaw_gray_ff.ref_acc = gimbal_cmd_recv.vision_yaw_acc_ff * DEGREE_2_RAD;
            } else {
                // 无视觉: 用本地轨迹生成的 ref_vel/ref_acc
                yaw_gray_ff.ref_vel = yaw_traj.vel_ref;
                yaw_gray_ff.ref_acc = yaw_traj.acc_ref;
            }

            yaw_current = Compute_Torque(&yaw_gray_ff);
            
            DMMotorSetRef(yaw_motor, 0.0f);
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
                        sys_id_target_speed = Gimbal_SysID_Generate_Chirp(0.001f);
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
            yaw_current = 0.0f;
            yaw_gray_ff.ref_vel = 0.0f;
            yaw_gray_ff.ref_acc = 0.0f;
            // 同步参考轨迹到当前实际角度, 防止回 GYRO 时猛冲
            yaw_traj.pos_f   = gimba_IMU_data->YawTotalAngle * DEGREE_2_RAD;

            break;
        default:
            break;
        }
    }
    // 根据IMU姿态/pitch电机角度反馈计算出当前配重下的重力矩

    theta = gimba_IMU_data->Pitch/180*PI;
    if (theta>0.4) {//0.6
        gravity_current = 0.1f;
        //gravity_current = 0;
    }
    else {
        gravity_current = 0.8383*arm_cos_f32(theta+1.0106)-0.2;
        //gravity_current = 0;
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
