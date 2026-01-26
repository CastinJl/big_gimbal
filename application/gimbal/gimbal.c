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
uint32_t motor_model_count;

static float last_yaw_ref = 0;
static float dt = 0.001f; // 任务运行周期为5ms，也可用 DWT 获取真实 dt

static void LimitPitchAngleAuto() {
    /** 注意电机角度与陀螺仪角度方向是否相同
     * 目前 add > 0, 向下转动
     * 电机角度值减小
     * 陀螺仪角度值增大
     **/
    float add;
    float angle_set;

    add = gimbal_cmd_recv.pitch - gimba_IMU_data->Pitch;

    if(pitch_motor->measure.angle_single_round - add > PITCH_MAX_RELATIVE_ANGLE){
        if(add < 0.0f ){
            add = PITCH_MAX_ANGLE - pitch_motor->measure.angle_single_round;
        }
    }else if(pitch_motor->measure.angle_single_round - add < PITCH_MIN_RELATIVE_ANGLE){
        if(add > 0.0f){
            add = PITCH_MIN_RELATIVE_ANGLE - pitch_motor->measure.angle_single_round;
        }
    }
    angle_set = gimba_IMU_data->Pitch;
    DMMotorSetRef(pitch_motor, angle_set+add);
}

const double TARGET_ANGLE_HIGH = 30.0;  // 例如: +20 度
const double TARGET_ANGLE_LOW  = -15.0; // 例如: -15 度

// 2. 移位寄存器的位数 (通常 9 到 15 之间就足够了)
// 序列的长度是 (2^N)-1
#define LFSR_BITS 10

// 全局静态变量，用于保存移位寄存器的当前状态
// static 确保了它在函数调用之间保持其值
// 使用 volatile 可以在中断服务程序中安全使用
static volatile uint16_t lfsr_register = 0x0001; // 初始种子(seed)，不能为 0

/**
 * @brief 生成并返回下一个 PRBS 目标角度设定值
 * @note  此函数应该在一个固定的时间间隔被周期性地调用 (例如每 40ms)。
 * @return 返回 TARGET_ANGLE_HIGH 或 TARGET_ANGLE_LOW
 */
double generate_angle_setpoint(void) {
    // 根据 LFSR_BITS 选择反馈多项式 (taps)
    // 这是 PRBS 信号“随机性”的来源
    // 对于 10-bit, 多项式是 x^10 + x^3 + 1, 所以 taps 在第 9 和第 2 位 (0-indexed)
    const uint16_t taps = (1 << (10-1)) | (1 << (3-1));

    // 1. 获取输出位 (最低位)
    uint16_t output_bit = lfsr_register & 1;

    // 2. 寄存器右移一位
    lfsr_register >>= 1;

    // 3. 如果输出位是 1, 则与 taps 进行异或操作
    if (output_bit) {
        lfsr_register ^= taps;
    }

    // 4. 将二进制输出 {0, 1} 映射到高低两个目标角度
    if (output_bit) {
        return TARGET_ANGLE_HIGH;
    } else {
        return TARGET_ANGLE_LOW;
    }
}

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
                            .Kp = 50,//30//0.8f,//1.2
                            .Ki = 0.0f,//0
                            .Kd = 0.0f,//0.04f,//0.05
                            .DeadBand = 0.0f,
                            .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                            .IntegralLimit = 100,
                            .MaxOut = 20//500//300,

                    },
                    .speed_PID = {
                            .Kp = 10,//-400//1410,  //4000
                            .Ki = 0,//33.0f, //100
                            .Kd = 0.005f, //0
                            .Improve = PID_Integral_Limit | PID_Derivative_On_Measurement | PID_OutputFilter,                            .IntegralLimit = 10000,
                            .MaxOut = 30,
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
                    .close_loop_type = SPEED_LOOP | ANGLE_LOOP,
                    // .outer_loop_type = SPEED_LOOP,
                    // .close_loop_type = SPEED_LOOP,
                    .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
                    .feedforward_flag = SPEED_FEEDFORWARD,
            },
            .motor_type = DM4310,
            .DM_P_MAX = 3.14,
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
            .DM_P_MAX = 12.5,
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
    //     // 停止
    // case GIMBAL_ZERO_FORCE:
    //     DJIMotorStop(yaw_motor);
    //     DJIMotorStop(pitch_motor);
    //     break;
    //     // 使用陀螺仪的反馈,底盘根据yaw电机的offset跟随云台或视觉模式采用
    // case GIMBAL_GYRO_MODE: // 后续只保留此模式
    //     DJIMotorEnable(yaw_motor);
    //     DJIMotorEnable(pitch_motor);
    //     DJIMotorChangeFeed(yaw_motor, ANGLE_LOOP, OTHER_FEED);
    //     DJIMotorChangeFeed(yaw_motor, SPEED_LOOP, OTHER_FEED);
    //     DJIMotorChangeFeed(pitch_motor, ANGLE_LOOP, OTHER_FEED);
    //     DJIMotorChangeFeed(pitch_motor, SPEED_LOOP, OTHER_FEED);
    //     DJIMotorSetRef(yaw_motor, gimbal_cmd_recv.yaw); // yaw和pitch会在robot_cmd中处理好多圈和单圈
    //     DJIMotorSetRef(pitch_motor, gimbal_cmd_recv.pitch);
    //     //LimitPitchAngleAuto();
    //     break;
    //     // 云台自由模式,使用编码器反馈,底盘和云台分离,仅云台旋转,一般用于调整云台姿态(英雄吊射等)/能量机关
    // case GIMBAL_FREE_MODE: // 后续删除,或加入云台追地盘的跟随模式(响应速度更快)
    //     DJIMotorEnable(yaw_motor);
    //     DJIMotorEnable(pitch_motor);
    //     DJIMotorChangeFeed(yaw_motor, ANGLE_LOOP, OTHER_FEED);
    //     DJIMotorChangeFeed(yaw_motor, SPEED_LOOP, OTHER_FEED);
    //     DJIMotorChangeFeed(pitch_motor, ANGLE_LOOP, OTHER_FEED);
    //     DJIMotorChangeFeed(pitch_motor, SPEED_LOOP, OTHER_FEED);
    //     DJIMotorSetRef(yaw_motor, gimbal_cmd_recv.yaw); // yaw和pitch会在robot_cmd中处理好多圈和单圈
    //     DJIMotorSetRef(pitch_motor, gimbal_cmd_recv.pitch);
    //     break;
    // default:
    //     break;
        {
        case GIMBAL_ZERO_FORCE:
            DMMotorStop(yaw_motor);
            DMMotorStop(pitch_motor);

            break;
            // 使用陀螺仪的反馈,底盘根据yaw电机的offset跟随云台或视觉模式采用
        case GIMBAL_GYRO_MODE: // 后续只保留此模式
            DMMotorEnable(yaw_motor);
            DMMotorEnable(pitch_motor);
            float yaw_target_speed = (gimbal_cmd_recv.yaw - last_yaw_ref) * PI / 180 / dt;
            last_yaw_ref = gimbal_cmd_recv.yaw;
            float chassis_comp_speed = -gimbal_cmd_recv.real_wz;
            // 视觉预测前馈系数，默认 1.0，如果响应不够可适当加大
            float vision_kv = 1.3f;
            //速度前馈限幅
            if (yaw_target_speed > MAX_FF_SPEED_RAD) {
                yaw_target_speed = MAX_FF_SPEED_RAD;
            } else if (yaw_target_speed < -MAX_FF_SPEED_RAD) {
                yaw_target_speed = -MAX_FF_SPEED_RAD;
            }
            //yaw_target_speed = Yaw_FF_Filter(yaw_target_speed);   //速度前馈滤波，代价是响应速度
            // 速度前馈
            yaw_speed_feedforward = chassis_comp_speed + (yaw_target_speed * vision_kv);
            DMMotorSetRef(yaw_motor, gimbal_cmd_recv.yaw); // yaw和pitch会在robot_cmd中处理好多圈和单圈
            pitch_get = gimba_IMU_data->Pitch;
            // if (motor_model_count > 50) {
            //     motor_model_count = 0;
            //     pitch_set = generate_angle_setpoint();
            // }
            // motor_model_count++;
            DMMotorSetRef(pitch_motor,gimbal_cmd_recv.pitch);
            //DMMotorSetRef(pitch_motor,pitch_set);
            // // DMMotorSetRef(pitch_motor,5);
            // roll_get = gimba_IMU_data->Roll;
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

    yaw_speed_feedforward  -= gimbal_cmd_recv.real_wz * 180 / PI;
    theta = gimba_IMU_data->Pitch/180*PI;
    if (theta>0.4) {//0.6
        gravity_current = 0.1;
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