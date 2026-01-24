// /**
//  * @file chassis.c
//  * @author NeoZeng neozng1@hnu.edu.cn
//  * @brief 底盘应用,负责接收robot_cmd的控制命令并根据命令进行运动学解算,得到输出
//  *        注意底盘采取右手系,对于平面视图,底盘纵向运动的正前方为x正方向;横向运动的右侧为y正方向
//  *
//  * @version 0.1
//  * @date 2022-12-04
//  *
//  * @copyright Copyright (c) 2022
//  *
//  */
//
// #include "chassis.h"
// #include "robot_def.h"
// #include "dji_motor.h"
// #include "super_cap.h"
// #include "message_center.h"
// #include "referee_task.h"
//
// #include "general_def.h"
// #include "bsp_dwt.h"
// #include "referee_UI.h"
// #include "arm_math.h"
// #include "user_lib.h"
//
// #include "vofa.h"
//
// /* 根据robot_def.h中的macro自动计算的参数 */
// #define HALF_WHEEL_BASE (WHEEL_BASE / 2.0f)     // 半轴距
// #define HALF_TRACK_WIDTH (TRACK_WIDTH / 2.0f)   // 半轮距
// #define PERIMETER_WHEEL (RADIUS_WHEEL * 2 * PI) // 轮子周长
//
// #define P_MAX 50.0f//功率控制 单位：W
//
// /* 底盘应用包含的模块和信息存储,底盘是单例模式,因此不需要为底盘建立单独的结构体 */
// #ifdef CHASSIS_BOARD // 如果是底盘板,使用板载IMU获取底盘转动角速度
// #include "can_comm.h"
// #include "ins_task.h"
// static CANCommInstance *chasiss_can_comm; // 双板通信CAN comm
// attitude_t *Chassis_IMU_data;
// #endif // CHASSIS_BOARD
// #ifdef ONE_BOARD
// static Publisher_t *chassis_pub;                    // 用于发布底盘的数据
// static Subscriber_t *chassis_sub;                   // 用于订阅底盘的控制命令
// #endif                                              // !ONE_BOARD
// static Chassis_Ctrl_Cmd_s chassis_cmd_recv;         // 底盘接收到的控制命令
// static Chassis_Upload_Data_s chassis_feedback_data; // 底盘回传的反馈数据
// static Chassis_Ctrl_Cmd_s Chassis_Power_Mx;
// const static float CHASSIS_ACCEL_X_NUM=0.1f;
// const static float CHASSIS_ACCEL_Y_NUM=0.1f;
//
// // 超级电容
// SuperCapInstance *cap;    // 超级电容全局
// static uint16_t last_chassis_power_limit=0;//超级电容更新
// static DJIMotorInstance *motor_lf, *motor_rf, *motor_lb, *motor_rb; // left right forward back
//
// /* 用于自旋变速策略的时间变量 */
// // static float t;
//
// /* 私有函数计算的中介变量,设为静态避免参数传递的开销 */
// static float chassis_vx, chassis_vy;     // 将云台系的速度投影到底盘
// static float vt_lf, vt_rf, vt_lb, vt_rb; // 底盘速度解算后的临时输出,待进行限幅
// first_order_filter_type_t vx_filter;
// first_order_filter_type_t vy_filter;
//
// void ChassisInit() {
//     // 四个轮子的参数一样,改tx_id和反转标志位即可
//     Motor_Init_Config_s chassis_motor_config = {
//             .can_init_config.can_handle = &hcan1,
//             .controller_param_init_config = {
//                     .speed_PID = {
//                             .Kp = 0.0f,
//                             .Ki = 0.0f,
//                             .Kd = 0.0f,
//                             .IntegralLimit = 3000,
//                             .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
//                             .MaxOut = 30000,
//                     },
//                     .current_PID = {
//                             .Kp = 0.0f,
//                             .Ki = 0.0f,
//                             .Kd = 0.0f,
//                             .IntegralLimit = 3000,
//                             .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
//                             .MaxOut = 15000,
//                     },
//             },
//             .controller_setting_init_config = {
//                     .angle_feedback_source = MOTOR_FEED,
//                     .speed_feedback_source = MOTOR_FEED,
//                     .outer_loop_type = SPEED_LOOP,
//                     .close_loop_type = SPEED_LOOP, // CURRENT_LOOP,
//                     .power_limit_flag = POWER_LIMIT_ON,
//             },
//             .motor_type = M3508,
//     };
//     //  @todo: 当前还没有设置电机的正反转,仍然需要手动添加reference的正负号,需要电机module的支持,待修改.
//     // 四个轮子pid分开
//     //右前
//     chassis_motor_config.can_init_config.tx_id = 1;
//     chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
//     chassis_motor_config.controller_param_init_config.speed_PID.Kp = 5.0f;
//     chassis_motor_config.controller_param_init_config.speed_PID.Ki = 0.01f;
//     chassis_motor_config.controller_param_init_config.speed_PID.Kd = 0.002f;
//     motor_rf = DJIMotorInit(&chassis_motor_config);
//     //左前
//     chassis_motor_config.can_init_config.tx_id = 2;
//     chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
//     chassis_motor_config.controller_param_init_config.speed_PID.Kp = 5.0f;
//     chassis_motor_config.controller_param_init_config.speed_PID.Ki = 0.01f;
//     chassis_motor_config.controller_param_init_config.speed_PID.Kd = 0.002f;
//     motor_lf = DJIMotorInit(&chassis_motor_config);
//     //左后
//     chassis_motor_config.can_init_config.tx_id = 3;
//     chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
//     chassis_motor_config.controller_param_init_config.speed_PID.Kp = 5.5f;
//     chassis_motor_config.controller_param_init_config.speed_PID.Ki = 0.01f;
//     chassis_motor_config.controller_param_init_config.speed_PID.Kd = 0.002f;
//     motor_lb = DJIMotorInit(&chassis_motor_config);
//     //右后
//     chassis_motor_config.can_init_config.tx_id = 4;
//     chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
//     chassis_motor_config.controller_param_init_config.speed_PID.Kp = 5.5f;
//     chassis_motor_config.controller_param_init_config.speed_PID.Ki = 0.01f;
//     chassis_motor_config.controller_param_init_config.speed_PID.Kd = 0.002f;
//     motor_rb = DJIMotorInit(&chassis_motor_config);
//
//     //超级电容
//     SuperCap_Init_Config_s cap_conf = {
//             .can_config = {
//                     .can_handle = &hcan1,
//                     .tx_id = 0x210,
//                     .rx_id = 0x211,
//
//             },
//             .buffer_config_pid = {
//                     .Kp = 1.0f,
//                     .Ki = 0,
//                     .Kd = 0,
//                     .MaxOut = 300,
//             },
//     };
//     cap = SuperCapInit(&cap_conf); // 超级电容初始化
//
//     //用一阶滤波代替斜波函数生成                                     //增大更能刹住
//     first_order_filter_init(&vx_filter, 0.007f, &CHASSIS_ACCEL_X_NUM);
//     first_order_filter_init(&vy_filter, 0.007f, &CHASSIS_ACCEL_Y_NUM);
//
//     // 发布订阅初始化,如果为双板,则需要can comm来传递消息
// #ifdef CHASSIS_BOARD
//     Chassis_IMU_data = INS_Init(); // 底盘IMU初始化
//
//     CANComm_Init_Config_s comm_conf = {
//         .can_config = {
//             .can_handle = &hcan2,
//             .tx_id = 0x311,
//             .rx_id = 0x312,
//         },
//         .recv_data_len = sizeof(Chassis_Ctrl_Cmd_s),
//         .send_data_len = sizeof(Chassis_Upload_Data_s),
//     };
//     chasiss_can_comm = CANCommInit(&comm_conf); // can comm初始化
// #endif                                          // CHASSIS_BOARD
//
// #ifdef ONE_BOARD // 单板控制整车,则通过pubsub来传递消息
//     chassis_sub = SubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
//     chassis_pub = PubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
// #endif // ONE_BOARD
// }
//
// #define LF_CENTER ((HALF_TRACK_WIDTH + CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE - CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
// #define RF_CENTER ((HALF_TRACK_WIDTH - CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE - CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
// #define LB_CENTER ((HALF_TRACK_WIDTH + CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE + CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
// #define RB_CENTER ((HALF_TRACK_WIDTH - CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE + CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
//
// /**
//  * @brief 计算每个轮毂电机的输出,正运动学解算
//  *        用宏进行预替换减小开销,运动解算具体过程参考教程
//  */
// //全向轮解算
// static void OmniCalculate() {
//     vt_rf = -(HALF_TRACK_WIDTH + HALF_WHEEL_BASE) * chassis_cmd_recv.wz + chassis_vx - chassis_vy;
//     vt_rb = -(HALF_TRACK_WIDTH + HALF_WHEEL_BASE) * chassis_cmd_recv.wz - chassis_vx - chassis_vy;
//     vt_lb = -(HALF_TRACK_WIDTH + HALF_WHEEL_BASE) * chassis_cmd_recv.wz - chassis_vx + chassis_vy;
//     vt_lf = -(HALF_TRACK_WIDTH + HALF_WHEEL_BASE) * chassis_cmd_recv.wz + chassis_vx + chassis_vy;
//
//     vt_rf /= (WHEEL_BASE * 1.414f);
//     vt_rb /= (WHEEL_BASE * 1.414f);
//     vt_lb /= (WHEEL_BASE * 1.414f);
//     vt_lf /= (WHEEL_BASE * 1.414f);
// }
//
// static const float motor_power_K[3] = {1.6301e-6f,5.7501e-7f,2.5863e-7f};
// float input;
//
// extern float P_max = 0;
//
// ///依据3508电机功率模型，预测电机输出功率
// static float EstimatePower(DJIMotorInstance* chassis_motor)
// {
//
//     float I_cmd = chassis_motor->motor_controller.current_PID.Output;
//     float w = chassis_motor->measure.speed_aps /6 ;//aps to rpm
//
//     float power = motor_power_K[0] * I_cmd * w + motor_power_K[1]*w*w  + motor_power_K[2]*I_cmd*I_cmd;
//
//     return power;
// }
// /**
//  * @brief 根据裁判系统和电容剩余容量对输出进行限制并设置电机参考值
//  *
//  */
// static void LimitChassisOutput()
// {
//     //    float Plimit = 1.0f;
//     float P_cmd = motor_rf->motor_controller.motor_power_predict +
//                   motor_rb->motor_controller.motor_power_predict +
//                   motor_lb->motor_controller.motor_power_predict +
//                   motor_lf->motor_controller.motor_power_predict + 3.6f;
//     //    if(chassis_cmd_recv.buffer_energy<50&&chassis_cmd_recv.buffer_energy>=40)	    Plimit=0.9f;
//     //    else if(chassis_cmd_recv.buffer_energy<40&&chassis_cmd_recv.buffer_energy>=35)	Plimit=0.75f;
//     //    else if(chassis_cmd_recv.buffer_energy<35&&chassis_cmd_recv.buffer_energy>=30)	Plimit=0.5f;
//     //    else if(chassis_cmd_recv.buffer_energy<30&&chassis_cmd_recv.buffer_energy>=20)	Plimit=0.25f;
//     //    else if(chassis_cmd_recv.buffer_energy<20&&chassis_cmd_recv.buffer_energy>=10)	Plimit=0.125f;
//     //    else if(chassis_cmd_recv.buffer_energy<10&&chassis_cmd_recv.buffer_energy>=0)	Plimit=0.05f;
//     //    else if(chassis_cmd_recv.buffer_energy==60)					                    Plimit=1.0f;
//
//     if (cap->cap_msg.cap_vol > 1800)
//     {
//         P_max = input + chassis_cmd_recv.buffer_supercap;
//     }
//     else
//     {
//         P_max = input;
//     }
//     // float K = P_max / P_cmd;
//     float error[3];
//     error[0] = motor_rf->motor_controller.pid_ref * PI / 180 - motor_rf->measure.speed_aps * PI / 180;
//     error[1] = motor_rb->motor_controller.pid_ref * PI / 180 - motor_rb->measure.speed_aps * PI / 180;
//     error[2] = motor_lf->motor_controller.pid_ref * PI / 180 - motor_rf->measure.speed_aps * PI / 180;
//     error[3] = motor_lb->motor_controller.pid_ref * PI / 180 - motor_rf->measure.speed_aps * PI / 180;
//
//     float errorlower, errorupper, kcoe;
//     float totalSum = 0;
//     if ((error[0] + error[1] + error[2] + error[3]) < errorlower)
//     {
//         kcoe = 0;
//     }
//     else if ((error[0] + error[1] + error[2] + error[3]) > errorupper)
//     {
//         kcoe = 1;
//     }
//     else if (errorlower < (error[0] + error[1] + error[2] + error[3]) < errorupper)
//     {
//         for(int i = 0;i < 4;i++)
//         {
//             float sum = error[i] - errorlower;
//             totalSum += sum;
//         }
//         kcoe = totalSum/(errorupper - errorlower);
//     }
//     float  Pmaxrf,Pmaxrb,Pmaxlf,Pmaxlb,krf,krb,klf,klb;
//     krf = (kcoe * error[0]/(error[0]+error[1]+error[2]+error[3]) +
//             (1 - kcoe)*motor_rf->motor_controller.motor_power_predict/P_cmd)/P_max;
//     krb = kcoe * error[0]/(error[0]+error[1]+error[2]+error[3]) +
//                         (1 - kcoe)*motor_rb->motor_controller.motor_power_predict/P_cmd;
//     klf = kcoe * error[0]/(error[0]+error[1]+error[2]+error[3]) +
//                         (1 - kcoe)*motor_lf->motor_controller.motor_power_predict/P_cmd;
//     klb = kcoe * error[0]/(error[0]+error[1]+error[2]+error[3]) +
//                         (1 - kcoe)*motor_lb->motor_controller.motor_power_predict/P_cmd;
//             //    if(chassis_cmd_recv.buffer_energy<5)//当缓冲功率过小时，限制功率给小；
//        // K = (float)(chassis_cmd_recv.chassis_power_limit - 3) / P_cmd;
//     motor_rf->motor_controller.motor_power_scale = krf;
//     motor_rb->motor_controller.motor_power_scale = krb;
//     motor_lf->motor_controller.motor_power_scale = klf;
//     motor_lb->motor_controller.motor_power_scale = klb;
//
//     {
//         DJIMotorSetRef(motor_lf, vt_lf);
//         DJIMotorSetRef(motor_rf, vt_rf);
//         DJIMotorSetRef(motor_lb, vt_lb);
//         DJIMotorSetRef(motor_rb, vt_rb);
//     }
//
// }
// /**
//  * @brief 超电功率对缓冲功率进行闭环
//  *
//  *
//  */
// static void SuperCapSetUpdate()
// {
//
//     PIDCalculate(&cap->buffer_pid, chassis_cmd_recv.buffer_energy,30);//对缓冲功率进行闭环
//     input = chassis_cmd_recv.chassis_power_limit - cap->buffer_pid.Output;
//     LIMIT_MIN_MAX(input, 30, 130);
//     SuperCapSetPower(cap,input);
//
// }
// /**
//  * @brief 根据每个轮子的速度反馈,计算底盘的实际运动速度,逆运动解算
//  *        对于双板的情况,考虑增加来自底盘板IMU的数据
//  *
//  */
// static void EstimateSpeed() {
//     // 根据电机速度和陀螺仪的角速度进行解算,还可以利用加速度计判断是否打滑(如果有)
//     // chassis_feedback_data.vx vy wz =
//     //  ...
// }
// static chassis_mode_e last_chassis_mode;
// static float rotate_speed = 80000;
// /* 机器人底盘控制核心任务 */
// void ChassisTask() {
//     // 后续增加没收到消息的处理(双板的情况)
//     // 获取新的控制信息
// #ifdef ONE_BOARD
//     SubGetMessage(chassis_sub, &chassis_cmd_recv);
// #endif
// #ifdef CHASSIS_BOARD
//     chassis_cmd_recv = *(Chassis_Ctrl_Cmd_s *)CANCommGet(chasiss_can_comm);
// #endif // CHASSIS_BOARD
//
//     if (chassis_cmd_recv.chassis_mode == CHASSIS_ZERO_FORCE) { // 如果出现重要模块离线或遥控器设置为急停,让电机停止
//         DJIMotorStop(motor_lf);
//         DJIMotorStop(motor_rf);
//         DJIMotorStop(motor_lb);
//         DJIMotorStop(motor_rb);
//     } else { // 正常工作
//         DJIMotorEnable(motor_lf);
//         DJIMotorEnable(motor_rf);
//         DJIMotorEnable(motor_lb);
//         DJIMotorEnable(motor_rb);
//     }
//
//         // 根据控制模式设定旋转速度
//         switch (chassis_cmd_recv.chassis_mode) {
//             case CHASSIS_NO_FOLLOW: // 底盘不旋转,但维持全向机动,一般用于调整云台姿态
//                 chassis_cmd_recv.wz = 0;
//                 break;
//             case CHASSIS_FOLLOW_GIMBAL_YAW: // 跟随云台,不单独设置pid,以误差角度平方为速度输出
//                 chassis_cmd_recv.wz = 40.0f * chassis_cmd_recv.offset_angle * abs(chassis_cmd_recv.offset_angle);
//                 LIMIT_MIN_MAX(chassis_cmd_recv.wz,-40000,40000);
//                 break;
//             case CHASSIS_SIDEWAYS: // 侧向,不单独设置pid,以误差角度平方为速度输出
//                 chassis_cmd_recv.wz = 40.0f * (chassis_cmd_recv.offset_angle - 45)* abs(chassis_cmd_recv.offset_angle - 45);
//                 LIMIT_MIN_MAX(chassis_cmd_recv.wz,-40000,40000);
//                 break;
//             case CHASSIS_ROTATE: // 自旋,同时保持全向机动;当前wz维持定值,后续增加不规则的变速策略
//                 if(last_chassis_mode != CHASSIS_ROTATE){
//                     rotate_speed = -rotate_speed;
//                 }
//                 chassis_cmd_recv.wz = rotate_speed;
//                 break;
//             default:
//                 break;
//         }
//
//     last_chassis_mode = chassis_cmd_recv.chassis_mode;
//     // 根据云台和底盘的角度offset将控制量映射到底盘坐标系上
//     // 底盘逆时针旋转为角度正方向;云台命令的方向以云台指向的方向为x,采用右手系(x指向正北时y在正东)
//     static float sin_theta, cos_theta;
//     cos_theta = arm_cos_f32(chassis_cmd_recv.offset_angle * DEGREE_2_RAD);
//     sin_theta = arm_sin_f32(chassis_cmd_recv.offset_angle * DEGREE_2_RAD);
//
//     //一阶低通滤波计算
//     first_order_filter_cali(&vx_filter, chassis_cmd_recv.vx);
//     first_order_filter_cali(&vy_filter, chassis_cmd_recv.vy);
//
//     chassis_cmd_recv.vx = vx_filter.out;
//     chassis_cmd_recv.vy = vy_filter.out;
//
//     chassis_vx = chassis_cmd_recv.vx * cos_theta - chassis_cmd_recv.vy * sin_theta;
//     chassis_vy = chassis_cmd_recv.vx * sin_theta + chassis_cmd_recv.vy * cos_theta;
//
// //    chassis_vx = (1.0f - 0.30f) * chassis_vx + 0.30f * (chassis_cmd_recv.vx * cos_theta - chassis_cmd_recv.vy * sin_theta);
// //    chassis_vy = (1.0f - 0.30f) * chassis_vy + 0.30f * (chassis_cmd_recv.vx * sin_theta + chassis_cmd_recv.vy * cos_theta);
//
//
//     // 根据控制模式进行正运动学解算,计算底盘输出
//     //MecanumCalculate();
//     OmniCalculate();
//     ////对缓冲功率进行闭环
//     SuperCapSetUpdate();
//     // 根据裁判系统的反馈数据和电容数据对输出限幅并设定闭环参考值
//     LimitChassisOutput();
//
// //    float vofa_send_data[2];
// //    vofa_send_data[0] = motor_lb->motor_controller.speed_PID.Ref;
// //    vofa_send_data[1] = motor_lb->motor_controller.speed_PID.Measure;
// //    vofa_justfloat_output(vofa_send_data, 8, &huart1);
//
//     // 根据电机的反馈速度和IMU(如果有)计算真实速度
//     EstimateSpeed();
//
//     //todo: 裁判系统信息移植到消息中心发送
//     // 获取裁判系统数据   建议将裁判系统与底盘分离，所以此处数据应使用消息中心发送
//     // 发送敌方方颜色id
//     //chassis_feedback_data.enemy_color = !referee_data->referee_id.Robot_Color;
//     // 当前只做了17mm热量的数据获取,后续根据robot_def中的宏切换双枪管和英雄42mm的情况
//     //chassis_feedback_data.bullet_speed = referee_data->GameRobotState.shooter_id1_17mm_speed_limit;
//
//     chassis_feedback_data.cap_vol = cap->cap_msg.cap_vol;
//
//     // 推送反馈消息
// #ifdef ONE_BOARD
//     PubPushMessage(chassis_pub, (void *) &chassis_feedback_data);
// #endif
// #ifdef CHASSIS_BOARD
//     CANCommSend(chasiss_can_comm, (void *)&chassis_feedback_data);
// #endif // CHASSIS_BOARD
// }
