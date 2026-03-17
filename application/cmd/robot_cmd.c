// app
#include "robot_def.h"
#include "robot_cmd.h"
// module
#include "remote_control.h"
#include "ins_task.h"
#include "master_process.h"
#include "message_center.h"
#include "general_def.h"
#include "dji_motor.h"
#include "auto_aim.h"
#include "super_cap.h"
#include "Power.h"
#include "referee_VT.h"

// bsp
#include "bsp_dwt.h"
#include "bsp_log.h"
#include "referee_task.h"
#include "vision_transfer.h"

// 私有宏,自动将编码器转换成角度值
#define YAW_ALIGN_ANGLE (YAW_CHASSIS_ALIGN_ECD * ECD_ANGLE_COEF_DJI) // 对齐时的角度,0-360
#define PTICH_HORIZON_ANGLE (PITCH_HORIZON_ECD * ECD_ANGLE_COEF_DJI) // pitch水平时电机的角度,0-360

/* cmd应用包含的模块实例指针和交互信息存储*/
#ifdef GIMBAL_BOARD // 对双板的兼容,条件编译
#include "can_comm.h"
#include "user_lib.h"
static CANCommInstance *cmd_can_comm; // 双板通信CAN comm
// static CANCommInstance *shoot_can_comm; // 双板通信CAN comm#endif
#endif
#ifdef ONE_BOARD
static Publisher_t *chassis_cmd_pub;   // 底盘控制消息发布者
static Subscriber_t *chassis_feed_sub; // 底盘反馈信息订阅者
#endif                                 // ONE_BOARD

static Chassis_Ctrl_Cmd_s chassis_cmd_send;      // 发送给底盘应用的信息,包括控制信息和UI绘制相关
static Chassis_Upload_Data_s chassis_fetch_data; // 从底盘应用接收的反馈信息信息,底盘功率枪口热量与底盘运动状态等
static Shoot_Ctrl_Cmd_s shoot_cmd_send;      // 发送给底盘应用的信息,包括控制信息和UI绘制相关
static Shoot_Upload_Data_s shoot_fetch_data; // 从底盘应用接收的反馈信息信息,底盘功率枪口热量与底盘运动状态等
extern SuperCapInstance *cap;             // 超级电容
static RC_ctrl_t *rc_data;                // 遥控器数据,初始化时返回
static VT_ctrl_t *vt_data;               // 图传链路下发的键鼠遥控数据 与遥控器数据格式保持一致
static float diff_yaw;
//static Vision_Recv_s *vision_recv_data; // 视觉接收数据指针,初始化时返回
//static Vision_Send_s vision_send_data;  // 视觉发送数据
static RecievePacket_t *vision_recv_data; // 视觉接收数据指针,初始化时返回
static SendPacket_t vision_send_data;     // 视觉发送数据
VT_info_t *referee_VT_data; //图传链路数据

//自瞄相关信息
static Trajectory_Type_t trajectory_cal;
static Aim_Select_Type_t aim_select;
static uint32_t no_find_cnt;      // 未发现目标计数
static uint8_t auto_aim_flag = 0; //辅助瞄准标志位  视野内有目标开启 目标丢失关闭
static uint8_t choose_amor_id = 0;
static float yaw_err_for_test;

static Publisher_t *gimbal_cmd_pub;            // 云台控制消息发布者
static Subscriber_t *gimbal_feed_sub;          // 云台反馈信息订阅者
static Gimbal_Ctrl_Cmd_s gimbal_cmd_send;      // 传递给云台的控制信息
static Gimbal_Upload_Data_s gimbal_fetch_data; // 从云台获取的反馈信息

static Publisher_t *shoot_cmd_pub;           // 发射控制消息发布者
static Subscriber_t *shoot_feed_sub;         // 发射反馈信息订阅者
static Shoot_Ctrl_Cmd_s shoot_cmd_send;      // 传递给发射的控制信息
static Shoot_Upload_Data_s shoot_fetch_data; // 从发射获取的反馈信息

static Robot_Status_e robot_state; // 机器人整体工作状态

static referee_info_t *referee_data; // 用于获取裁判系统的数据
static Power_data power_info;			  // 裁判系统数据

static uint8_t loader_flag = 0;      //拨弹模式选择标志位
float tmp_chassis_velocitx_max, tmp_chassis_velocity_max;
uint8_t remote_flag = 0,VTremote_flag = 0,VT_cotine = 0;
uint8_t press_left_last = 0;
uint8_t press_left = 0;

typedef enum {
    TEST_OFF = 0,
    TEST_SINE,      // 正弦波：测试跟随性能和滞后
    TEST_STEP       // 阶跃（方波）：测试超调和响应速度
} Test_Mode_e;

static Test_Mode_e current_test_mode = TEST_SINE;
static float test_time_tick = 0;       // 时间累加器
static float test_center_yaw = 0;      // 测试开始时的Yaw中心
static float test_center_pitch = 0;    // 测试开始时的Pitch中心

// --- 调参区域 ---
#define TEST_FREQ_HZ        3.0f       // 正弦波频率 1Hz
#define TEST_AMP_YAW        30.0f      // Yaw轴摆动幅度 (度)
#define TEST_AMP_PITCH      10.0f      // Pitch轴摆动幅度 (度)
#define STEP_INTERVAL_MS    400       // 阶跃跳变间隔 (ms)
#define CONTROL_PERIOD_S    0.001f     // 控制周期 5ms (1000Hz)

// uint8_t key_s,key_w,key_a,key_d;
void RobotCMDInit() {
    rc_data = RemoteControlInit(&huart3);   // 修改为对应串口,注意如果是自研板dbus协议串口需选用添加了反相器的那个
    vision_recv_data = VisionInit(); // 视觉通信串口
    referee_VT_data = VTInit(&huart6);
    //referee_data = UITaskInit(&huart6, &ui_data); // 裁判系统初始化,会同时初始化UI
    // Power_data *power_info_ptr = PowerInit(&huart6);
    gimbal_cmd_pub = PubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
    gimbal_feed_sub = SubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    shoot_cmd_pub = PubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
    shoot_feed_sub = SubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));

#ifdef ONE_BOARD // 双板兼容
    chassis_cmd_pub = PubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_feed_sub = SubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
#endif // ONE_BOARD
#ifdef GIMBAL_BOARD
    CANComm_Init_Config_s comm_conf = {
        .can_config = {
            .can_handle = &hcan1,
            .tx_id = 0x312,
            .rx_id = 0x311,
        },
        .recv_data_len = sizeof(Chassis_Upload_Data_s),
        .send_data_len = sizeof(Chassis_Ctrl_Cmd_s),
    };

    cmd_can_comm = CANCommInit(&comm_conf);


#endif // GIMBAL_BOARD
    gimbal_cmd_send.pitch = 0;

    robot_state = ROBOT_STOP;
}

/**
 * @brief 根据gimbal app传回的当前电机角度计算和零位的误差
 *        单圈绝对角度的范围是0~360,说明文档中有图示
 *
 */
// static void CalcOffsetAngle() {
//     // 别名angle提高可读性,不然太长了不好看,虽然基本不会动这个函数
//     static float angle;
//     angle = gimbal_fetch_data.yaw_motor_single_round_angle; // 从云台获取的当前yaw电机单圈角度
//
// #if YAW_ECD_GREATER_THAN_4096                               // 如果大于180度
//     if (angle > YAW_ALIGN_ANGLE && angle <= 180.0f + YAW_ALIGN_ANGLE)
//         chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
//     else if (angle > 180.0f + YAW_ALIGN_ANGLE)
//         chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE - 360.0f;
//         // chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
//         else
//         chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
// #else // 小于180度
//     if (angle > YAW_ALIGN_ANGLE)
//         chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
//     else if (angle <= YAW_ALIGN_ANGLE && angle >= YAW_ALIGN_ANGLE - 180.0f)
//         chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
//     else
//         chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE + 360.0f;
// #endif
// }

static void Gimbal_Test_Signal_Generator(void) {


    test_time_tick += CONTROL_PERIOD_S;

    // 1. 正弦波模式 (Sine Wave)
    if (current_test_mode == TEST_SINE) {
        // Yaw: 以进入模式时的角度为中心进行正弦摆动
        gimbal_cmd_send.yaw = test_center_yaw + TEST_AMP_YAW * sinf(2 * PI * TEST_FREQ_HZ * test_time_tick);

        // Pitch: 同理
        gimbal_cmd_send.pitch = 0;
    }
    // 2. 阶跃模式 (Step Response / Square Wave)
    else if (current_test_mode == TEST_STEP) {
        // 利用取余运算实现方波
        // (int)(time * 1000) % (2000) < 1000
        uint32_t time_ms = (uint32_t)(test_time_tick * 1000);

        if (time_ms % (STEP_INTERVAL_MS * 2) < STEP_INTERVAL_MS) {
            // High Level
            gimbal_cmd_send.yaw = test_center_yaw + TEST_AMP_YAW;
            gimbal_cmd_send.pitch = 0;
        } else {
            // Low Level (回到中心或反向)
            gimbal_cmd_send.yaw = test_center_yaw - TEST_AMP_YAW;
            gimbal_cmd_send.pitch = 0;
        }
    }

    // 强制设置模式，确保云台响应
    //gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;

    // 限制 Pitch 防止撞击机械限位
    if (gimbal_cmd_send.pitch >= PITCH_MAX_ANGLE) gimbal_cmd_send.pitch = PITCH_MAX_ANGLE;
    if (gimbal_cmd_send.pitch <= PITCH_MIN_ANGLE) gimbal_cmd_send.pitch = PITCH_MIN_ANGLE;
}

static void CalcOffsetAngle() {
    // 别名angle提高可读性
    // static float angle;
    // //angle = gimbal_fetch_data.yaw_motor_single_round_angle; // 从云台获取的当前yaw电机单圈角度
    // angle = gimbal_fetch_data.yaw_motor_single_round_angle; // 从云台获取的当前yaw电机单圈角度

    // 计算偏移角度
    float offset = gimbal_fetch_data.yaw_motor_single_round_angle/3.14159*PI - YAW_CHASSIS_ALIGN_RAD;
    //float offset = angle - (1.2474);

    // 将偏移角度归一化到 [-180, 180] 度之间
    if (offset > PI) {
        offset -= 2*PI;
    } else if (offset < -PI) {
        offset += 2*PI;
    }
    // 赋值给输出
    chassis_cmd_send.offset_angle = offset;
    gimbal_cmd_send.offset_angle = offset;

}
//功能：死亡后清除小陀螺的状态
// static void death_check()
// {
//     if(referee_data->GameRobotState.current_HP <= 0 || referee_data->GameRobotState.power_management_chassis_output == 0)
//     {
//         rc_data[TEMP].key_count[KEY_PRESS][Key_E] = 0;
//         gimbal_cmd_send.yaw = gimbal_fetch_data.gimbal_imu_data.YawTotalAngle;
//     }
// }
static void death_check()
{
    if(chassis_fetch_data.dead_state)
    {
        referee_VT_data->vt_ctrl[TEMP].key[KEY_PRESS].shift = 0;
        gimbal_cmd_send.yaw = gimbal_fetch_data.gimbal_imu_data.YawTotalAngle;
    }
}
//功能：等级提升弹频提高
static void shoot_rate_improve()
{
    if(chassis_fetch_data.robot_level <= 5)
    {
        if (vision_recv_data->mode == 0)
            shoot_cmd_send.shoot_rate = 15;
        else
            shoot_cmd_send.shoot_rate = 15;
    }
    if(chassis_fetch_data.robot_level >= 5)
    {
        if (vision_recv_data->mode == 0)
            shoot_cmd_send.shoot_rate = 15;
        else
            shoot_cmd_send.shoot_rate = 15;
    }
}

// static void update_ui_data() {
//
// }
// static void auto_aim_mode() {
//     // 弹速设置
//     // trajectory_cal.v0 = 20; // 弹速30
//
//     // 目标检测
//     if (vision_recv_data->yaw == 0 && vision_recv_data->pitch == 0) {
//         aim_select.suggest_fire = 0;
//         chassis_cmd_send.suggest_fire = 0;
//
//         // 未发现目标
//         no_find_cnt++;
//
//         if (no_find_cnt >= 2000) {
//             // gimbal_scan_flag = 1;
//             // auto_aim_flag = 0;
//             no_find_cnt = 0; // 重置计数器
//         }
//     } else {
//         // 弹道解算
//         float single_angle_yaw_now = gimbal_fetch_data.gimbal_imu_data.Yaw;
//         diff_yaw = vision_recv_data->yaw - single_angle_yaw_now;
//         float yaw_err = diff_yaw;
//         yaw_err_for_test = yaw_err;
//
//         if (diff_yaw > 180)
//             diff_yaw -= 360;
//         else if (diff_yaw < -180)
//             diff_yaw += 360;
//
//         // 调整云台角度
//         gimbal_cmd_send.yaw = gimbal_fetch_data.gimbal_imu_data.YawTotalAngle + diff_yaw;
//         gimbal_cmd_send.pitch = vision_recv_data->pitch;
//
//         // 开火决策
//         if (vision_recv_data->distance != -1 && vision_recv_data->yaw != 0 && vision_recv_data->pitch != 0) {
//             if (vision_recv_data->mode == 1) {
//                 aim_select.suggest_fire = 1;
//                 chassis_cmd_send.suggest_fire = 1;
//             } else {
//                 aim_select.suggest_fire = 0;
//                 chassis_cmd_send.suggest_fire = 0;
//             }
//         }
//     }
// }
static void auto_aim_mode() {
    // 如果按下按键 B，云台控制由鼠标输入决定
    if (rc_data[TEMP].key_count[KEY_PRESS][Key_B] % 2 == 1 ) {
        // 鼠标控制云台
        gimbal_cmd_send.yaw -= (float) rc_data[TEMP].mouse.x / 660 * 4; // 系数待测
        gimbal_cmd_send.pitch -= (float) rc_data[TEMP].mouse.y / 660 * 6;

        // 限制云台俯仰角度
        if (gimbal_cmd_send.pitch >= PITCH_MAX_ANGLE) gimbal_cmd_send.pitch = PITCH_MAX_ANGLE;
        if (gimbal_cmd_send.pitch <= PITCH_MIN_ANGLE) gimbal_cmd_send.pitch = PITCH_MIN_ANGLE;
        //chassis_cmd_send.heat_mode = HEAT_CLOSE;

    } else {
        // 视觉控制云台
        if (vision_recv_data->yaw == 0 && vision_recv_data->pitch == 0) {
            aim_select.suggest_fire = 0;
            chassis_cmd_send.auto_aim_fire_suggestion = 0;

            // 未发现目标
            no_find_cnt++;

            if (no_find_cnt >= 2000) {
                no_find_cnt = 0; // 重置计数器
            }
        } else {
            // 弹道解算
            float single_angle_yaw_now = gimbal_fetch_data.gimbal_imu_data.Yaw;
            diff_yaw = vision_recv_data->yaw - single_angle_yaw_now;
            //diff_yaw = single_angle_yaw_now - vision_recv_data->yaw;
            float yaw_err = diff_yaw;
            yaw_err_for_test = yaw_err;

            if (diff_yaw > 180)
                diff_yaw -= 360;
            else if (diff_yaw < -180)
                diff_yaw += 360;

            // 调整云台角度
            gimbal_cmd_send.yaw = gimbal_fetch_data.gimbal_imu_data.YawTotalAngle + diff_yaw;
            gimbal_cmd_send.pitch = vision_recv_data->pitch ;
            //chassis_cmd_send.heat_mode = HEAT_OPEN;


        }
    }

    // 开火决策（在鼠标控制和视觉控制下都生效）
    if (vision_recv_data->distance != -1 && vision_recv_data->yaw != 0 && vision_recv_data->pitch != 0) {
        if (vision_recv_data->mode == 1) {
            aim_select.suggest_fire = 1;
            chassis_cmd_send.auto_aim_fire_suggestion = 1;
        } else {
            aim_select.suggest_fire = 0;
            chassis_cmd_send.auto_aim_fire_suggestion = 0;
        }
    } else {
        aim_select.suggest_fire = 0;
        chassis_cmd_send.auto_aim_fire_suggestion = 0;
    }
}
// static void auto_aim_mode() {
//     //  trajectory_cal.v0 = 20; //弹速30
//     // if (vision_recv_data->x == 0 && vision_recv_data->y == 0 && vision_recv_data->z == 0
//     //     && vision_recv_data->vx == 0 && vision_recv_data->vy == 0 && vision_recv_data->vz == 0) {
//     //     aim_select.suggest_fire = 0;
//     //     chassis_cmd_send.suggest_fire = 0;
//     //
//     //     //未发现目标
//     //     no_find_cnt++;
//     //
//     //     if (no_find_cnt >= 2000) {
//     //         //gimbal_scan_flag = 1;
//     //         //auto_aim_flag = 0;
//     //     }
//     //     //else
//     //     //auto_aim_flag = 1;
//     // } else {
//     //     //弹道解算
//     //
//         float single_angle_yaw_now = gimbal_fetch_data.gimbal_imu_data.Yaw;
//         float diff_yaw = vision_recv_data->yaw - single_angle_yaw_now;
//         float yaw_err = diff_yaw;
//         yaw_err_for_test = yaw_err;
//
//         if(diff_yaw>180)
//             diff_yaw -= 360;
//         else if(diff_yaw<-180)
//             diff_yaw += 360;
//
//         // gimbal_cmd_send.yaw = gimbal_fetch_data.gimbal_imu_data.YawTotalAngle + diff_yaw;
//         // gimbal_cmd_send.pitch = vision_recv_data->pitch;
//
//     if(vision_recv_data->distance!=-1&&vision_recv_data->yaw!= 0&&vision_recv_data->pitch!=0) {
//         // gimbal_cmd_send.yaw = vision_recv_data->yaw;
//         gimbal_cmd_send.yaw = gimbal_fetch_data.gimbal_imu_data.YawTotalAngle + diff_yaw;
//         gimbal_cmd_send.pitch = vision_recv_data->pitch;
//         if(vision_recv_data->mode == 1)
//         {
//             aim_select.suggest_fire = 1;
//             chassis_cmd_send.suggest_fire = 1;
//         }
//         else
//         {
//             aim_select.suggest_fire = 0;
//             chassis_cmd_send.suggest_fire = 0;
//         }
//     }
// }

/**
 * @brief 控制输入为遥控器(调试时)的模式和控制量设置
 *
 */
static void RemoteControlSet() {
    // 控制底盘和云台运行模式,云台待添加,云台是否始终使用IMU数据?
    if (switch_is_down(rc_data[TEMP].rc.switch_right)) // 右侧开关状态[下],小陀螺
    {
        chassis_cmd_send.chassis_mode = CHASSIS_ROTATE;
        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;

    } else if (switch_is_mid(rc_data[TEMP].rc.switch_right)) // 右侧开关状态[中],,底盘跟随云台
    {
        // chassis_cmd_send.chassis_mode = CHASSIS_NO_FOLLOW;
        chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
        shoot_cmd_send.lid_mode = LID_CLOSE;
    } else if (switch_is_up(rc_data[TEMP].rc.switch_right))// 右侧开关状态[上]
    {
        chassis_cmd_send.chassis_mode = CHASSIS_NO_FOLLOW; //TODO 待测试
        shoot_cmd_send.lid_mode = LID_OPEN;
    }
    // 云台参数,确定云台控制数据
    if (switch_is_mid(rc_data[TEMP].rc.switch_left) ||
        (vision_recv_data->distance==-1)) // 左侧开关状态为[中],或视觉未识别到目标,纯遥控器拨杆控制
    {
        gimbal_cmd_send.auto_aim_mode = 0;
        // 待添加,视觉会发来和目标的误差,同样将其转化为total angle的增量进行控制
        // ...
        aim_select.suggest_fire = 0;
        chassis_cmd_send.auto_aim_fire_suggestion = 0;
        //gimbal_cmd_send.yaw -= 0.0025f * (float) rc_data[TEMP].rc.rocker_l_;
        gimbal_cmd_send.yaw -= 0.0005f * (float) rc_data[TEMP].rc.rocker_l_;
        //gimbal_cmd_send.pitch += 0.001f * (float) rc_data[TEMP].rc.rocker_l1;
        gimbal_cmd_send.pitch += 0.0005f * (float) rc_data[TEMP].rc.rocker_l1;

        if (gimbal_cmd_send.pitch >= PITCH_MAX_ANGLE) gimbal_cmd_send.pitch = PITCH_MAX_ANGLE;
        if (gimbal_cmd_send.pitch <= PITCH_MIN_ANGLE) gimbal_cmd_send.pitch = PITCH_MIN_ANGLE;
        //Gimbal_Test_Signal_Generator();

    }
    // 左侧开关状态为[下],视觉模式

    if (switch_is_down(rc_data[TEMP].rc.switch_left)) {
        auto_aim_mode();
        gimbal_cmd_send.auto_aim_mode = 1;
    }
    // 云台软件限位

    // 底盘参数,目前没有加入小陀螺(调试似乎暂时没有必要),系数需要调整，遥控器输入灵敏度
    chassis_cmd_send.vx = 5.0f * ((float) rc_data[TEMP].rc.rocker_r1 / 660); // _水平方向
    chassis_cmd_send.vy = 5.0f * ((float) rc_data[TEMP].rc.rocker_r_ / 660); // 1数值方向
    // chassis_cmd_send.vx = 10.0f * (float) rc_data[TEMP].rc.rocker_r_; // _水平方向
    // chassis_cmd_send.vy = 10.0f * (float) rc_data[TEMP].rc.rocker_r1; // 1数值方向
    if (switch_is_up(rc_data[TEMP].rc.switch_right)) // 右侧开关状态[上],弹舱打开
        ;                                            // 弹舱舵机控制,待添加servo_motor模块,开启
    else; // 弹舱舵机控制,待添加servo_motor模块,关闭


    // 摩擦轮控制,拨轮向上打为负,向下为正
    if (rc_data[TEMP].rc.dial < -100) // 向上超过100,打开摩擦轮
    {
        shoot_cmd_send.friction_mode = FRICTION_ON;
        chassis_cmd_send.friction_mode = FRICTION_ON;
    }
    else
    {
        shoot_cmd_send.friction_mode = FRICTION_OFF;
        chassis_cmd_send.friction_mode = FRICTION_OFF;
    }


    // 拨弹控制,遥控器固定为一种拨弹模式,可自行选择
    if (rc_data[TEMP].rc.dial < -500){//
        if(switch_is_down(rc_data[TEMP].rc.switch_left)&&vision_recv_data->mode==0 &&aim_select.suggest_fire == 0){
            shoot_cmd_send.shoot_mode = SHOOT_OFF;
            shoot_cmd_send.loader_mode = LOAD_STOP;
        }
        else {
            shoot_cmd_send.shoot_mode = SHOOT_ON;
            shoot_cmd_send.loader_mode = LOAD_BURSTFIRE;
            // shoot_cmd_send.loader_mode = LOAD_1_BULLET;
        }
    }
    else
    {
        shoot_cmd_send.loader_mode = LOAD_STOP;
        shoot_cmd_send.shoot_mode = SHOOT_OFF;
    }

    // 视觉控制发射
    //    if (aim_select.suggest_fire == 1) {
    //        //shoot_cmd_send.friction_mode = FRICTION_ON;
    //        //shoot_cmd_send.shoot_mode = SHOOT_ON;
    //        shoot_cmd_send.load_mode = LOAD_BURSTFIRE;
    //    } else {
    //        //shoot_cmd_send.friction_mode = FRICTION_OFF;
    //        shoot_cmd_send.load_mode = LOAD_STOP;
    //    }

    // 射频控制,固定每秒1发,后续可以根据左侧拨轮的值大小切换射频,
    if (rc_data[TEMP].rc.dial < -500){//
        if(switch_is_down(rc_data[TEMP].rc.switch_left)&&vision_recv_data->mode==0){
            shoot_cmd_send.shoot_rate = 35;
        }
        else {
            shoot_cmd_send.shoot_rate = 35;
        }
    }
    else
    {
        shoot_cmd_send.shoot_rate = 35;
    }
    // shoot_cmd_send.shoot_rate = 25;

    if (shoot_fetch_data.stalled_flag == 1)
        shoot_cmd_send.loader_mode = LOAD_REVERSE;
    if(chassis_fetch_data.dead_state)
        gimbal_cmd_send.yaw = gimbal_fetch_data.gimbal_imu_data.YawTotalAngle;

}
/**
* @brief 控制输入为图传遥控器(调试时)的模式和控制量设置
*
*/
static void VTRemoteControlSet() {

    if(referee_VT_data->vt_ctrl[TEMP].vt.fn_right == 0)
    {
        chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
        // chassis_cmd_send.chassis_mode = CHASSIS_NO_FOLLOW;

        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
    }
    if(referee_VT_data->vt_ctrl[TEMP].vt.fn_right == 1)
    {
        chassis_cmd_send.chassis_mode = CHASSIS_ROTATE;
        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
    }

    // 云台参数,确定云台控制数据
    if (referee_VT_data->vt_ctrl[TEMP].vt.fn_left == 0 ||
        (vision_recv_data->distance==-1)) // 左侧开关状态为[中],或视觉未识别到目标,纯遥控器拨杆控制
    {
        aim_select.suggest_fire = 0;
        gimbal_cmd_send.yaw -= 0.0025f * (float)referee_VT_data->vt_ctrl[TEMP].vt.ch_left_x;
        gimbal_cmd_send.pitch += 0.001f * (float)referee_VT_data->vt_ctrl[TEMP].vt.ch_left_y;

        if (gimbal_cmd_send.pitch >= PITCH_MAX_ANGLE) gimbal_cmd_send.pitch = PITCH_MAX_ANGLE;
        if (gimbal_cmd_send.pitch <= PITCH_MIN_ANGLE) gimbal_cmd_send.pitch = PITCH_MIN_ANGLE;
    }

    // 待添加,视觉会发来和目标的误差,同样将其转化为total angle的增量进行控制
    // ...
    aim_select.suggest_fire = 0;
    float VTright_,VTright1;
    VTright_ = (float)referee_VT_data->vt_ctrl[TEMP].vt.ch_right_x / 660.0f;
    VTright1 = (float)referee_VT_data->vt_ctrl[TEMP].vt.ch_right_y / 660.0f;

    if (abs(VTright_) < 0.03f) VTright_ = 0;
    if (abs(VTright1) < 0.03f) VTright1 = 0;

    chassis_cmd_send.vx = 5.0f * VTright1;
    chassis_cmd_send.vy = 5.0f * VTright_;
    //    if (switch_is_down(rc_data[TEMP].rc.switch_left))
    //    {
    //        gimbal_cmd_send.gimbal_mode = GIMBAL_STABILIZE_MODE;
    //    }
    if(referee_VT_data->vt_ctrl[TEMP].vt.fn_left == 1)
    {
        auto_aim_mode();
        gimbal_cmd_send.auto_aim_mode = 1;
    }
    // 射频控制,固定每秒1发,后续可以根据左侧拨轮的值大小切换射频,
    shoot_cmd_send.shoot_rate = 15;

    // 摩擦轮控制
    if (referee_VT_data->vt_ctrl[TEMP].vt.wheel > 1300)// 向上超过100,打开摩擦轮
    {
        shoot_cmd_send.friction_mode = FRICTION_ON;
    }
    else
    {
        shoot_cmd_send.friction_mode = FRICTION_OFF;
    }

    // 拨弹控制,遥控器固定为一种拨弹模式,可自行选择
    if (referee_VT_data->vt_ctrl[TEMP].vt.wheel > 1500)
    {
        if(referee_VT_data->vt_ctrl[TEMP].vt.mode_sw == 0 &&vision_recv_data->mode==0 &&aim_select.suggest_fire == 0){
            shoot_cmd_send.shoot_mode = SHOOT_OFF;
            shoot_cmd_send.loader_mode = LOAD_STOP;
        }
        else {
            shoot_cmd_send.shoot_mode = SHOOT_ON;
            shoot_cmd_send.loader_mode = LOAD_BURSTFIRE;
        }
    }
    else
    {
        shoot_cmd_send.loader_mode = LOAD_STOP;
        shoot_cmd_send.shoot_mode = SHOOT_OFF;
    }

    if (referee_VT_data->vt_ctrl[TEMP].vt.wheel > 1500){
        if(referee_VT_data->vt_ctrl[TEMP].vt.mode_sw == 0 && vision_recv_data->mode == 0){
            shoot_cmd_send.aim_flag = 0;
        }
        else {
            shoot_cmd_send.aim_flag = 1;
        }
    }
    else
    {
        shoot_cmd_send.aim_flag = 0;
    }
    // shoot_cmd_send.shoot_rate = 25;

    if (shoot_fetch_data.stalled_flag == 1)
        shoot_cmd_send.loader_mode = LOAD_REVERSE;
}

/**
 * @brief 输入为(DT7链路)键鼠时模式和控制量设置
 *
 */
static void MouseKeySet() {

    uint8_t key_s,key_w,key_a,key_d;
    if(abs(rc_data[TEMP].key[KEY_PRESS].w))
    {
        key_w = 1;
    }
    else
    {
        key_w = 0;
    }
    if(abs(rc_data[TEMP].key[KEY_PRESS].s))
    {
        key_s = 1;
    }
    else
    {
        key_s = 0;
    }
    if(abs(rc_data[TEMP].key[KEY_PRESS].a))
    {
        key_a = 1;
    }
    else
    {
        key_a = 0;
    }
    if(abs(rc_data[TEMP].key[KEY_PRESS].d))
    {
        key_d = 1;
    }
    else
    {
        key_d = 0;
    }
    chassis_cmd_send.vx = (float)key_w * 5.0f - (float)key_s * 5.0f ; // 系数待测
    chassis_cmd_send.vy = (float)key_a * 5.0f - (float)key_d * 5.0f;

    gimbal_cmd_send.yaw -= (float) rc_data[TEMP].mouse.x / 660 * 4; // 系数待测
    gimbal_cmd_send.pitch += (float) rc_data[TEMP].mouse.y / 660 * 6;

    if (gimbal_cmd_send.pitch >= PITCH_MAX_ANGLE) gimbal_cmd_send.pitch = PITCH_MAX_ANGLE;
    if (gimbal_cmd_send.pitch <= PITCH_MIN_ANGLE) gimbal_cmd_send.pitch = PITCH_MIN_ANGLE;

    aim_select.suggest_fire = 0;
    chassis_cmd_send.auto_aim_fire_suggestion = 0;
    press_left = rc_data[TEMP].mouse.press_l;
    if (rc_data[TEMP].mouse.press_l && (!rc_data[TEMP].mouse.press_r)) // 左键发射模式
    {
        if (shoot_cmd_send.friction_mode == FRICTION_ON) {
            shoot_cmd_send.shoot_mode = SHOOT_ON;
            if(loader_flag == 0 && press_left_last == press_left){
                shoot_cmd_send.loader_mode = LOAD_BURSTFIRE;
            }else if(loader_flag == 1 && press_left_last != press_left){
                shoot_cmd_send.loader_mode = LOAD_1_BULLET;
            }
        }
    } else if ((!rc_data[TEMP].mouse.press_l) && (!rc_data[TEMP].mouse.press_r))
    {
        shoot_cmd_send.loader_mode = LOAD_STOP;
    }
    press_left_last = press_left ; // 更新状态
    // if (rc_data[TEMP].mouse.press_r) // 右键自瞄模式
    // {
    //     auto_aim_mode();
    //         // if (aim_select.suggest_fire == 1 && rc_data[TEMP].mouse.press_l &&
    //         //     shoot_cmd_send.friction_mode == FRICTION_ON) {
    //             if (aim_select.suggest_fire == 1&&vision_recv_data->mode == 1 && rc_data[TEMP].mouse.press_l &&shoot_cmd_send.friction_mode == FRICTION_ON) {
    //                 // aim_select.suggest_fire = 1;
    //                 // chassis_cmd_send.suggest_fire = 1;
    //             shoot_cmd_send.shoot_mode = SHOOT_ON;
    //             shoot_cmd_send.loader_mode = LOAD_BURSTFIRE;
    //         } else {
    //             shoot_cmd_send.loader_mode = LOAD_STOP;
    //                 // aim_select.suggest_fire = 0;
    //                 // chassis_cmd_send.suggest_fire = 0;
    //         }
    // }
    if (rc_data[TEMP].mouse.press_r) {
        auto_aim_mode();

        // switch(rc_data[TEMP].key_count[KEY_PRESS][Key_B] % 2)
        // {
        //     case 0:
        //     // 调整云台角度
        //     gimbal_cmd_send.yaw = gimbal_fetch_data.gimbal_imu_data.YawTotalAngle + diff_yaw;
        //     gimbal_cmd_send.pitch = vision_recv_data->pitch;
        //     break;
        // default:
        //     gimbal_cmd_send.yaw -= (float) rc_data[TEMP].mouse.x / 660 * 4; // 系数待测
        //     gimbal_cmd_send.pitch -= (float) rc_data[TEMP].mouse.y / 660 * 6;
        //
        //     if (gimbal_cmd_send.pitch >= PITCH_MAX_ANGLE) gimbal_cmd_send.pitch = PITCH_MAX_ANGLE;
        //     if (gimbal_cmd_send.pitch <= PITCH_MIN_ANGLE) gimbal_cmd_send.pitch = PITCH_MIN_ANGLE;
        //     break;
        // }// 开火控制
        if (aim_select.suggest_fire == 1 && rc_data[TEMP].mouse.press_l && shoot_cmd_send.friction_mode == FRICTION_ON) {
            shoot_cmd_send.aim_flag = 1;
            shoot_cmd_send.shoot_mode = SHOOT_ON;
            shoot_cmd_send.loader_mode = LOAD_BURSTFIRE;
        } else {
            shoot_cmd_send.aim_flag = 0;
            shoot_cmd_send.loader_mode = LOAD_STOP;
        }
    }
    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_E] % 2) // R键手动换弹
    {
    case 0:
        loader_flag = 0;
        break;
        default:
            loader_flag = 1;
            break;
    }
    // switch (rc_data[TEMP].key_count[KEY_PRESS][Key_R] % 2) // R键手动换弹
    // {
    // case 0:
    //     // MyUIInit();
    //     chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
    //     gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
    //     break;
    // default:
    //     chassis_cmd_send.chassis_mode = CHASSIS_NO_FOLLOW;
    //     chassis_cmd_send.offset_angle = 0;
    //     gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
    //     break;
    // }

    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_F] % 2) // F键开关摩擦轮
    {
        case 0:
            shoot_cmd_send.shoot_mode = SHOOT_OFF;
            shoot_cmd_send.friction_mode = FRICTION_OFF;
            chassis_cmd_send.friction_mode = FRICTION_OFF;
            break;
        default:
            shoot_cmd_send.shoot_mode = SHOOT_ON;
            shoot_cmd_send.friction_mode = FRICTION_ON;
            chassis_cmd_send.friction_mode = FRICTION_ON;
        break;
    }
    // switch (rc_data[TEMP].key_count[KEY_PRESS][Key_B] % 2) // B键开关弹舱盖
    // {
    //     case 0:
    //         shoot_cmd_send.lid_mode = LID_CLOSE;
    //         break;
    //     default:
    //         shoot_cmd_send.lid_mode = LID_OPEN;
    //         break;
    // }
    // static uint8_t e_key_prev_state = 0; // 用于记录 E 键上一次的状态

//     switch (rc_data[TEMP].key_count[KEY_PRESS][Key_R] % 2) // R键手动刷新UI
//     {
//     case 1:
//         chassis_cmd_send.ui_updata = 1;
//         rc_data[TEMP].key_count[KEY_PRESS][Key_R]++;
//     break;
//     default:
//         chassis_cmd_send.ui_updata = 0;
//     break;
// }
    // switch (rc_data[TEMP].key_count[KEY_PRESS][Key_E] % 2) // E键开关小陀螺
    // {
    // case 0:
    //     if (rc_data[TEMP].key[KEY_PRESS].e) { // 如果 E 键被按下
    //         if (!e_key_prev_state) { // 如果之前 E 键没有被按下
    //             // 进入小陀螺模式
    //             chassis_cmd_send.chassis_mode = CHASSIS_ROTATE;
    //             gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
    //         }
    //         e_key_prev_state = 1; // 更新 E 键状态为按下
    //     } else { // 如果 E 键没有被按下
    //         if (e_key_prev_state) { // 如果之前 E 键被按下
    //             // 恢复原来的模式
    //             chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
    //             gimbal_cmd_send.gimbal_mode = GIMBAL_FREE_MODE;
    //         }
    //         e_key_prev_state = 0; // 更新 E 键状态为松开
    //     }
    //     break;
    // default:
    //     // 保持与 case 0 相同的逻辑
    //     if (rc_data[TEMP].key[KEY_PRESS].e) { // 如果 E 键被按下
    //         if (!e_key_prev_state) { // 如果之前 E 键没有被按下
    //             // 进入小陀螺模式
    //             chassis_cmd_send.chassis_mode = CHASSIS_ROTATE;
    //             gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
    //         }
    //         e_key_prev_state = 1; // 更新 E 键状态为按下
    //     } else { // 如果 E 键没有被按下
    //         if (e_key_prev_state) { // 如果之前 E 键被按下
    //             // 恢复原来的模式
    //             chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
    //             gimbal_cmd_send.gimbal_mode = GIMBAL_FREE_MODE;
    //         }
    //         e_key_prev_state = 0; // 更新 E 键状态为松开
    //     }
    //     break;
    // }
    switch (rc_data[TEMP].key[KEY_PRESS].shift) //按住shift小陀螺
    {
    case 0:
        chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
        gimbal_cmd_send.gimbal_mode = GIMBAL_FREE_MODE;
        break;
    case 1:
        chassis_cmd_send.chassis_mode = CHASSIS_ROTATE;
        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
        break;
    }
        // switch (rc_data[TEMP].key_count[KEY_PRESS][Key_E] % 2) // E键开关小陀螺
    // {
    //     case 0:
    //         chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
    //         gimbal_cmd_send.gimbal_mode = GIMBAL_FREE_MODE;
    //         break;
    //     default:
    //         chassis_cmd_send.chassis_mode = CHASSIS_ROTATE;
    //         gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
    //         break;
    // }
    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_Q] % 2) // Q键开关热控
    {
        case 0:
            shoot_cmd_send.heat_mode = HEAT_OPEN;
            // chassis_cmd_send.heat_mode = HEAT_OPEN;
            break;
        default:
            shoot_cmd_send.heat_mode = HEAT_CLOSE;
            // chassis_cmd_send.heat_mode = HEAT_CLOSE;
        break;
    }
//    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_G] % 3) // G键切换发射模式
//    {
//        case 0:
//            loader_flag = 0;
//            break;
//        case 1:
//            loader_flag = 1;
//            break;
//        default:
//            loader_flag = 3;
//            break;
//    }
    switch (rc_data[TEMP].key[KEY_PRESS].v) //按V允许消耗超级电容能量
    {
        case 0:
            chassis_cmd_send.cap_open = 0;
            break;
        case 1:
            chassis_cmd_send.cap_open = 1;
            // chassis_cmd_send.vx = rc_data[TEMP].key[KEY_PRESS].a * 15000 - rc_data[TEMP].key[KEY_PRESS].d * 15000;
            break;
    }
    switch (rc_data[TEMP].key[KEY_PRESS].c) //按C允许消耗超级电容能量
    {
    case 0:
        chassis_cmd_send.cap_open = 0;
        break;
    case 1:
        chassis_cmd_send.cap_open = 1;
        // chassis_cmd_send.vx = rc_data[TEMP].key[KEY_PRESS].a * 15000 - rc_data[TEMP].key[KEY_PRESS].d * 15000;
        break;
    }
    shoot_rate_improve();

    if(shoot_fetch_data.stalled_flag ==1)
        shoot_cmd_send.loader_mode = LOAD_REVERSE;

    death_check();
}

/**
 * @brief 输入为（图传链路）键鼠时模式和控制量设置
 *
 */
static void VTMouseKeySet()
{
    //粗调

    uint8_t key_s, key_w, key_a, key_d;
    if (abs(referee_VT_data->vt_ctrl[TEMP].key[KEY_PRESS].a))
    {
        key_a = 1;
    }
    else
    {
        key_a = 0;
    }
    if (abs(referee_VT_data->vt_ctrl[TEMP].key[KEY_PRESS].d))
    {
        key_d = 1;
    }
    else
    {
        key_d = 0;
    }
    if (abs(referee_VT_data->vt_ctrl[TEMP].key[KEY_PRESS].w))
    {
        key_w = 1;
    }
    else
    {
        key_w = 0;
    }
    if (abs(referee_VT_data->vt_ctrl[TEMP].key[KEY_PRESS].s))
    {
        key_s = 1;
    }
    else
    {
        key_s = 0;
    }
    //chassis_cmd_send.vx = (float)
    chassis_cmd_send.vx = (float) key_w * 3.0f - (float) key_s * 3.0f; // 系数待测
    chassis_cmd_send.vy = (float) key_d * 3.0f - (float) key_a * 3.0f;

    gimbal_cmd_send.yaw -= (float) referee_VT_data->vt_ctrl[TEMP].mouse.x / 660 * 4; // 系数待测
    gimbal_cmd_send.pitch += (float) referee_VT_data->vt_ctrl[TEMP].mouse.y / 660 * 6;
    if (gimbal_cmd_send.pitch >= PITCH_MAX_ANGLE)
        gimbal_cmd_send.pitch = PITCH_MAX_ANGLE;
    if (gimbal_cmd_send.pitch <= PITCH_MIN_ANGLE)
        gimbal_cmd_send.pitch = PITCH_MIN_ANGLE;
    // gimbal_cmd_send.pid_flag = 0;// 云台pitch的pid切换标志
    // 待添加,视觉会发来和目标的误差,同样将其转化为total angle的增量进行控制
    // ...
    aim_select.suggest_fire = 0;
    chassis_cmd_send.auto_aim_mode = 0;
    chassis_cmd_send.auto_aim_fire_suggestion = 0;
    // if (referee_VT_data->vt_ctrl[TEMP].mouse.left && (!referee_VT_data->vt_ctrl[TEMP].mouse.right)) // 左键发射模式
    // {
    //     if (shoot_cmd_send.friction_mode == FRICTION_ON)
    //     {
    //         shoot_cmd_send.shoot_mode = SHOOT_ON;
    //         if(loader_flag == 0){
    //             shoot_cmd_send.loader_mode = LOAD_BURSTFIRE;
    //         }else if(loader_flag == 1){
    //             shoot_cmd_send.loader_mode = LOAD_1_BULLET;
    //         }
    //     }
    // }
    // else if ((!referee_VT_data->vt_ctrl[TEMP].mouse.left) && (!referee_VT_data->vt_ctrl[TEMP].mouse.right))
    // {
    //     shoot_cmd_send.loader_mode = LOAD_STOP;
    // }
    press_left = referee_VT_data->vt_ctrl[TEMP].mouse.left;
    // if (referee_VT_data->vt_ctrl[TEMP].mouse.left && (!referee_VT_data->vt_ctrl[TEMP].mouse.right)) // 左键发射模式
    if (referee_VT_data->vt_ctrl[TEMP].mouse.left  && (!referee_VT_data->vt_ctrl[TEMP].mouse.right)) // 左键发射模式
    {
        if (shoot_cmd_send.friction_mode == FRICTION_ON)
        {
            shoot_cmd_send.shoot_mode = SHOOT_ON;
            if(loader_flag == 0){
                shoot_cmd_send.loader_mode = LOAD_BURSTFIRE;
                chassis_cmd_send.is_running_fire = 1;
            } else if(loader_flag == 1){
                shoot_cmd_send.loader_mode = LOAD_1_BULLET;
                chassis_cmd_send.is_running_fire = 0;
            }
            // if(loader_flag == 0 && press_left_last == press_left ){
            //     shoot_cmd_send.loader_mode = LOAD_BURSTFIRE;
            //     chassis_cmd_send.loader_mode = LOAD_BURSTFIRE;
            //     // press_left_last = press_left ; // 更新状态
            // }else if(loader_flag == 1 && press_left_last != press_left){ // 刚按下瞬间
            //     shoot_cmd_send.loader_mode = LOAD_1_BULLET;
            //     chassis_cmd_send.loader_mode = LOAD_1_BULLET;
            // }else if(loader_flag == 1 && press_left_last == press_left){ // 单发模式下按住不放
            //     // 必须加上这一段，否则单发会变成慢连发
            //     shoot_cmd_send.loader_mode = LOAD_STOP;
            //     chassis_cmd_send.loader_mode = LOAD_STOP;
            // }
        }
    }
    // else if ((referee_VT_data->vt_ctrl[TEMP].mouse.left) && prev_vt_left && (!referee_VT_data->vt_ctrl[TEMP].mouse.right))
    // {
    //     if (shoot_cmd_send.friction_mode == FRICTION_ON)
    //     {
    //         // shoot_cmd_send.shoot_mode = SHOOT_ON;
    //         // if(loader_flag == 0){
    //         shoot_cmd_send.loader_mode = LOAD_BURSTFIRE;
    //     }
    // }
    else if ((!referee_VT_data->vt_ctrl[TEMP].mouse.left) && (!referee_VT_data->vt_ctrl[TEMP].mouse.right))
    {
        shoot_cmd_send.loader_mode = LOAD_STOP;
    }
    press_left_last = press_left ; // 更新状态
    if (referee_VT_data->vt_ctrl[TEMP].mouse.right) // 右键自瞄模式
    {
        chassis_cmd_send.auto_aim_mode = 1;

        auto_aim_mode();
        if (aim_select.suggest_fire == 1 && referee_VT_data->vt_ctrl[TEMP].mouse.left && shoot_cmd_send.friction_mode == FRICTION_ON)
        {
            shoot_cmd_send.aim_flag = 1;
            chassis_cmd_send.auto_aim_fire_suggestion = 1;
            shoot_cmd_send.shoot_mode = SHOOT_ON;
            shoot_cmd_send.loader_mode = LOAD_BURSTFIRE;
        } else
        {
            shoot_cmd_send.aim_flag = 0;
            chassis_cmd_send.auto_aim_fire_suggestion = 0;
            shoot_cmd_send.loader_mode = LOAD_STOP;
        }
    }
    switch (referee_VT_data->vt_ctrl[TEMP].key_count[KEY_PRESS][Key_B] % 2) // B键换单发模式
    {
    case 0:
        loader_flag = 0;
        chassis_cmd_send.is_running_fire = 1;
        break;
    default:
        loader_flag = 1;
        chassis_cmd_send.is_running_fire = 0;
        break;
    }
    switch (referee_VT_data->vt_ctrl[TEMP].key_count[KEY_PRESS][Key_G] % 2) // G键开关摩擦轮
    {
    case 0:
        shoot_cmd_send.shoot_mode = SHOOT_OFF;
        shoot_cmd_send.friction_mode = FRICTION_OFF;
        chassis_cmd_send.friction_mode = FRICTION_OFF;
        break;
    default:
        shoot_cmd_send.shoot_mode = SHOOT_ON;
        shoot_cmd_send.friction_mode = FRICTION_ON;
        chassis_cmd_send.friction_mode = FRICTION_ON;
        break;
    }
    switch (referee_VT_data->vt_ctrl[TEMP].key_count[KEY_PRESS][Key_Q] % 2) // Q键开关热控
    {
    case 0:
       shoot_cmd_send.heat_mode = HEAT_OPEN;
       //chassis_cmd_send.heat_mode = HEAT_OPEN;
        break;
    default:
        shoot_cmd_send.heat_mode = HEAT_CLOSE;
        //chassis_cmd_send.heat_mode = HEAT_CLOSE;
        break;
    }
    switch (referee_VT_data->vt_ctrl[TEMP].key_count[KEY_PRESS][Key_R] % 2) // R键手动刷新UI
    {
    case 1:
        chassis_cmd_send.ui_refresh = 1;
        referee_VT_data->vt_ctrl[TEMP].key_count[KEY_PRESS][Key_R]++;
        break;
    default:
        chassis_cmd_send.ui_refresh = 0;
        break;
    }
    switch (referee_VT_data->vt_ctrl[TEMP].key[KEY_PRESS].shift) //按住shift小陀螺
    {
    case 0:
        chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
        break;
    case 1:
        chassis_cmd_send.chassis_mode = CHASSIS_ROTATE;
        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
        break;

    default:
        chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
        break;
    }
    static uint8_t x_key_prev_state = 0;
    uint8_t x_key_current = rc_data[TEMP].key[KEY_PRESS].x;
    if (x_key_current && !x_key_prev_state) {
        gimbal_cmd_send.yaw += 180.0f; // 根据方向调整正负号
    }
    x_key_prev_state = x_key_current;
    switch (referee_VT_data->vt_ctrl[TEMP].key[KEY_PRESS].e) //按e允许消耗超级电容能量
    {
    case 0:
        chassis_cmd_send.cap_open = 0;
        break;
    case 1:
        chassis_cmd_send.cap_open = 1;
        // chassis_cmd_send.vx = rc_data[TEMP].key[KEY_PRESS].a * 15000 - rc_data[TEMP].key[KEY_PRESS].d * 15000;
        break;
    }
    shoot_rate_improve();
    if(shoot_fetch_data.stalled_flag ==1) {
        shoot_cmd_send.loader_mode = LOAD_REVERSE;
    }
    death_check();

}


/**
 * @brief  紧急停止,包括遥控器左上侧拨轮打满/重要模块离线/双板通信失效等
 *         停止的阈值'300'待修改成合适的值,或改为开关控制.
 *
 * @todo   后续修改为遥控器离线则电机停止(关闭遥控器急停),通过给遥控器模块添加daemon实现
 *
 */
static void EmergencyHandler() {
    // 拨轮的向下拨超过一半进入急停模式.注意向打时下拨轮是正
    //
    if(VTremote_flag == 1 && remote_flag == 1)
    {
        if (rc_data[TEMP].rc.dial > 300 || robot_state == ROBOT_STOP || (referee_VT_data->vt_ctrl[TEMP].vt.wheel < 500 && referee_VT_data->vt_ctrl[TEMP].vt.wheel>10)) // 还需添加重要应用和模块离线的判断
        {
            robot_state = ROBOT_STOP;
            gimbal_cmd_send.gimbal_mode = GIMBAL_ZERO_FORCE;
            chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
            shoot_cmd_send.shoot_mode = SHOOT_OFF;
            shoot_cmd_send.friction_mode = FRICTION_OFF;
            chassis_cmd_send.friction_mode = FRICTION_OFF;
            shoot_cmd_send.loader_mode = LOAD_STOP;
            LOGERROR("[CMD] emergency stop!");
        }
    }
    else if(VTremote_flag == 0 && remote_flag == 1)
    {
        if (rc_data[TEMP].rc.dial > 300 || robot_state == ROBOT_STOP) // 还需添加重要应用和模块离线的判断
        {
            robot_state = ROBOT_STOP;
            gimbal_cmd_send.gimbal_mode = GIMBAL_ZERO_FORCE;
            chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
            shoot_cmd_send.shoot_mode = SHOOT_OFF;
            shoot_cmd_send.friction_mode = FRICTION_OFF;
            chassis_cmd_send.friction_mode = FRICTION_OFF;
            shoot_cmd_send.loader_mode = LOAD_STOP;
            LOGERROR("[CMD] emergency stop!");
        }
    }
    else if(VTremote_flag == 1 && remote_flag == 0)
    {
        if (robot_state == ROBOT_STOP || (referee_VT_data->vt_ctrl[TEMP].vt.wheel < 500 && referee_VT_data->vt_ctrl[TEMP].vt.wheel>10)) // 还需添加重要应用和模块离线的判断
        {
            robot_state = ROBOT_STOP;
            gimbal_cmd_send.gimbal_mode = GIMBAL_ZERO_FORCE;
            chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
            shoot_cmd_send.shoot_mode = SHOOT_OFF;
            shoot_cmd_send.friction_mode = FRICTION_OFF;
            chassis_cmd_send.friction_mode = FRICTION_OFF;
            shoot_cmd_send.loader_mode = LOAD_STOP;
            LOGERROR("[CMD] emergency stop!");
        }
    }
    // 遥控器右侧开关为[上],恢复正常运行
    if (switch_is_up(rc_data[TEMP].rc.switch_right) || referee_VT_data->vt_ctrl[TEMP].vt.trigger == 1) {
        robot_state = ROBOT_READY;
        shoot_cmd_send.shoot_mode = SHOOT_ON;
        gimbal_cmd_send.gimbal_mode = GIMBAL_FREE_MODE;
        gimbal_cmd_send.yaw = gimbal_fetch_data.gimbal_imu_data.YawTotalAngle;
        LOGINFO("[CMD] reinstate, robot ready");
    }
}

/* 机器人核心控制任务,200Hz频率运行(必须高于视觉发送频率) */
void RobotCMDTask() {
    // 从其他应用获取回传数据
#ifdef ONE_BOARD
    SubGetMessage(chassis_feed_sub, (void *) &chassis_fetch_data);
#endif // ONE_BOARD
#ifdef GIMBAL_BOARD
    chassis_fetch_data = *(Chassis_Upload_Data_s *)CANCommGet(cmd_can_comm);
#endif // GIMBAL_BOARD
    SubGetMessage(shoot_feed_sub, &shoot_fetch_data);
    SubGetMessage(gimbal_feed_sub, &gimbal_fetch_data);
    //update_ui_data();
    // 根据gimbal的反馈值计算云台和底盘正方向的夹角,不需要传参,通过static私有变量完成
    CalcOffsetAngle();
    EmergencyHandler(); // 处理模块离线和遥控器急停等紧急情况
    if(referee_VT_data->vt_ctrl[TEMP].vt.wheel != 0)
    {
        VTremote_flag = 1;
        VT_cotine = 1;
    }
    else
    {
        VTremote_flag = 0;
        VT_cotine = 0;
    }
    if(rc_data[TEMP].rc.switch_left != 0)
    {
        remote_flag = 1;
    }
    else
    {
        remote_flag = 0;
    }

    if (robot_state != ROBOT_STOP ){
        // 根据遥控器左侧开关,确定当前使用的控制模式为遥控器调试还是键鼠
        if(VTremote_flag == 1)
        {
            if (referee_VT_data->vt_ctrl[TEMP].vt.mode_sw == 1)//图传遥控器挡位在N,遥控器控制
                VTRemoteControlSet();
                //VTMouseKeySet();
            if (referee_VT_data->vt_ctrl[TEMP].vt.mode_sw == 2)//图传遥控器挡位在S,键鼠控制
                VTMouseKeySet();
        }
        else if(remote_flag == 1 && VT_cotine == 0)
        {
            // if (switch_is_down(rc_data[TEMP].rc.switch_left) ||
            //     switch_is_mid(rc_data[TEMP].rc.switch_left)) // 遥控器左侧开关状态为[下],[中],遥控器控制
                RemoteControlSet();
        }
        // else if(remote_flag == 0) {
        //     robot_state = ROBOT_STOP;
        //     gimbal_cmd_send.gimbal_mode = GIMBAL_ZERO_FORCE;
        //     chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
        //     shoot_cmd_send.shoot_mode = SHOOT_OFF;
        //     shoot_cmd_send.friction_mode = FRICTION_OFF;
        //     chassis_cmd_send.friction_mode = FRICTION_OFF;
        //     shoot_cmd_send.loader_mode = LOAD_STOP;
        //     chassis_cmd_send.loader_mode = LOAD_STOP;
        // }
        // if(referee_VT_data->vt_ctrl[TEMP].vt.mode_sw == 1)
        // {
        //     VTRemoteControlSet();
        // }
        // if(referee_VT_data->vt_ctrl[TEMP].vt.mode_sw == 2)
        // {
        //     VTMouseKeySet();
        // }
        // if (switch_is_down(rc_data[TEMP].rc.switch_left) ||
        //     switch_is_mid(rc_data[TEMP].rc.switch_left)) // 遥控器左侧开关状态为[下],[中],遥控器控制
        //     RemoteControlSet();
        // else if (switch_is_up(rc_data[TEMP].rc.switch_left) &&
        //          switch_is_mid(rc_data[TEMP].rc.switch_right)) // 遥控器左侧开关状态为[上],键盘控制,遥控器右侧开关状态为[中],遥控器通道
        //     MouseKeySet();
        // else if (switch_is_up(rc_data[TEMP].rc.switch_left) &&
        //          switch_is_up(rc_data[TEMP].rc.switch_right)) // 遥控器左侧开关状态为[上],键盘控制，遥控器右侧开关状态为[上],图传链路通道
        //     VTMouseKeySet();
    }

    // 设置视觉发送数据,还需增加加速度和角速度数据
    // VisionSetFlag(!chassis_fetch_data.enemy_color);
    VisionSetFlag(chassis_fetch_data.enemy_color);

    //VisionSetFlag
    // //根据裁判系统反馈确定底盘功率上限
    // //根据裁判系统反馈确定缓冲功率
    //根据裁判系统反馈确定枪管热量控制
    shoot_cmd_send.heat_limit = chassis_fetch_data.heat_limit;
    //shoot_cmd_send.heat_cool = chassis_fetch_data.heat_colo;
    gimbal_cmd_send.real_wz = chassis_fetch_data.real_wz;
    // shoot_cmd_send.heat_limit = referee_data->GameRobotState.shooter_barrel_heat_limit;
    shoot_cmd_send.heat_cool = chassis_fetch_data.cooling_speed;
    shoot_cmd_send.robot_level = chassis_fetch_data.robot_level;
    shoot_cmd_send.heat = chassis_fetch_data.heat;
    if(referee_data->GameRobotState.power_management_chassis_output == 0){
        // chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
    }
    if(referee_data->GameRobotState.power_management_gimbal_output == 0){
        // gimbal_cmd_send.gimbal_mode = GIMBAL_ZERO_FORCE;
    }
    if (referee_data->GameRobotState.power_management_shooter_output == 0){
        // shoot_cmd_send.shoot_mode = SHOOT_OFF;
    }

    // death_check();
    // 推送消息,双板通信,视觉通信等
    // 其他应用所需的控制数据在remotecontrolsetmode和mousekeysetmode中完成设置
#ifdef ONE_BOARD
    PubPushMessage(chassis_cmd_pub, (void *) &chassis_cmd_send);
#endif // ONE_BOARD
#ifdef GIMBAL_BOARD
    CANCommSend(cmd_can_comm, (void *)&chassis_cmd_send);
     // CANCommSend(shoot_can_comm,(void *)&shoot_cmd_send);
#endif // GIMBAL_BOARD
    PubPushMessage(shoot_cmd_pub, (void *) &shoot_cmd_send);
    PubPushMessage(gimbal_cmd_pub, (void *) &gimbal_cmd_send);

}
