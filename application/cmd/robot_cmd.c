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
#include "super_cap.h"
#include "Power.h"
#include "referee_VT.h"

// bsp
#include "bsp_dwt.h"
#include "bsp_log.h"
#include "referee_task.h"
#include "vision_transfer.h"


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
static float diff_yaw;
//static Vision_Recv_s *vision_recv_data; // 视觉接收数据指针,初始化时返回

static RecievePacket_t *vision_recv_data; // 视觉接收数据指针,初始化时返回
VT_info_t *referee_VT_data; //图传链路数据

//自瞄相关信息
static uint8_t aim_suggest_fire;
static uint16_t friction_speed = 650;

static Publisher_t *gimbal_cmd_pub;            // 云台控制消息发布者
static Subscriber_t *gimbal_feed_sub;          // 云台反馈信息订阅者
static Gimbal_Ctrl_Cmd_s gimbal_cmd_send;      // 传递给云台的控制信息
static Gimbal_Upload_Data_s gimbal_fetch_data; // 从云台获取的反馈信息

static Publisher_t *shoot_cmd_pub;           // 发射控制消息发布者
static Subscriber_t *shoot_feed_sub;         // 发射反馈信息订阅者
static Shoot_Upload_Data_s shoot_fetch_data; // 从发射获取的反馈信息
static loader_mode_e loader_current_mode;
static float vision_fire_hold_time = 0;



static Robot_Status_e robot_state; // 机器人整体工作状态

static referee_info_t *referee_data; // 用于获取裁判系统的数据
static Power_data power_info;			  // 裁判系统数据

static uint8_t loader_flag = 0;      //拨弹模式选择标志位
float tmp_chassis_velocitx_max, tmp_chassis_velocity_max;
uint8_t remote_flag = 0,VTremote_flag = 0,VT_cotine = 0;
uint8_t press_left_last = 0;
uint8_t press_left = 0;
chassis_mode_e Chassis_Current_Mode = CHASSIS_NO_FOLLOW;

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
#define TEST_FREQ_HZ        25.0f       // 正弦波频率 1Hz
#define TEST_AMP_YAW_SPEED_RAD_S (150.0f * DEGREE_2_RAD)
#define STEP_INTERVAL_MS    1000       // 阶跃跳变间隔 (ms)
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
    shoot_cmd_send.friction_speed = friction_speed;
    chassis_cmd_send.friction_speed = friction_speed;

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
 * @brief Pitch编码器辅助软限位，解决上下坡时坐标系变化导致陀螺仪限位失效的问题
 *
 * @note 1. 注意核对单位，current_ecd与current_imu单位应统一
 *       2. 由于刚启动时云台未上电，读到的Pitch电机编码值为0，故应在所有的函数调用前加入lost_flag判断，防止引发异常
 *       3. 此逻辑易与其他算法产生冲突，谨慎使用
 */
static void Pitch_Dynamic_Limit_Update() {
    float current_ecd = gimbal_fetch_data.pitch_motor_single_round_angle; // DM position 本就 rad

    float current_imu = gimbal_fetch_data.gimbal_imu_data.Pitch;

    float delta_angle = gimbal_cmd_send.pitch - current_imu;

    // 假设 IMU 抬头为正，且编码器抬头也为正
    // 如果编码器抬头数值减小，这里改成减号
    float predicted_ecd = current_ecd + delta_angle;

    if (predicted_ecd > PITCH_MAX_ECD) {
        // 预测会撞到上边，反推允许的最大 IMU 目标值
        // 允许的最大变化量 = 距离上限的剩余空间
        float max_allowable_delta = PITCH_MAX_ECD - current_ecd;
        gimbal_cmd_send.pitch = current_imu + max_allowable_delta;
    }
    else if (predicted_ecd < PITCH_MIN_ECD) {
        // 预测会撞到下边，反推允许的最小 IMU 目标值
        float min_allowable_delta = PITCH_MIN_ECD - current_ecd;
        gimbal_cmd_send.pitch = current_imu + min_allowable_delta; // 注意符号
    }
}


static void Gimbal_Test_Signal_Generator(void) {

    test_time_tick += CONTROL_PERIOD_S;

    // 1. 正弦波模式 (Sine Wave)
    if (current_test_mode == TEST_SINE) {
        // Yaw: 以进入模式时的角度为中心进行正弦摆动
        gimbal_cmd_send.yaw_speed = TEST_AMP_YAW_SPEED_RAD_S * sinf(2 * PI * TEST_FREQ_HZ * test_time_tick);

        // Pitch: 同理
        gimbal_cmd_send.pitch_speed = 0;
    }
    // 2. 阶跃模式 (Step Response / Square Wave)
    else if (current_test_mode == TEST_STEP) {
        // 利用取余运算实现方波
        // (int)(time * 1000) % (2000) < 1000
        uint32_t time_ms = (uint32_t)(test_time_tick * 1000);

        if (time_ms % (STEP_INTERVAL_MS * 2) < STEP_INTERVAL_MS) {
            // High Level
            gimbal_cmd_send.yaw_speed = TEST_AMP_YAW_SPEED_RAD_S;
            gimbal_cmd_send.pitch_speed = 0;
        } else {
            // Low Level (回到中心或反向)
            gimbal_cmd_send.yaw_speed = -TEST_AMP_YAW_SPEED_RAD_S;
            gimbal_cmd_send.pitch_speed = 0;
        }
    }
}

static void CalcOffsetAngle() {
    // 计算偏移角度
    float offset = gimbal_fetch_data.yaw_motor_single_round_angle - YAW_CHASSIS_ALIGN_RAD;
    // 将偏移角度归一化
    if (offset > PI) {
        offset -= 2*PI;
    } else if (offset < -PI) {
        offset += 2*PI;
    }
    // 赋值给输出
    chassis_cmd_send.offset_angle = offset;
    gimbal_cmd_send.offset_angle = offset;

}

static void Vision_UI_Update() {
    // 涉及开火决策，必须在task开头调用，否则无法作出发射判定
    if (vision_recv_data->distance != -1 && vision_recv_data->yaw != 0 && vision_recv_data->pitch != 0) {
        chassis_cmd_send.vision_find_target_flag = 1;
        if (vision_recv_data->mode == 1) {
            aim_suggest_fire = 1;
            chassis_cmd_send.auto_aim_fire_suggestion = 1;
        } else {
            aim_suggest_fire = 0;
            chassis_cmd_send.auto_aim_fire_suggestion = 0;
        }
    } else {
        aim_suggest_fire = 0;
        chassis_cmd_send.auto_aim_fire_suggestion = 0;
        chassis_cmd_send.vision_find_target_flag = 0;
    }
}

//功能：死亡后清除小陀螺的状态
static void death_check()
{
    if(chassis_fetch_data.dead_state)
    {
        referee_VT_data->vt_ctrl[TEMP].key[KEY_PRESS].shift = 0;
        gimbal_cmd_send.yaw = gimbal_fetch_data.gimbal_imu_data.YawTotalAngle;
        // gimbal_cmd_send.gimbal_mode = GIMBAL_ZERO_FORCE;
        // shoot_cmd_send.shoot_mode = SHOOT_OFF;
        // chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
        // chassis_cmd_send.friction_mode = FRICTION_OFF;
        // shoot_cmd_send.loader_mode = LOAD_STOP;
    }
}

//软件复位函数
void System_Reset(void) {
    __set_FAULTMASK(1); //关闭所有中断
    NVIC_SystemReset(); //进行软件复位
}

static void auto_aim_mode() {
        // 视觉控制云台
            diff_yaw = vision_recv_data->yaw ;

            if (diff_yaw > PI)
                diff_yaw -= 2.0f * PI;
            else if (diff_yaw < -PI)
                diff_yaw += 2.0f * PI;

            // 调整云台角度
            gimbal_cmd_send.yaw = diff_yaw;
            gimbal_cmd_send.pitch = vision_recv_data->pitch; // 视觉 pitch 为弧度
            if (!gimbal_fetch_data.pitch_motor_lost_flag) {
                Pitch_Dynamic_Limit_Update();
            }
}

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
        chassis_cmd_send.chassis_mode = CHASSIS_NO_FOLLOW;
        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
        shoot_cmd_send.lid_mode = LID_CLOSE;
    } else if (switch_is_up(rc_data[TEMP].rc.switch_right))// 右侧开关状态[上]
    {
        chassis_cmd_send.chassis_mode = CHASSIS_NO_FOLLOW; //TODO 待测试
    }
    // 云台参数,确定云台控制数据
    if (switch_is_mid(rc_data[TEMP].rc.switch_left) ||
        (vision_recv_data->distance==-1)) // 左侧开关状态为[中],或视觉未识别到目标,纯遥控器拨杆控制
    {
        gimbal_cmd_send.auto_aim_mode = 0;
        chassis_cmd_send.auto_aim_mode = 0;
        // 待添加,视觉会发来目标角速度和角加速度 rad/s  rad/s^2
        // ...
        if (gimbal_cmd_send.gimbal_mode!=GIMBAL_ZERO_FORCE) {
            gimbal_cmd_send.yaw_speed = -0.25f * DEGREE_2_RAD * (float) rc_data[TEMP].rc.rocker_l_;
            //gimbal_cmd_send.pitch += 0.001f * (float) rc_data[TEMP].rc.rocker_l1;
            gimbal_cmd_send.pitch_speed = 0.25f * DEGREE_2_RAD * (float) rc_data[TEMP].rc.rocker_l1;
        }
         //Gimbal_Test_Signal_Generator();

    }
    // 左侧开关状态为[下],视觉模式

    if (switch_is_down(rc_data[TEMP].rc.switch_left) || switch_is_up(rc_data[TEMP].rc.switch_left)) {
        if (vision_recv_data->distance != -1 && vision_recv_data->yaw != 0 && vision_recv_data->pitch != 0) {
            auto_aim_mode();
            gimbal_cmd_send.auto_aim_mode = 1;
            chassis_cmd_send.auto_aim_mode = 1;
            gimbal_cmd_send.vision_yaw_speed_ff = vision_recv_data->yaw_v;
            gimbal_cmd_send.vision_pitch_speed_ff = vision_recv_data->pitch_v;
            gimbal_cmd_send.vision_yaw_acc_ff = vision_recv_data->yaw_acc;
            gimbal_cmd_send.vision_pitch_acc_ff = vision_recv_data->pitch_acc;
        }
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
    if (switch_is_up(rc_data[TEMP].rc.switch_left)) {

        shoot_cmd_send.friction_mode = FRICTION_ON;
        chassis_cmd_send.friction_mode = FRICTION_ON;

    }

    // 拨弹控制,遥控器固定为一种拨弹模式,可自行选择
    if (rc_data[TEMP].rc.dial < -500){//
        if(switch_is_down(rc_data[TEMP].rc.switch_left)&&aim_suggest_fire == 1){
            shoot_cmd_send.shoot_mode = SHOOT_ON;
            shoot_cmd_send.loader_mode = LOAD_BURSTFIRE;
        }
        else if (switch_is_mid(rc_data[TEMP].rc.switch_left)){
            shoot_cmd_send.shoot_mode = SHOOT_ON;
            shoot_cmd_send.loader_mode = LOAD_BURSTFIRE;
        }
        else if (switch_is_up(rc_data[TEMP].rc.switch_left)) {
            shoot_cmd_send.shoot_mode = SHOOT_ON;
            if (aim_suggest_fire == 1) {
                shoot_cmd_send.loader_mode = LOAD_1_BULLET;
            }
            else {

                shoot_cmd_send.loader_mode = LOAD_STOP;
            }
        }
    }
    else
    {
        shoot_cmd_send.loader_mode = LOAD_STOP;
        shoot_cmd_send.shoot_mode = SHOOT_OFF;
    }

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
    shoot_cmd_send.heat_mode = HEAT_CLOSE;
    if (shoot_fetch_data.stalled_flag == 1)
        shoot_cmd_send.loader_mode = LOAD_REVERSE;
    // if(chassis_fetch_data.dead_state)
    //     gimbal_cmd_send.yaw = gimbal_fetch_data.gimbal_imu_data.YawTotalAngle;
    //拆除了裁判系统导致一直处于阵亡状态

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
        aim_suggest_fire = 0;
        gimbal_cmd_send.yaw_speed = -0.0025f * DEGREE_2_RAD * (float)referee_VT_data->vt_ctrl[TEMP].vt.ch_left_x;
        gimbal_cmd_send.pitch_speed = 0.001f * DEGREE_2_RAD * (float)referee_VT_data->vt_ctrl[TEMP].vt.ch_left_y;
    }

    gimbal_cmd_send.auto_aim_mode = 0;
    float VTright_,VTright1;
    VTright_ = (float)referee_VT_data->vt_ctrl[TEMP].vt.ch_right_x / 660.0f;
    VTright1 = (float)referee_VT_data->vt_ctrl[TEMP].vt.ch_right_y / 660.0f;

    if (abs(VTright_) < 0.03f) VTright_ = 0;
    if (abs(VTright1) < 0.03f) VTright1 = 0;

    chassis_cmd_send.vx = 5.0f * VTright1;
    chassis_cmd_send.vy = 5.0f * VTright_;
    if(referee_VT_data->vt_ctrl[TEMP].vt.fn_left == 1)
    {
        if (vision_recv_data->distance != -1 && vision_recv_data->yaw != 0 && vision_recv_data->pitch != 0) {
            auto_aim_mode();
            gimbal_cmd_send.auto_aim_mode = 1;
            gimbal_cmd_send.vision_yaw_speed_ff = vision_recv_data->yaw_v;
            gimbal_cmd_send.vision_pitch_speed_ff = vision_recv_data->pitch_v;
            gimbal_cmd_send.vision_yaw_acc_ff = vision_recv_data->yaw_acc;
            gimbal_cmd_send.vision_pitch_acc_ff = vision_recv_data->pitch_acc;
        }
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
        if(referee_VT_data->vt_ctrl[TEMP].vt.mode_sw == 0 &&vision_recv_data->mode==0 &&aim_suggest_fire == 0){
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

    if (shoot_fetch_data.stalled_flag == 1)
        shoot_cmd_send.loader_mode = LOAD_REVERSE;
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
    chassis_cmd_send.vx = (float) key_w * 3.0f - (float) key_s * 3.0f; // 系数待测
    chassis_cmd_send.vy = (float) key_d * 3.0f - (float) key_a * 3.0f;

    gimbal_cmd_send.yaw_speed = -(float) referee_VT_data->vt_ctrl[TEMP].mouse.x / 660 * 20.0f * DEGREE_2_RAD;
    gimbal_cmd_send.pitch_speed = (float) referee_VT_data->vt_ctrl[TEMP].mouse.y / 660 * 30.0f * DEGREE_2_RAD;

    chassis_cmd_send.auto_aim_mode = 0;
    gimbal_cmd_send.auto_aim_mode = 0;
    press_left = referee_VT_data->vt_ctrl[TEMP].mouse.left;
    if (referee_VT_data->vt_ctrl[TEMP].mouse.left  && (!referee_VT_data->vt_ctrl[TEMP].mouse.right)) // 左键发射模式
    {
        if (shoot_cmd_send.friction_mode == FRICTION_ON)
        {
            shoot_cmd_send.shoot_mode = SHOOT_ON;
            if(loader_flag == 0){
                shoot_cmd_send.loader_mode = LOAD_BURSTFIRE;
                chassis_cmd_send.shoot_mode = LOAD_BURSTFIRE;
            }
            else if(loader_flag == 1){
                shoot_cmd_send.loader_mode = LOAD_1_BULLET;
                chassis_cmd_send.shoot_mode = LOAD_1_BULLET;
            }
            else if(loader_flag == 2){
                shoot_cmd_send.loader_mode = LOAD_2_BULLET;
                chassis_cmd_send.shoot_mode = LOAD_2_BULLET;
            }
        }
    }
    else if ((!referee_VT_data->vt_ctrl[TEMP].mouse.left) && (!referee_VT_data->vt_ctrl[TEMP].mouse.right))
    {
        shoot_cmd_send.loader_mode = LOAD_STOP;
    }
    press_left_last = press_left ; // 更新状态
    if (referee_VT_data->vt_ctrl[TEMP].mouse.right) // 右键自瞄模式
    {
        chassis_cmd_send.auto_aim_mode = 1;
        gimbal_cmd_send.auto_aim_mode = 1;
        if (vision_recv_data->distance != -1 && vision_recv_data->yaw != 0 && vision_recv_data->pitch != 0) {
            auto_aim_mode();
            gimbal_cmd_send.vision_yaw_speed_ff = vision_recv_data->yaw_v;
            gimbal_cmd_send.vision_pitch_speed_ff = vision_recv_data->pitch_v;
            gimbal_cmd_send.vision_yaw_acc_ff = vision_recv_data->yaw_acc;
            gimbal_cmd_send.vision_pitch_acc_ff = vision_recv_data->pitch_acc;
        }
        if (aim_suggest_fire == 1 && referee_VT_data->vt_ctrl[TEMP].mouse.left && shoot_cmd_send.friction_mode == FRICTION_ON)
        {
            shoot_cmd_send.shoot_mode = SHOOT_ON;
            if(loader_flag == 0){
                shoot_cmd_send.loader_mode = LOAD_BURSTFIRE;
                chassis_cmd_send.shoot_mode = LOAD_BURSTFIRE;
            }
            else if(loader_flag == 1){
                shoot_cmd_send.loader_mode = LOAD_1_BULLET;
                chassis_cmd_send.shoot_mode = LOAD_1_BULLET;
            }
            else if(loader_flag == 2){
                shoot_cmd_send.loader_mode = LOAD_2_BULLET;
                chassis_cmd_send.shoot_mode = LOAD_2_BULLET;
            }
        }
        else if (chassis_cmd_send.auto_aim_fire_suggest_mode ==IGNORE_FIRE_SUGGESTION && referee_VT_data->vt_ctrl[TEMP].mouse.left && shoot_cmd_send.friction_mode == FRICTION_ON)
        {
            shoot_cmd_send.shoot_mode = SHOOT_ON;
            if(loader_flag == 0){
                shoot_cmd_send.loader_mode = LOAD_BURSTFIRE;
                chassis_cmd_send.shoot_mode = LOAD_BURSTFIRE;
            }
            else if(loader_flag == 1){
                shoot_cmd_send.loader_mode = LOAD_1_BULLET;
                chassis_cmd_send.shoot_mode = LOAD_1_BULLET;
            }
            else if(loader_flag == 2){
                shoot_cmd_send.loader_mode = LOAD_2_BULLET;
                chassis_cmd_send.shoot_mode = LOAD_2_BULLET;
            }
        }
        else
        {
            shoot_cmd_send.loader_mode = LOAD_STOP;
        }
    }
    switch (referee_VT_data->vt_ctrl[TEMP].key_count[KEY_PRESS][Key_B] % 3) // B键换单发,双发，连发模式
    {
    case 0:
        loader_flag = 0;
        chassis_cmd_send.shoot_mode = LOAD_BURSTFIRE;
        break;
    case 1:
        loader_flag = 1;
        chassis_cmd_send.shoot_mode = LOAD_1_BULLET;
        break;
    case 2:
        loader_flag = 2;
        chassis_cmd_send.shoot_mode = LOAD_2_BULLET;
        break;
    default:
        loader_flag = 0;
        chassis_cmd_send.shoot_mode = LOAD_BURSTFIRE;
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
        if (chassis_fetch_data.heat_limit!=0) {
            shoot_cmd_send.heat_mode = HEAT_OPEN;
            chassis_cmd_send.heat_open = 1;
        }
        else {
            shoot_cmd_send.heat_mode = HEAT_CLOSE;
            chassis_cmd_send.heat_open = 0;
        }
       //chassis_cmd_send.heat_mode = HEAT_OPEN;
        break;
    default:
        shoot_cmd_send.heat_mode = HEAT_CLOSE;
        chassis_cmd_send.heat_open = 0;
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
    if(referee_VT_data->vt_ctrl[TEMP].key_count[KEY_PRESS][Key_Z]>=1) // Z键削减摩擦轮转速
    {
        friction_speed = friction_speed - 500;
        if (friction_speed<30000) friction_speed = 30000;
        shoot_cmd_send.friction_speed = friction_speed;
        chassis_cmd_send.friction_speed = friction_speed;
        referee_VT_data->vt_ctrl[TEMP].key_count[KEY_PRESS][Key_Z]=0;
    }
    if(referee_VT_data->vt_ctrl[TEMP].key_count[KEY_PRESS][Key_C]>=1) // C键增加摩擦轮转速
    {
        friction_speed = friction_speed + 500;
        if (friction_speed>38000) friction_speed = 38000;
        shoot_cmd_send.friction_speed = friction_speed;
        chassis_cmd_send.friction_speed = friction_speed;
        referee_VT_data->vt_ctrl[TEMP].key_count[KEY_PRESS][Key_C]=0;
    }
    switch (referee_VT_data->vt_ctrl[TEMP].key_count[KEY_PRESS][Key_X] % 2) // X键切换自瞄射击目标
    {
        case 0:
            chassis_cmd_send.auto_aim_fire_suggest_mode = TRUST_FIRE_SUGGESTION;
        break;
        case 1:
            chassis_cmd_send.auto_aim_fire_suggest_mode = IGNORE_FIRE_SUGGESTION;
        break;
        default:
            chassis_cmd_send.auto_aim_fire_suggest_mode = TRUST_FIRE_SUGGESTION;
        break;
    }

    if (referee_VT_data->vt_ctrl[TEMP].key[KEY_PRESS].shift == 0) {
        switch (referee_VT_data->vt_ctrl[TEMP].key[KEY_PRESS].ctrl) //按住ctrl反向小陀螺
        {
            case 0:
                chassis_cmd_send.chassis_mode = Chassis_Current_Mode;
            gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
            break;
            case 1:
                chassis_cmd_send.chassis_mode = CHASSIS_ROTATE;
            gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
            chassis_cmd_send.chassis_rotate_reverse_flag = 1;
            break;

            default:
                chassis_cmd_send.chassis_mode = Chassis_Current_Mode;
            gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
            break;
        }
    }

    if (referee_VT_data->vt_ctrl[TEMP].key[KEY_PRESS].ctrl == 0) {
        switch (referee_VT_data->vt_ctrl[TEMP].key[KEY_PRESS].shift) //按住shift小陀螺
        {
            case 0:
                chassis_cmd_send.chassis_mode = Chassis_Current_Mode;
            gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
            break;
            case 1:
                chassis_cmd_send.chassis_mode = CHASSIS_ROTATE;
            gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
            chassis_cmd_send.chassis_rotate_reverse_flag = 0;
            break;

            default:
                chassis_cmd_send.chassis_mode = Chassis_Current_Mode;
            gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
            break;
        }
    }
    // static uint8_t x_key_prev_state = 0;
    // uint8_t x_key_current = rc_data[TEMP].key[KEY_PRESS].x;
    // if (x_key_current && !x_key_prev_state) {
    //     gimbal_cmd_send.yaw += 180.0f; // 根据方向调整正负号
    // }
    // x_key_prev_state = x_key_current;
    switch (referee_VT_data->vt_ctrl[TEMP].key_count[KEY_PRESS][Key_E] % 2) //按e允许消耗超级电容能量
    {
    case 0:
        chassis_cmd_send.cap_open = 0;
        break;
    case 1:
        chassis_cmd_send.cap_open = 1;
        // chassis_cmd_send.vx = rc_data[TEMP].key[KEY_PRESS].a * 15000 - rc_data[TEMP].key[KEY_PRESS].d * 15000;
        break;
    }

    switch (referee_VT_data->vt_ctrl[TEMP].key_count[KEY_PRESS][Key_V] % 3) // V键切换底盘模式
    {
        case 0:
            Chassis_Current_Mode = CHASSIS_NO_FOLLOW;
        break;
        case 1:
            Chassis_Current_Mode = CHASSIS_FOLLOW_GIMBAL_YAW;
        break;
        case 2:
            Chassis_Current_Mode = CHASSIS_WAVE_ROAD;
        break;
        default:
            Chassis_Current_Mode = CHASSIS_NO_FOLLOW;
        break;
    }

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
    else {
        robot_state = ROBOT_STOP;
        gimbal_cmd_send.gimbal_mode = GIMBAL_ZERO_FORCE;
        chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
        shoot_cmd_send.shoot_mode = SHOOT_OFF;
        shoot_cmd_send.friction_mode = FRICTION_OFF;
        chassis_cmd_send.friction_mode = FRICTION_OFF;
        shoot_cmd_send.loader_mode = LOAD_STOP;
        LOGERROR("[CMD] emergency stop!");
    }
    // 遥控器右侧开关为[上],恢复正常运行
    if (switch_is_up(rc_data[TEMP].rc.switch_right) || referee_VT_data->vt_ctrl[TEMP].vt.trigger == 1) {
        robot_state = ROBOT_READY;
        shoot_cmd_send.shoot_mode = SHOOT_ON;
        gimbal_cmd_send.gimbal_mode = GIMBAL_ZERO_FORCE;
        gimbal_cmd_send.yaw = gimbal_fetch_data.gimbal_imu_data.YawTotalAngle;
        LOGINFO("[CMD] reinstate, robot ready");
    }
}

/* 机器人核心控制任务,200Hz频率运行(必须高于视觉发送频率) */
void RobotCMDTask() {
    static uint8_t last_aim_suggest_fire;
    uint8_t aim_suggest_fire_rising;

    // 从其他应用获取回传数据
#ifdef ONE_BOARD
    SubGetMessage(chassis_feed_sub, (void *) &chassis_fetch_data);
#endif // ONE_BOARD
#ifdef GIMBAL_BOARD
#endif // GIMBAL_BOARD
    SubGetMessage(shoot_feed_sub, &shoot_fetch_data);
    SubGetMessage(gimbal_feed_sub, &gimbal_fetch_data);
    gimbal_cmd_send.vision_yaw_speed_ff = 0;
    gimbal_cmd_send.vision_pitch_speed_ff = 0;
    gimbal_cmd_send.vision_yaw_acc_ff = 0;
    gimbal_cmd_send.vision_pitch_acc_ff = 0;
    chassis_cmd_send.force_reset_flag = 0;
    gimbal_cmd_send.DMMOTOR_Error_Clear_flag = 0;
    chassis_fetch_data = *(Chassis_Upload_Data_s *)CANCommGet(cmd_can_comm);
    Vision_UI_Update();
    aim_suggest_fire_rising = aim_suggest_fire && !last_aim_suggest_fire;
    last_aim_suggest_fire = aim_suggest_fire;
    shoot_cmd_send.vision_fire_trigger = 0;
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
    if(VTremote_flag == 1 && referee_VT_data->vt_ctrl[TEMP].vt.mode_sw == 2 && referee_VT_data->vt_ctrl[TEMP].vt.wheel > 1500)
    {

                gimbal_cmd_send.DMMOTOR_Error_Clear_flag = 1;
                robot_state = ROBOT_STOP;
                gimbal_cmd_send.gimbal_mode = GIMBAL_ZERO_FORCE;
                chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
                shoot_cmd_send.shoot_mode = SHOOT_OFF;
                shoot_cmd_send.friction_mode = FRICTION_OFF;
                chassis_cmd_send.friction_mode = FRICTION_OFF;
                shoot_cmd_send.loader_mode = LOAD_STOP;
                chassis_cmd_send.force_reset_flag = 1;
                LOGERROR("[CMD] Force Reset!");
                static uint8_t reset_delay = 0;
                reset_delay++;
                if (reset_delay > 6) {
                    System_Reset();
                }
    }

    if (robot_state != ROBOT_STOP ){
        // 根据遥控器左侧开关,确定当前使用的控制模式为遥控器调试还是键鼠
        if(VTremote_flag == 1)
        {
            if (referee_VT_data->vt_ctrl[TEMP].vt.mode_sw == 1)//图传遥控器挡位在N,遥控器控制
                VTRemoteControlSet();
            if (referee_VT_data->vt_ctrl[TEMP].vt.mode_sw == 2)//图传遥控器挡位在S,键鼠控制
            {
                VTMouseKeySet();
            }
        }
        else if(remote_flag == 1 && VT_cotine == 0)
        {
                RemoteControlSet();
        }
    }

    //0装甲板打红，1装甲板打蓝，2小符红，3小符蓝，4打大符红，5大符蓝（视觉预研中，未加入）
    VisionSetFlag(chassis_fetch_data.enemy_color);

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
    shoot_cmd_send.bullet_speed = chassis_fetch_data.bullet_speed;
    if(referee_data->GameRobotState.power_management_chassis_output == 0){
        // chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
    }
    if(referee_data->GameRobotState.power_management_gimbal_output == 0){
        // gimbal_cmd_send.gimbal_mode = GIMBAL_ZERO_FORCE;
    }
    if (referee_data->GameRobotState.power_management_shooter_output == 0){
        // shoot_cmd_send.shoot_mode = SHOOT_OFF;
    }

    if (aim_suggest_fire_rising && gimbal_cmd_send.auto_aim_mode &&
        shoot_cmd_send.shoot_mode == SHOOT_ON &&
        shoot_cmd_send.friction_mode == FRICTION_ON &&
        (shoot_cmd_send.loader_mode == LOAD_1_BULLET ||
         shoot_cmd_send.loader_mode == LOAD_BURSTFIRE)) {
        shoot_cmd_send.vision_fire_trigger = 1;
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
