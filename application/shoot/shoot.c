#include "shoot.h"
#include "robot_def.h"

#include "dji_motor.h"
#include "message_center.h"
#include "bsp_dwt.h"
#include "general_def.h"
#include "servo_motor.h"

#define CURRENT_QUEUE_SIZE     100     // 电流窗口队列长度
#define DEFAULT_Q_THRESHOLD    45.0f  // 初始电流差值阈值（需校准）
#define HEAT_PER_BULLET        10.0f  // 每发子弹热量增量
#define MAX_ALLOWED_HEAT       100.0f // 最大允许热量
#define COOLING_RATE_SCALE     0.005f // 冷却系数毫秒转换

#define Heat_Limit_Slowdown_Threshold     50.0f // 冷却系数毫秒转换
#define Heat_Limit_Ceasefire_Threshold    20.0f // 冷却系数毫秒转换

/* 对于双发射机构的机器人,将下面的数据封装成结构体即可,生成两份shoot应用实例 */
static DJIMotorInstance *friction_l, *friction_r, *loader; // 拨盘电机
static ServoInstance *lid;

static Publisher_t *shoot_pub;
static Shoot_Ctrl_Cmd_s shoot_cmd_recv; // 来自cmd的发射控制信息
static Subscriber_t *shoot_sub;
static Shoot_Upload_Data_s shoot_feedback_data; // 来自cmd的发射控制信息
static uint16_t heat_limit;
// dwt定时,计算冷却用
static float hibernate_time = 0,last_hibernate_time = 0, dead_time = 0;
static float current_loader;
static float speed_loader;
float speed_loader_set = 8;

static HeatControl_t heat_control;
float shoot_speed;
float ref_speed = 1000.0f,fan_speed = 12.0f,fan_time=50;


void ShootInit()
{
    // 左摩擦轮
    Motor_Init_Config_s friction_config = {
        .can_init_config = {
            .can_handle = &hcan2,
    },
    .controller_param_init_config = {
            .speed_PID = {
                .Kp = 1.5f,
                .Ki = 0.2f,
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 10000,
                .MaxOut = 15000,
        },
        .current_PID = {
                .Kp = 0,
                .Ki = 0,
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 10000,
                .MaxOut = 15000,
        },
},
.controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP,
    },
    .motor_type = M3508};
    friction_config.can_init_config.tx_id = 2,
            friction_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    //friction_config.controller_setting_init_config.feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL;
    friction_l = DJIMotorInit(&friction_config);

    friction_config.can_init_config.tx_id = 4; // 右摩擦轮,改txid和方向就行
    friction_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    //friction_config.controller_setting_init_config.feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL;
    friction_r = DJIMotorInit(&friction_config);

    // 拨盘电机
    Motor_Init_Config_s loader_config = {
            .can_init_config = {
                    .can_handle = &hcan1,
                    .tx_id = 1
            },
            .controller_param_init_config = {
                    .angle_PID = {
                            // 如果启用位置环来控制发弹,需要较大的I值保证输出力矩的线性度否则出现接近拨出的力矩大幅下降
                            .Kp = 15,
                            .Ki = 0,
                            .Kd = 0,
                            .MaxOut = 10000,
                    },
                    .speed_PID = {
                            .Kp = 3.5, // 3
                            .Ki = 1, // 1
                            .Kd = 0,
                            .Improve = PID_Integral_Limit,
                            .IntegralLimit = 5000,
                            .MaxOut = 10000,
                    },
                    .current_PID = {
                            .Kp = 0, // 0.7
                            .Ki = 0, // 0.1
                            .Kd = 0,
                            .Improve = PID_Integral_Limit,
                            .IntegralLimit = 5000,
                            .MaxOut = 5000,
                    },
            },
            .controller_setting_init_config = {
                    .angle_feedback_source = MOTOR_FEED,
                    .speed_feedback_source = MOTOR_FEED,
                    .outer_loop_type = SPEED_LOOP, // 初始化成SPEED_LOOP,让拨盘停在原地,防止拨盘上电时乱转
                    .close_loop_type = SPEED_LOOP | ANGLE_LOOP,
                    .motor_reverse_flag = MOTOR_DIRECTION_NORMAL, // 注意方向设置为拨盘的拨出的击发方向
                    //.feedback_reverse_flag = FEEDBACK_DIRECTION_REVERSE,
            },
            .motor_type = M3508 // 英雄使用m3508
    };
    loader = DJIMotorInit(&loader_config);
    shoot_pub = PubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));
    shoot_sub = SubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));

}


// //卡弹检测
// void stalled_detect()
// {
//     static float last_detect_time = 0,detect_time = 0;
//     static uint8_t stalled_cnt = 0;
//
//     detect_time = DWT_GetTimeline_ms();
//     if(detect_time - last_detect_time < 30.0f)  // 800ms 检测一次
//         return;
//     if(shoot_cmd_recv.loader_mode == LOAD_BURSTFIRE)
//     {
//         if(abs(loader->measure.speed_aps)<=500)
//         {
//             stalled_cnt ++;
//         }
//         else
//         {
//             stalled_cnt = 0;
//         }
//         if (stalled_cnt >= 3)
//         {
//             shoot_feedback_data.stalled_flag = 1;
//             stalled_cnt=0;
//         }
//         last_detect_time = DWT_GetTimeline_ms();
//     }
// }


//卡弹检测
void stalled_detect()
{
    static float last_detect_time = 0,detect_time = 0;
    static uint8_t stalled_cnt = 0;

    detect_time = DWT_GetTimeline_ms();
    if(detect_time - last_detect_time < 30.0f)  // 800ms 检测一次
        return;
    if(shoot_cmd_recv.loader_mode == LOAD_BURSTFIRE)
    {
        if(abs(loader->measure.speed_aps)<=500)
        {
            stalled_cnt ++;
        }
        else
        {
            stalled_cnt = 0;
        }
        if (stalled_cnt >= 3)
        {
            shoot_feedback_data.stalled_flag = 1;
            stalled_cnt=0;
        }
        last_detect_time = DWT_GetTimeline_ms();
    }
}



/* 机器人发射机构控制核心任务 */
void ShootTask()
{
    //从cmd获取控制数据
    SubGetMessage(shoot_sub, &shoot_cmd_recv);
    heat_control.heat_limit = shoot_cmd_recv.heat_limit;
    heat_control.cooling_rate = shoot_cmd_recv.heat_cool;
    current_loader = loader->measure.real_current;
    speed_loader = loader->measure.speed_aps;

    // 对shoot mode等于SHOOT_STOP的情况特殊处理,直接停止所有电机(紧急停止)
    if (shoot_cmd_recv.shoot_mode == SHOOT_OFF)
    {
        DJIMotorSetRef(friction_l, 0);
        DJIMotorSetRef(friction_r, 0);
        DJIMotorStop(loader);
    }
    else // 恢复运行
    {
        DJIMotorEnable(friction_l);
        DJIMotorEnable(friction_r);
        DJIMotorEnable(loader);
    }


    // 获取机器人的极限机械射速限制
    float max_mech_speed;
    if(shoot_cmd_recv.aim_flag == 1) {
        max_mech_speed = (shoot_cmd_recv.robot_level <= 5) ? 25.0f : 30.0f;
    } else {
        max_mech_speed = (shoot_cmd_recv.robot_level <= 1) ? 20.0f : 25.0f;
    }

    if(shoot_cmd_recv.heat_mode == HEAT_OPEN && shoot_cmd_recv.loader_mode == LOAD_BURSTFIRE)
    {
        float a = shoot_cmd_recv.heat_cool; // 冷却速率
        float m = shoot_cmd_recv.heat_limit - shoot_cmd_recv.heat; // 剩余热量
        float d = HEAT_PER_BULLET; // 单发热量

        // 1. 稳态机械补偿 (解决热量往下掉的问题)
        // 理论上稳态射速 = a/d (比如 40/10 = 4)。
        // 但因为摩擦轮阻力、拨盘空腔，你给电机设定 4发/秒，它可能实际只打出 2.5发。
        // 所以我们需要给稳态速度乘一个补偿系数，这个系数根据你这台车的机械阻力来定。
        float compensate_factor = 1.0f; // 【重点调参】：如果你稳态热量还是掉，就把这个往上加(1.4, 1.5)；如果稳态热量超了，就减小(1.1)
        float target_steady_speed = (a / d) * compensate_factor;

        // 2. 计算 10Hz 延迟的绝对安全刹车距离
        // 裁判系统 100ms(0.1s) 刷新一次。在这 0.1s 内，以最高射速会打出多少热量？
        // max_mech_speed * 0.1s * d。例如：25 * 0.1 * 10 = 25 热量。
        // 我们再加上 5 作为容错底线。
        float safe_buffer = (max_mech_speed * 0.1f * d) + 5.0f;

        // 减速缓冲带：距离安全底线还有 30 热量时，开始线性收油门
        float decelerate_buffer = safe_buffer + 30.0f;

        // 3. 状态机判断输出
        if (m < d) {
            // 极限保护：连1发子弹的热量都不够了，必须停！
            shoot_speed = 0.0f;
        }
        else if (m > decelerate_buffer) {
            // 完美爆发期：距离上限还很远，无脑输出最高射速
            shoot_speed = max_mech_speed;
        }
        else if (m > safe_buffer) {
            // 减速刹车期：距离上限近了，为了防止 100ms 延迟导致超标，平滑降低射速
            // 把剩余热量在 [safe_buffer, decelerate_buffer] 映射到 [target_steady_speed, max_mech_speed]
            float ratio = (m - safe_buffer) / (decelerate_buffer - safe_buffer);
            shoot_speed = target_steady_speed + ratio * (max_mech_speed - target_steady_speed);
        }
        else {
            // 稳态压制期：已经贴近危险边缘，严格输出带补偿的冷却速率
            shoot_speed = target_steady_speed;
        }
    }
    else
    {
        shoot_speed = max_mech_speed;
    }


    // 如果上一次触发单发或3发指令的时间加上不应期仍然大于当前时间(尚未休眠完毕),直接返回即可
    // 单发模式主要提供给能量机关激活使用(以及英雄的射击大部分处于单发)
    if (hibernate_time + dead_time > DWT_GetTimeline_ms())
        return;
    static uint8_t single_shot_lock = 0;
    // 若不在休眠状态,根据robotCMD传来的控制模式进行拨盘电机参考值设定和模式切换
    switch (shoot_cmd_recv.loader_mode)
    {
        // 停止拨盘
    case LOAD_STOP:
        DJIMotorOuterLoop(loader, SPEED_LOOP); // 切换到速度环
        DJIMotorSetRef(loader, 0);             // 同时设定参考值为0,这样停止的速度最快
        single_shot_lock = 0;
        break;
        // 单发模式,根据鼠标按下的时间,触发一次之后需要进入不响应输入的状态(否则按下的时间内可能多次进入,导致多次发射)
    case LOAD_1_BULLET:           // 激活能量机关/干扰对方用,英雄用
        if (single_shot_lock == 0)
        {
            DJIMotorOuterLoop(loader, ANGLE_LOOP);                              // 切换到角度环
            DJIMotorSetRef(loader, loader->measure.total_angle + ONE_BULLET_DELTA_ANGLE * REDUCTION_RATIO_LOADER); // 控制量增加一发弹丸的角度
            hibernate_time = DWT_GetTimeline_ms();                                              // 记录触发指令的时间
            dead_time = 150;// 完成1发弹丸发射的时间
            single_shot_lock = 1;  //执行一次后立刻上锁
        }

        break;
        // 三连发,如果不需要后续可能删除
    case LOAD_2_BULLET:
        if (single_shot_lock == 0)
        {
            DJIMotorOuterLoop(loader, ANGLE_LOOP);                              // 切换到角度环
            DJIMotorSetRef(loader, loader->measure.total_angle + 2 * ONE_BULLET_DELTA_ANGLE * REDUCTION_RATIO_LOADER); // 控制量增加一发弹丸的角度
            hibernate_time = DWT_GetTimeline_ms();                                              // 记录触发指令的时间
            dead_time = 300;// 完成1发弹丸发射的时间
            single_shot_lock = 1;  //执行一次后立刻上锁
        }                                                                   // 完成3发弹丸发射的时间
        break;
        // 连发模式,对速度闭环,射频后续修改为可变,目前固定为1Hz
        case LOAD_BURSTFIRE:
            single_shot_lock = 0;                  // 切入连发时，为了安全起见也清零锁
            DJIMotorOuterLoop(loader, SPEED_LOOP);
            DJIMotorSetRef(loader, shoot_speed * 360 * REDUCTION_RATIO_LOADER / 9);
        // DJIMotorSetRef(loader, 20 * 360 * REDUCTION_RATIO_LOADER / 9);
        // x颗/秒换算成速度: 已知一圈的载弹量,由此计算出1s需要转的角度,注意换算角速度(DJIMotor的速度单位是angle per second)
        break;
        // 拨盘反转,对速度闭环,后续增加卡弹检测(通过裁判系统剩余热量反馈和电机电流)
        // 也有可能需要从switch-case中独立出来

        // case LOAD_VISION:
        //
        //
        //
        //     break;

        case LOAD_REVERSE:
            single_shot_lock = 0;                  // 切入连发时，为了安全起见也清零锁
            DJIMotorOuterLoop(loader, SPEED_LOOP);
        DJIMotorSetRef(loader, -5 * 360 * REDUCTION_RATIO_LOADER / 9);             // 固定速度反转
        hibernate_time = DWT_GetTimeline_ms();                                              // 记录触发指令的时间
        dead_time = 200;                                                                    // 翻转500ms
        shoot_feedback_data.stalled_flag = 0 ; //清除堵转标志
        // ...
        break;
    default:
        while (1); // 未知模式,停止运行,检查指针越界,内存溢出等问题
    }

    // 确定是否开启摩擦轮,后续可能修改为键鼠模式下始终开启摩擦轮(上场时建议一直开启)
    if (shoot_cmd_recv.friction_mode == FRICTION_ON)
    {
        // 根据收到的弹速设置设定摩擦轮电机参考值,需实测后填入
        //        switch (shoot_cmd_recv.bullet_speed)
        //        {
        //            case SMALL_AMU_15:
        //                DJIMotorSetRef(friction_l, 0);
        //                DJIMotorSetRef(friction_r, 0);
        //                break;
        //            case SMALL_AMU_18:
        //                DJIMotorSetRef(friction_l, 0);
        //                DJIMotorSetRef(friction_r, 0);
        //                break;
        //            case SMALL_AMU_30:
        //                DJIMotorSetRef(friction_l, 0);
        //                DJIMotorSetRef(friction_r, 0);
        //                break;
        // default: // 当前为了调试设定的默认值4000,因为还没有加入裁判系统无法读取弹速.
        DJIMotorSetRef(friction_l, shoot_cmd_recv.friction_speed);
        DJIMotorSetRef(friction_r, shoot_cmd_recv.friction_speed);
                       // break;
        //        }
    }
    else // 关闭摩擦轮
    {
        DJIMotorSetRef(friction_l, 0);
        DJIMotorSetRef(friction_r, 0);
    }


        stalled_detect();

    // 反馈数据,目前暂时没有要设定的反馈数据,后续可能增加应用离线监测以及卡弹反馈
    PubPushMessage(shoot_pub, (void *)&shoot_feedback_data);
}