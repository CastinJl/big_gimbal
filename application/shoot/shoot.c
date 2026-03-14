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
                            .Kp = 500,
                            .Ki = 600,
                            .Kd = 0,
                            .MaxOut = 30000,
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
    // ------------------- 热量与射速控制核心策略（预测+稳态算法） -------------------

    // 用于记录连发的计时状态
    static uint32_t burst_start_time = 0;
    static uint8_t is_bursting = 0;
    static float planned_burst_speed = 0.0f;
    static uint32_t planned_burst_duration = 0;

    // 获取机器人的极限机械射速限制 (原有逻辑保留，作为速度上限)
    float max_mech_speed;
    if(shoot_cmd_recv.aim_flag == 1) {
        max_mech_speed = (shoot_cmd_recv.robot_level <= 5) ? 40.0f : 45.0f;
    } else {
        max_mech_speed = (shoot_cmd_recv.robot_level <= 1) ? 20.0f : 25.0f;
    }

    if(shoot_cmd_recv.heat_mode == HEAT_OPEN && shoot_cmd_recv.loader_mode == LOAD_BURSTFIRE)
    {
        // a: 当前裁判系统给定的冷却速率 (每秒下降的热量)
        float a = shoot_cmd_recv.heat_cool;
        // m: 当前可用的热量缓冲容量
        float m = shoot_cmd_recv.heat_limit - shoot_cmd_recv.heat;
        // d: 每发子弹产生的热量
        float d = HEAT_PER_BULLET;

        // 极限安全保护：如果余量已经小于一发子弹的热量，直接停火
        if (m < d) {
            shoot_speed = 0.0f;
            is_bursting = 0;
        }
        else
        {
            // 扳机刚刚按下，或者刚刚切入开火模式，计算爆发策略
            if (!is_bursting) {
                is_bursting = 1;
                burst_start_time = DWT_GetTimeline_ms();

                // 估算爆发持续时间 (单位: 毫秒)
                // 原公式 (m + 2a)*10 是一种经验公式，为了抹除低等级和高等级的射击手感差异
                planned_burst_duration = (uint32_t)((m + 2.0f * a) * 10.0f);

                // 限制最长和最短爆发时间，防止数据异常
                if(planned_burst_duration > 3000) planned_burst_duration = 3000;
                if(planned_burst_duration < 100) planned_burst_duration = 100;

                // 计算爆发期应该采用的恒定射速 (发/秒)
                float duration_factor = planned_burst_duration / 100.0f;
                if (m < 100.0f) {
                    planned_burst_speed = (10.0f * m - a - 3.0f * d) / (d * duration_factor) + a / d;
                } else {
                    planned_burst_speed = (10.0f * m - a - 5.0f * d) / (d * duration_factor) + a / d;
                }

                // 限制计算出的爆发射速，不能超过机械极限，也不能低于稳态冷却速率
                if (planned_burst_speed > max_mech_speed) planned_burst_speed = max_mech_speed;
                if (planned_burst_speed < (a / d)) planned_burst_speed = (a / d);
            }

            // --- 执行射击策略 ---
            uint32_t current_time = DWT_GetTimeline_ms();

            if (current_time - burst_start_time < planned_burst_duration) {
                // 1. 处于爆发期：按计算好的较高射速输出
                shoot_speed = planned_burst_speed;
            } else {
                // 2. 爆发期结束：无缝转入稳态冷却期，射速严格等于冷却速率 (绝对不会超热量)
                shoot_speed = a / d;
            }
        }
    }
    else
    {
        // 未开启热控，或不处于连发模式（如单发/停止），重置状态机
        is_bursting = 0;
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
        break;
        // 单发模式,根据鼠标按下的时间,触发一次之后需要进入不响应输入的状态(否则按下的时间内可能多次进入,导致多次发射)
    case LOAD_1_BULLET:                                                                     // 激活能量机关/干扰对方用,英雄用.
        DJIMotorOuterLoop(loader, ANGLE_LOOP);                              // 切换到角度环
        DJIMotorSetRef(loader, loader->measure.total_angle + ONE_BULLET_DELTA_ANGLE); // 控制量增加一发弹丸的角度
        hibernate_time = DWT_GetTimeline_ms();                                              // 记录触发指令的时间
        dead_time = 150;// 完成1发弹丸发射的时间
        if(shoot_cmd_recv.heat > 0 && shoot_cmd_recv.heat <= 5)
        {
            DJIMotorOuterLoop(loader, SPEED_LOOP); // 切换到速度环
            DJIMotorSetRef(loader, 0);             // 同时设定参考值为0,这样停止的速度最快
        }
        // DJIMotorSetRef(loader, 0); // 控制量增加一发弹丸的角度

        break;
        // 三连发,如果不需要后续可能删除
    case LOAD_3_BULLET:
        DJIMotorOuterLoop(loader, ANGLE_LOOP);                                     // 切换到角度环
        DJIMotorSetRef(loader, loader->measure.total_angle + 3 * ONE_BULLET_DELTA_ANGLE); // 增加3发
        hibernate_time = DWT_GetTimeline_ms();                                                  // 记录触发指令的时间
        dead_time = 1000;                                                                       // 完成3发弹丸发射的时间
        break;
        // 连发模式,对速度闭环,射频后续修改为可变,目前固定为1Hz
        case LOAD_BURSTFIRE:
            DJIMotorOuterLoop(loader, SPEED_LOOP);
        DJIMotorSetRef(loader, 20 * 360 * REDUCTION_RATIO_LOADER / 9);
        // DJIMotorSetRef(loader, 20 * 360 * REDUCTION_RATIO_LOADER / 9);
        // x颗/秒换算成速度: 已知一圈的载弹量,由此计算出1s需要转的角度,注意换算角速度(DJIMotor的速度单位是angle per second)
        break;
        // 拨盘反转,对速度闭环,后续增加卡弹检测(通过裁判系统剩余热量反馈和电机电流)
        // 也有可能需要从switch-case中独立出来
        case LOAD_REVERSE:
            DJIMotorOuterLoop(loader, SPEED_LOOP);
        DJIMotorSetRef(loader, -5 * 360 * REDUCTION_RATIO_LOADER / 9);             // 固定速度反转
        hibernate_time = DWT_GetTimeline_ms();                                              // 记录触发指令的时间
        dead_time = 200;                                                                    // 翻转500ms
        shoot_feedback_data.stalled_flag = 0 ; //清除堵转标志
        // ...
        break;
    default:
        while (1)
            ; // 未知模式,停止运行,检查指针越界,内存溢出等问题
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
        DJIMotorSetRef(friction_l, 43000);
        DJIMotorSetRef(friction_r, 43000);
                       // break;
        //        }
    }
    else // 关闭摩擦轮
    {
        DJIMotorSetRef(friction_l, 0);
        DJIMotorSetRef(friction_r, 0);
    }

    // // 开关热控
    // if(shoot_cmd_recv.heat_mode == HEAT_OPEN)
    // {
    //     if(shoot_cmd_recv.heat>=(0.85 * heat_limit))
    //     {
    //         DJIMotorStop(loader);
    //     }
    //     else
    //     {
    //         DJIMotorEnable(loader);
    //     }
    // }
    // else if(shoot_cmd_recv.heat_mode == HEAT_CLOSE) {
    //     DJIMotorEnable(loader);
    // }
    // //卡弹检测
    // float shoot_rate = get_shoot_rate(); // 从控制系统获取射速
   // if(heat_detector.now_heat > 0.95 * shoot_cmd_recv.heat_limit)
   // {
   //     DJIMotorStop(loader);
   //     shoot_feedback_data.over1 = 1;
   // }
   // else
   // {
   //     shoot_feedback_data.over1 = 0;
   // }
   //  if(heat_detector.now_heat > 0.92 * shoot_cmd_recv.heat_limit)
   //  {
   //      DJIMotorStop(loader);
   //      shoot_feedback_data.over2 = 1;
   //  }
   //  else
   //  {
   //      shoot_feedback_data.over2 = 0;
   //  }
   //  if(heat_detector.now_heat > 0.9 * shoot_cmd_recv.heat_limit)
   //  {
   //      // DJIMotorStop(loader);
   //      shoot_feedback_data.over3 = 1;
   //  }
   //  else
   //  {
   //      shoot_feedback_data.over3 = 0;
   //  }
   // // if(shoot_cmd_recv.shoot_rate == 15)
   // // {
   // //     shoot_feedback_data.over = 0;
   // // }
   //  if(heat_detector.now_heat > 0.96 * shoot_cmd_recv.heat_limit)
   //  {
   //      DJIMotorStop(loader);
   //  }
        stalled_detect();
   //  Trigger_Update();
    // // // 热量控制
    // if(First_flag==100){
    //     //初始化角度
    //     init_totalangle=loader->measure.init_totalangle;
    //     First_flag=0;
    // }
    // if(First_flag<100)
    //     First_flag++;
    // // Cal_bullet_heat();
    // 反馈数据,目前暂时没有要设定的反馈数据,后续可能增加应用离线监测以及卡弹反馈
    PubPushMessage(shoot_pub, (void *)&shoot_feedback_data);
}