#include "shoot.h"
#include "robot_def.h"

#include "dji_motor.h"
#include "message_center.h"
#include "bsp_dwt.h"
#include "general_def.h"
#include "servo_motor.h"
#include "master_process.h"

#define HEAT_PER_BULLET        10.0f  // 每发子弹热量增量

// TODO: 根据实测的摩擦轮稳态电流和弹丸脉冲统计值调整这些默认参数。
#define BULLET_CURRENT_RISE_DELTA_DEFAULT     1100.0f // 相对基准上升超过该值，认为弹丸开始经过摩擦轮
#define BULLET_CURRENT_RECOVER_DELTA_DEFAULT   400.0f // 回落至基准值加该余量以内，认为弹丸离开摩擦轮
#define BULLET_RISE_CONFIRM_MS_DEFAULT           6.0f // 电流上升的最短确认时间
#define BULLET_RECOVER_CONFIRM_MS_DEFAULT        6.0f // 电流恢复的最短确认时间
#define BULLET_DETECT_TIMEOUT_MS_DEFAULT        500.0f//超时检测。超过这个时间还没有下降，认为出现卡弹或者其他故障
#define BULLET_BURST_EXIT_COMPENSATION_MS_DEFAULT 0.0f // 连发：电流上升沿到枪口的实测补偿

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
static Bullet_Detect_State_s bullet_detect;
static float fire_delay_ms;

//保护函数
static void BulletDetectReset(void)
{
    bullet_detect.state = BULLET_DETECT_IDLE;
    bullet_detect.rise_candidate_time_ms = 0.0f;
    bullet_detect.recover_candidate_time_ms = 0.0f;
}
static void BulletDetectUpdate(void) {
    static uint8_t trigger_latched;
    const float now_ms = DWT_GetTimeline_ms();

    // 取左右轮电流的平均值
    const float speed = (fabsf((float)friction_l->measure.speed_rad_per_second) +
                               fabsf((float)friction_r->measure.speed_rad_per_second)) * 0.5f;

    // 1. 上升沿触发检测启动
    if (shoot_cmd_recv.vision_fire_trigger && !trigger_latched &&
        shoot_cmd_recv.friction_mode == FRICTION_ON &&
        (shoot_cmd_recv.loader_mode == LOAD_1_BULLET ||
         shoot_cmd_recv.loader_mode == LOAD_BURSTFIRE ||
         shoot_cmd_recv.loader_mode == LOAD_2_BULLET)) {

        bullet_detect.state = BULLET_DETECT_WAIT_RISE;
        bullet_detect.loader_mode = shoot_cmd_recv.loader_mode;
        bullet_detect.trigger_time_ms = now_ms;
        bullet_detect.rise_candidate_time_ms = 0.0f;
        bullet_detect.baseline_speed = speed;
    }
    trigger_latched = shoot_cmd_recv.vision_fire_trigger;

    // 空闲状态直接退出
    if (bullet_detect.state == BULLET_DETECT_IDLE) {
        return;
    }

    // 2. 超时与异常保护
    if ((now_ms - bullet_detect.trigger_time_ms > BULLET_DETECT_TIMEOUT_MS_DEFAULT) ||
        (shoot_cmd_recv.friction_mode != FRICTION_ON)) {
        BulletDetectReset();
        return;
    }

    const float speed_delta = speed - bullet_detect.baseline_speed;

    // 3. 上升沿确认逻辑（修复状态机）
    if (bullet_detect.state == BULLET_DETECT_WAIT_RISE) {
        if (speed <= 670-25) {
            // 首次越过阈值，记录候选时间
            if (bullet_detect.rise_candidate_time_ms == 0.0f) {
                bullet_detect.rise_candidate_time_ms = now_ms;
            }
            // 持续超过阈值达到确认时间，判定发弹成功
            if (now_ms - bullet_detect.rise_candidate_time_ms >= BULLET_RISE_CONFIRM_MS_DEFAULT) {
                float measured_delay = bullet_detect.rise_candidate_time_ms -
                                      bullet_detect.trigger_time_ms +
                                      BULLET_BURST_EXIT_COMPENSATION_MS_DEFAULT;

                // 建议加入合理性范围校验（如 10ms ~ 150ms 之间才有效）
                if (measured_delay > 5.0f && measured_delay < 200.0f) {
                    // 指数平滑滤波更新
                    fire_delay_ms = fire_delay_ms * 0.7f + measured_delay * 0.3f;
                    VisionSetFireDelay(fire_delay_ms);
                }

                BulletDetectReset();
            }
        } else {
            // 中途掉下阈值则重置计时（滤波防抖）
            bullet_detect.rise_candidate_time_ms = 0.0f;
        }
    }
}



void ShootInit()
{
    // 左摩擦轮
    Motor_Init_Config_s friction_config = {
        .can_init_config = {
            .can_handle = &hcan2,
    },
    .controller_param_init_config = {
            .speed_PID = {
                .Kp = 86.0f , // 原1.5按度/s调, 现rad/s
                .Ki = 6.0f ,
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
            .loop_type = SPEED_LOOP,
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
                            .Kp = 860.0f , // 原15按度调, 现rad
                            .Ki = 0,
                            .Kd = 0,
                            .MaxOut = 10000,
                    },
                    .speed_PID = {
                            .Kp = 200.5f, // 原3.5按度/s调, 现rad/s
                            .Ki = 4.0f,
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
                    .loop_type = SPEED_LOOP, // 初始化成SPEED_LOOP,让拨盘停在原地,防止拨盘上电时乱转
                    .motor_reverse_flag = MOTOR_DIRECTION_NORMAL, // 注意方向设置为拨盘的拨出的击发方向
                    //.feedback_reverse_flag = FEEDBACK_DIRECTION_REVERSE,
            },
            .motor_type = M3508 // 步兵使用M3508
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
        if(fabsf(loader->measure.speed_rad_per_second)<=500.0f * DEGREE_2_RAD) // speed_aps 已 rad/s, 阈值500°=8.73rad/s
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
    speed_loader = loader->measure.speed_rad_per_second;
    BulletDetectUpdate();

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

        // 稳态机械补偿
        // 理论上稳态射速 = a/d (比如 40/10 = 4)。
        // 但因为摩擦轮阻力、拨盘空腔，你给电机设定 4发/秒，它可能实际只打出 2.5发。
        // 所以我们需要给稳态速度乘一个补偿系数，这个系数根据你这台车的机械阻力来定。
        float compensate_factor = 1.0f; // 如果稳态热量掉，就把这个往上加(1.4, 1.5)；如果稳态热量超了，就减小(1.1)
        float target_steady_speed = (a / d) * compensate_factor;

        // 计算 10Hz 延迟的安全余量
        // 裁判系统 100ms(0.1s) 刷新一次。在这 0.1s 内，以最高射速会打出多少热量？
        // max_mech_speed * 0.1s * d。例如：25 * 0.1 * 10 = 25 热量。
        // 我们再加上 5 作为容错底线。
        float safe_buffer = (max_mech_speed * 0.1f * d) + 5.0f;

        // 减速缓冲带：距离安全底线还有 30 热量时，开始线性收转速
        float decelerate_buffer = safe_buffer + 30.0f;

        // 状态机判断输出
        if (m < d) {
            // 极限保护：连1发子弹的热量都不够了，必须停！
            shoot_speed = 0.0f;
        }
        else if (m > decelerate_buffer) {
            // 爆发期：距离上限还很远，输出最高射速
            shoot_speed = max_mech_speed;
        }
        else if (m > safe_buffer) {
            // 减速期：距离上限近了，为了防止 100ms 延迟导致超标，平滑降低射速
            // 把剩余热量在 [safe_buffer, decelerate_buffer] 映射到 [target_steady_speed, max_mech_speed]
            float ratio = (m - safe_buffer) / (decelerate_buffer - safe_buffer);
            shoot_speed = target_steady_speed + ratio * (max_mech_speed - target_steady_speed);
        }
        else {
            // 稳态期：已经贴近边缘，输出带补偿的冷却速率
            shoot_speed = target_steady_speed;
        }
    }
    else
    {
        shoot_speed = max_mech_speed;
    }


    // 单发模式主要提供给能量机关激活使用(以及英雄的射击大部分处于单发)
    if (hibernate_time + dead_time > DWT_GetTimeline_ms())
        return;
    static uint8_t single_shot_lock = 0;
    float last_friction_speed = friction_l->measure.speed_rad_per_second;
    // 若不在休眠状态,根据robotCMD传来的控制模式进行拨盘电机参考值设定和模式切换
    switch (shoot_cmd_recv.loader_mode)
    {
        // 停止拨盘
    case LOAD_STOP:
        DJIMotorLoopType(loader, SPEED_LOOP); // 切换到速度环
        DJIMotorSetRef(loader, 0);             // 同时设定参考值为0,这样停止的速度最快
        single_shot_lock = 0;
        break;
        // 单发模式,根据鼠标按下的时间,触发一次之后需要进入不响应输入的状态(否则按下的时间内可能多次进入,导致多次发射)
    case LOAD_1_BULLET:           // 激活能量机关/干扰对方用,英雄用
        if (single_shot_lock == 0)
        {
         //   DJIMotorLoopType(loader, ANGLE_LOOP);                              // 切换到角度环
          //  DJIMotorSetRef(loader, loader->measure.total_angle + ONE_BULLET_DELTA_ANGLE * DEGREE_2_RAD * REDUCTION_RATIO_LOADER); // 控制量增加一发弹丸的角度(total_angle已rad)
            DJIMotorLoopType(loader, SPEED_LOOP);
            DJIMotorSetRef(loader, 60);
         //   hibernate_time = DWT_GetTimeline_ms();                                              // 记录触发指令的时间
          //  dead_time = 150;// 完成1发弹丸发射的时间
          if (friction_l->measure.speed_rad_per_second  < 670-25) {
              single_shot_lock = 1;
              DJIMotorSetRef(loader,0);
          } //执行一次后立刻上锁
        }
        break;
        // 二连发
    case LOAD_2_BULLET:
        if (single_shot_lock == 0)
        {
            DJIMotorLoopType(loader, ANGLE_SPEED_CASCADE);                              // 切换到角度环
            DJIMotorSetRef(loader, loader->measure.total_angle + 2 * ONE_BULLET_DELTA_ANGLE * DEGREE_2_RAD * REDUCTION_RATIO_LOADER); // 控制量增加一发弹丸的角度(total_angle已rad)
            hibernate_time = DWT_GetTimeline_ms();                                              // 记录触发指令的时间
            dead_time = 300;// 完成1发弹丸发射的时间
            single_shot_lock = 1;  //执行一次后立刻上锁
        }                                                                   // 完成3发弹丸发射的时间
        break;
        // 连发,对速度闭环,射频后续修改为可变,目前固定为1Hz
        case LOAD_BURSTFIRE:
            single_shot_lock = 0;                  // 切入连发时，为了安全起见也清零锁
            DJIMotorLoopType(loader, SPEED_LOOP);
            DJIMotorSetRef(loader, shoot_speed * 2.0f * PI * REDUCTION_RATIO_LOADER / 9);
        // x颗/秒换算成速度: 已知一圈的载弹量,由此计算出1s需要转的角度(DJIMotor的速度单位是rad/s)
        break;

        case LOAD_REVERSE:
            single_shot_lock = 0;                  // 切入连发时，为了安全起见也清零锁
            DJIMotorLoopType(loader, SPEED_LOOP);
        DJIMotorSetRef(loader, -5 * 2.0f * PI * REDUCTION_RATIO_LOADER / 9);             // 固定速度反转(rad/s)
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
        DJIMotorSetRef(friction_l, 670); //    rad/s
        DJIMotorSetRef(friction_r, 670);
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
