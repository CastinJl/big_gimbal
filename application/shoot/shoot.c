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
// 热量检测实例（带初始校准阈值）
static float current_q_buf[CURRENT_QUEUE_SIZE];
static HeatDetector heat_detector = {
        .state = ST_CLOSE,
        .q_threshold = 26.0f
};
static void Queue_Init(CurrentQueue* q, float* buf, int size) {
    q->buf = buf;
    q->cap = size;
    q->front = q->size = 0;
    q->rear = -1;
}

// 元素入队
static void Queue_Push(CurrentQueue* q, float val) {
    if (q->size >= q->cap) return;
    q->rear = (q->rear + 1) % q->cap;
    q->buf[q->rear] = val;
    q->size++;
}

// 元素出队
static void Queue_Pop(CurrentQueue* q) {
    if (q->size <= 0) return;
    q->front = (q->front + 1) % q->cap;
    q->size--;
}

// 获取队首元素
static float Queue_Front(CurrentQueue* q) {
    return q->size ? q->buf[q->front] : 0.0f;
}

// 清空队列
static void Queue_Clear(CurrentQueue* q) {
    q->front = q->size = 0;
    q->rear = -1;
}
void HeatControl_Init() {
    heat_control.heat_per_bullet = 10.0f;
    heat_control.current_heat = 0;
    heat_control.last_heat_time = DWT_GetTimeline_ms();
    heat_control.overheat_flag = 0;
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
    Queue_Init(&heat_detector.current_q, current_q_buf, CURRENT_QUEUE_SIZE);
    HeatControl_Init();
    // Servo_Init_Config_s lid_config = {
    //         .htim = &htim1,
    //         .Channel = TIM_CHANNEL_1,
    //         .Servo_type = Servo180,
    //         .Servo_Angle_Type = Free_Angle_mode,
    // };
    //
    // lid = ServoInit(&lid_config);

    //    SPI_Init_Config_s rgb_config = {
    //           .spi_handle = ;
    //           .GPIOx = GPIOx;
    //           .cs_pin = cs_pin;
    //           .spi_work_mode = spi_work_mode;
    //           .callback = callback;
    //           .id = 1;
    //    }
}


static void HeatDetector_Update(HeatDetector* hd,
                                DJIMotorInstance* left,
                                DJIMotorInstance* right,
                                float cooling_rate)
{
    switch (hd->state) {
    case ST_CLOSE:
        // 开机条件：双电机使能+转速达标
        if (left->stop_flag == MOTOR_ENALBED &&
            right->stop_flag == MOTOR_ENALBED &&
            left->motor_controller.pid_ref > 0 &&
            right->motor_controller.pid_ref > 0 &&
            fabsf(left->measure.speed_aps) >= 0.95f * fabsf(left->motor_controller.pid_ref) &&
            fabsf(right->measure.speed_aps) >= 0.95f * fabsf(right->motor_controller.pid_ref))
        {
            hd->state = ST_OPEN;
            Queue_Clear(&hd->current_q);
            hd->q_sum = 0;
            hd->last_q_sum = 0;  // 状态切换时重置历史值
        }
        break;

    case ST_OPEN: {
        /* 关键修改点1：先保存上次队列总和 */
        float previous_sum = hd->last_q_sum;  // 保存进入时的历史值

        /* 关键修改点2：在队列操作前更新last_q_sum */
        hd->last_q_sum = hd->q_sum;  // 当前值将成为下次的历史值

        // 1. 计算电流差值并更新队列
        float current_diff = left->measure.real_current * 20 / 16384 - right->measure.real_current * 20 / 16384;
        Queue_Push(&hd->current_q, current_diff);
        hd->q_sum += current_diff;

        // 2. 维护队列长度
        if (hd->current_q.size > CURRENT_QUEUE_SIZE) {
            hd->q_sum -= Queue_Front(&hd->current_q);
            Queue_Pop(&hd->current_q);

            /* 关键修改点3：调整历史值避免负误差 */
            // previous_sum = (previous_sum > removed) ? (previous_sum - removed) : 0;
        }

        // 3. 子弹发射检测（核心逻辑）
        if (hd->q_sum > hd->q_threshold) {
            Queue_Clear(&hd->current_q);
            hd->q_sum = 0;
            hd->now_heat += HEAT_PER_BULLET;
            hd->total_ammo++;

            /* 可选：动态阈值调整 */
            // hd->q_threshold *= 1.05f;  // 适当提高阈值避免连发误判
        }

        // 4. 热量冷却
        hd->now_heat = fmaxf(hd->now_heat - cooling_rate * COOLING_RATE_SCALE, 0);

        // 5. 关机条件
        if ((left->stop_flag == MOTOR_STOP && right->stop_flag == MOTOR_STOP) ||
            (fabsf(left->motor_controller.pid_ref) < 1e-6 &&
             fabsf(right->motor_controller.pid_ref) < 1e-6))
        {
            hd->state = ST_CLOSE;
            hd->now_heat = 0;
            hd->last_q_sum = 0;  // 状态切换时重置
        }
        break;
    }
    } // end switch
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
void HeatControl_Update() {
    float current_time = DWT_GetTimeline_ms();
    float delta_time = (current_time - heat_control.last_heat_time) / 1000.0f; // 转换为秒
    heat_control.current_heat = heat_detector.now_heat;
    // 热量冷却
    heat_control.current_heat = fmaxf(heat_control.current_heat - heat_control.cooling_rate * delta_time, 0);

    // 更新射速限制
    if (heat_control.current_heat > 0.9f * heat_control.heat_limit) {
        heat_control.overheat_flag = 1;
        // 超热时限制射速
        heat_control.shoot_rate = heat_control.cooling_rate / heat_control.heat_per_bullet;
    } else {
        heat_control.overheat_flag = 0;
    }

    heat_control.last_heat_time = current_time;
}



//热量控制
void Heat_control()
{
        // switch (shoot_cmd_recv.robot_level)
        // {
        // case 1:
        //     heat_limit = 150;
        //     break;
        // case 2:
        //     heat_limit = 200;
        //     break;
        // case 3:
        //     heat_limit = 250;
        //     break;
        // case 4:
        //     heat_limit = 350;
        //     break;
        // case 5:
        //     heat_limit = 400;
        //     break;
        // case 6:
        //     heat_limit = 450;
        //     break;
        // case 7:
        //     heat_limit = 500;
        //     break;
        // case 8:
        //     heat_limit = 550;
        //     break;
        // case 9:
        //     heat_limit = 600;
        //     break;
        // case 10:
        //     heat_limit = 650;
        //     break;
        // }
    switch (shoot_cmd_recv.robot_level)
    {
    case 1:
        // shoot_cmd_recv.shoot_rate = 30;
        heat_limit = 30;
        break;
    case 2:
        heat_limit = 65;
        // shoot_cmd_recv.shoot_rate = 30;
        break;
    case 3:
        heat_limit = 95;
        break;
    case 4:
        heat_limit = 155;
        break;
    case 5:
        heat_limit = 190;
        break;
    case 6:
        heat_limit = 225;
        break;
    case 7:
        heat_limit = 260;
        break;
    case 8:
        heat_limit = 295;
        break;
    case 9:
        heat_limit = 330;
        break;
    case 10:
        heat_limit = 400;
        break;
    }
    if(shoot_cmd_recv.heat_mode == HEAT_OPEN)
    {
        if(shoot_cmd_recv.heat>=(0.8 * heat_limit))
        {
            DJIMotorStop(loader);
        }
        else
        {
            DJIMotorEnable(loader);
        }
    }
    else if(shoot_cmd_recv.heat_mode == HEAT_CLOSE)
    {
        DJIMotorEnable(loader);
    }

    // if(shoot_cmd_recv.heat>=(0.9*heat_limit))
   // {
   //     DJIMotorStop(loader);
   // }
   // else
   // {
   //     DJIMotorEnable(loader);
   // }
}

/* 机器人发射机构控制核心任务 */
void ShootTask()
{
    //从cmd获取控制数据
    SubGetMessage(shoot_sub, &shoot_cmd_recv);
    HeatDetector_Update(&heat_detector, friction_l, friction_r, shoot_cmd_recv.heat_cool);
    heat_control.heat_limit = shoot_cmd_recv.heat_limit;
    heat_control.cooling_rate = shoot_cmd_recv.heat_cool;
    HeatControl_Update();
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
    float jc_speed;
    // if(shoot_cmd_recv.aim_flag == 1)
    // {
    //     shoot_speed = 45.0f;
    // }
    // else
    // {
    //     shoot_speed = 25.0f;
    // }
        // if(shoot_cmd_recv.aim_flag == 1)
        // {}
    //     if(shoot_cmd_recv.robot_level <= 5)
    //     {
    //         jc_speed = 40.0f;
    //     }
    //     else if(shoot_cmd_recv.robot_level > 5 && shoot_cmd_recv.robot_level <= 10)
    //     {
    //         jc_speed = 45.0f;
    //     }
    // }
    // else
    // {
    //     if(shoot_cmd_recv.robot_level <= 1)
    //     {
    //         jc_speed = 20.0f;
    //     }
    //     else if(shoot_cmd_recv.robot_level > 1 && shoot_cmd_recv.robot_level <= 10)
    //     {
    //         jc_speed = 25.0f;
    //     }
    // }
    // if(shoot_cmd_recv.heat_mode == HEAT_OPEN)
    // {
    //     float tep_delta = (float)shoot_cmd_recv.heat_limit - shoot_cmd_recv.heat;
    //     if(tep_delta >= Heat_Limit_Slowdown_Threshold)
    //     {
    //         shoot_speed = jc_speed;
    //     }
    //     else if(tep_delta >= Heat_Limit_Ceasefire_Threshold && tep_delta < Heat_Limit_Slowdown_Threshold)
    //     {
    //         shoot_speed = (jc_speed * (Heat_Limit_Ceasefire_Threshold - tep_delta) + shoot_cmd_recv.heat_cool / 10.0f * (tep_delta - Heat_Limit_Slowdown_Threshold)) / (Heat_Limit_Ceasefire_Threshold - Heat_Limit_Slowdown_Threshold);
    //     }
    //     else if(tep_delta < Heat_Limit_Ceasefire_Threshold)
    //     {
    //         shoot_speed = 0.0f;
    //     }
    // }
    // else
    // {
    //     shoot_speed = jc_speed;
    // }


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
        DJIMotorSetRef(loader, loader->measure.total_angle + 3600); // 控制量增加一发弹丸的角度
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
        DJIMotorSetRef(loader, loader->measure.total_angle - 3 * ONE_BULLET_DELTA_ANGLE); // 增加3发
        hibernate_time = DWT_GetTimeline_ms();                                                  // 记录触发指令的时间
        dead_time = 1000;                                                                       // 完成3发弹丸发射的时间
        break;
        // 连发模式,对速度闭环,射频后续修改为可变,目前固定为1Hz
        case LOAD_BURSTFIRE:
            DJIMotorOuterLoop(loader, SPEED_LOOP);
        //DJIMotorSetRef(loader, -shoot_speed * 360 * REDUCTION_RATIO_LOADER / 8);
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