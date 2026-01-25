#include "dmmotor.h"
#include "memory.h"
#include "general_def.h"
#include "user_lib.h"
#include "cmsis_os.h"
#include "string.h"
#include "daemon.h"
#include "stdlib.h"
#include "bsp_log.h"

static uint8_t idx;
static DMMotorInstance *dm_motor_instance[DM_MOTOR_CNT];
static osThreadId dm_task_handle[DM_MOTOR_CNT];
/* 两个用于将uint值和float值进行映射的函数,在设定发送值和解析反馈值时使用 */


void DMMotorSetMode(DMMotor_Mode_e cmd, DMMotorInstance *motor)
{
    memset(motor->motor_can_instance->tx_buff, 0xff, 7);  // 发送电机指令的时候前面7bytes都是0xff
    motor->motor_can_instance->tx_buff[7] = (uint8_t)cmd; // 最后一位是命令id
    CANTransmit(motor->motor_can_instance, 5);
}

static void DMMotorDecode(CANInstance *motor_can)
{
    uint16_t tmp; // 用于暂存解析值,稍后转换成float数据,避免多次创建临时变量
    uint8_t *rxbuff = motor_can->rx_buff;
    DMMotorInstance *motor = (DMMotorInstance *)motor_can->id;
    DM_Motor_Measure_s *measure = &(motor->measure); // 将can实例中保存的id转换成电机实例的指针

    //    DaemonReload(motor->motor_daemon);
    //0 失能  1 使能  8 超压 9 欠压  A 过流  B MOS过压  C 电机线圈过温   D 通信丢失  E 过载
    tmp =  (uint8_t)(rxbuff[0]>>4);
    measure->state = tmp;
    if(measure->state==1){
        DaemonReload(motor->motor_daemon);//重装看门狗
        motor->lost_flag=0;//电机找回
        motor->deltatime=DWT_GetDeltaT(&motor->deltatimeline);
    }

    measure->last_position = measure->position;

    tmp = (uint16_t)((rxbuff[1] << 8) | rxbuff[2]);
    // measure->position = uint_to_float(tmp, DM_P_MIN, DM_P_MAX, 16);
    measure->position = uint_to_float(tmp, -motor->DM_P_MAX, motor->DM_P_MAX, 16);

    //measure->angle_single_round =  RAD_2_DEGREE * (measure->position+3.141593);
    measure->angle_single_round =  (measure->position);

    tmp = (uint16_t)((rxbuff[3] << 4) | rxbuff[4] >> 4);
    // measure->velocity = uint_to_float(tmp, DM_V_MIN, DM_V_MAX, 12);
    measure->velocity = uint_to_float(tmp, -motor->DM_V_MAX, motor->DM_V_MAX, 12);

    tmp = (uint16_t)(((rxbuff[4] & 0x0f) << 8) | rxbuff[5]);
    // measure->torque = uint_to_float(tmp, DM_T_MIN, DM_T_MAX, 12);
    measure->torque = uint_to_float(tmp, -motor->DM_T_MAX, motor->DM_T_MAX, 12);

    measure->T_Mos = (float)rxbuff[6];
    measure->T_Rotor = (float)rxbuff[7];

    // 多圈角度计算,前提是假设两次采样间电机转过的角度小于180°,自己画个图就清楚计算过程了
    if (measure->position - measure->last_position > PI) //3.1425926
        measure->total_round--;
    else if (measure->position - measure->last_position < -PI)
        measure->total_round++;

    //measure->total_angle = measure->total_round * (DM_P_MAX * DEGREE_2_RAD) + measure->angle_single_round;
    measure->total_angle = measure->total_round * (motor->DM_P_MAX*2) + measure->angle_single_round;
}

static void DMMotorLostCallback(void *motor_ptr)
{
    DMMotorInstance *motor = (DMMotorInstance *)motor_ptr;
    // uint16_t can_bus = motor->motor_can_instance->can_handle == &hcan1 ? 1 : 2;
    // LOGWARNING("[dm_motor] Motor lost, can bus [%d] , id [%d]", can_bus, motor->motor_can_instance->tx_id);
    motor->lost_flag=1;
    // DMMotorEnable(motor);
    // DMMotorSetMode(DM_CMD_MOTOR_MODE, motor);
    // LOGWARNING("[dm_motor] Trying to restart motor, can bus [%d] , id [%d]\"", can_bus, motor->motor_can_instance->tx_id);
    LOGWARNING("[dm_motor] Trying to restart motor, id [%d]\"", motor->motor_can_instance->tx_id);

}
void DMMotorCaliEncoder(DMMotorInstance *motor)
{
    DMMotorSetMode(DM_CMD_RESET_MODE, motor);
    DWT_Delay(0.1);

    DMMotorSetMode(DM_CMD_ZERO_POSITION, motor);
    DWT_Delay(0.1);

    DMMotorSetMode(DM_CMD_MOTOR_MODE, motor);
    DWT_Delay(0.1);
}
DMMotorInstance *DMMotorInit(Motor_Init_Config_s *config)
{
    DMMotorInstance *motor = (DMMotorInstance *)malloc(sizeof(DMMotorInstance));
    memset(motor, 0, sizeof(DMMotorInstance));

    config->can_init_config.rx_id = config->can_init_config.rx_id;
    config->can_init_config.tx_id = config->can_init_config.tx_id;
    motor->motor_settings = config->controller_setting_init_config;
    PIDInit(&motor->motor_controller.current_PID, &config->controller_param_init_config.current_PID);
    PIDInit(&motor->motor_controller.speed_PID, &config->controller_param_init_config.speed_PID);
    PIDInit(&motor->motor_controller.angle_PID, &config->controller_param_init_config.angle_PID);
    motor->motor_controller.other_angle_feedback_ptr = config->controller_param_init_config.other_angle_feedback_ptr;
    motor->motor_controller.other_speed_feedback_ptr = config->controller_param_init_config.other_speed_feedback_ptr;

    motor->motor_controller.current_feedforward_ptr = config->controller_param_init_config.current_feedforward_ptr;
    motor->motor_controller.speed_feedforward_ptr = config->controller_param_init_config.speed_feedforward_ptr;
    motor->DM_P_MAX = config->DM_P_MAX;
    motor->DM_V_MAX = config->DM_V_MAX;
    motor->DM_T_MAX = config->DM_T_MAX;
    config->can_init_config.can_module_callback = DMMotorDecode;
    config->can_init_config.id = motor;
    motor->motor_can_instance = CANRegister(&config->can_init_config);

    Daemon_Init_Config_s conf = {
            .callback = DMMotorLostCallback,
            .owner_id = motor,
            .reload_count = 10,
    };
    motor->motor_daemon = DaemonRegister(&conf);
    motor->lost_flag=1;//初始设定电机丢失，保证先使能
    DMMotorEnable(motor);
    DMMotorSetMode(DM_CMD_MOTOR_MODE, motor);
    //DWT_Delay(0.1f);
    dm_motor_instance[idx++] = motor;
    return motor;
}

void DMMotorSetRef(DMMotorInstance *motor, float ref)
{
    motor->motor_controller.pid_ref = ref;
}

void DMMotorEnable(DMMotorInstance *motor)
{
    motor->stop_flag = MOTOR_ENALBED;
}

void DMMotorStop(DMMotorInstance *motor)//不使用使能模式是因为需要收到反馈
{
    motor->stop_flag = MOTOR_STOP;
}

void DMMotorOuterLoop(DMMotorInstance *motor, Closeloop_Type_e type)
{
    motor->motor_settings.outer_loop_type = type;
}


void DMMotorControl(){
    DMMotorInstance *motor;
    Motor_Controller_s *motor_controller;   // 电机控制器
    DM_Motor_Measure_s *measure;
    Motor_Control_Setting_s *setting;
    //CANInstance *motor_can = motor->motor_can_instance;
    //uint16_t tmp;
    DMMotor_Send_s motor_send_mailbox;
    float  pid_ref, set, pid_measure;
    static uint16_t reload_cnt = 0; // 新增计数器

    for (size_t i = 0; i < idx; ++i)
    {
        motor = dm_motor_instance[i];
        setting = &motor->motor_settings;
        motor_controller = &motor->motor_controller;
        measure = &motor->measure;

        if (motor->lost_flag==1) {
            //说明触发看门狗了，不再发送消息，仅仅重新发送使能指令
            reload_cnt++;
            // 假设控制频率是1kHz，这里设置为每500ms(500次)尝试重启一次
            // 避免高频占用CAN邮箱
            if(reload_cnt > 500) {
                DMMotorSetMode(DM_CMD_MOTOR_MODE, motor);
                reload_cnt = 0;
            }
            continue;
        }
        pid_ref = motor->motor_controller.pid_ref;//保存设定值

        if (setting->motor_reverse_flag == MOTOR_DIRECTION_REVERSE)
            pid_ref *= -1;

        // pid_ref会顺次通过被启用的闭环充当数据的载体
        // 计算位置环,只有启用位置环且外层闭环为位置时会计算速度环输出
        if ((setting->close_loop_type & ANGLE_LOOP) && setting->outer_loop_type == ANGLE_LOOP)
        {
            if (setting->angle_feedback_source == OTHER_FEED) {
                pid_measure = *motor_controller->other_angle_feedback_ptr*PI/180;
                pid_ref = motor->motor_controller.pid_ref*PI/180;
            }
            else
                pid_measure = measure->position; // MOTOR_FEED,对total angle闭环,防止在边界处出现突跃
            // 更新pid_ref进入下一个环
            pid_ref = PIDCalculate(&motor_controller->angle_PID, pid_measure, pid_ref);
        }

        // 计算速度环,(外层闭环为速度或位置)且(启用速度环)时会计算速度环


        if ((setting->close_loop_type & SPEED_LOOP) && (setting->outer_loop_type & (ANGLE_LOOP | SPEED_LOOP)))
        {
            if (setting->feedforward_flag & SPEED_FEEDFORWARD)
                pid_ref += *motor_controller->speed_feedforward_ptr;
            if (setting->speed_feedback_source == OTHER_FEED)
                pid_measure = *motor_controller->other_speed_feedback_ptr;
            else // MOTOR_FEED
                pid_measure = measure->velocity;
            // 更新pid_ref进入下一个环
            pid_ref = PIDCalculate(&motor_controller->speed_PID, pid_measure, pid_ref);
        }
        // 电流环前馈
        if (setting->feedforward_flag & CURRENT_FEEDFORWARD)
            pid_ref += *motor_controller->current_feedforward_ptr;

        if (setting->feedback_reverse_flag == FEEDBACK_DIRECTION_REVERSE)
            pid_ref *= -1;

        set = pid_ref;

        if (motor->stop_flag == MOTOR_STOP)
            set = 0;

        LIMIT_MIN_MAX(set, -motor->DM_T_MAX, motor->DM_T_MAX);
        motor_send_mailbox.position_des = float_to_uint(0, -motor->DM_P_MAX, motor->DM_P_MAX, 16);
        motor_send_mailbox.velocity_des = float_to_uint(0, -motor->DM_V_MAX, motor->DM_V_MAX, 12);
        motor_send_mailbox.torque_des = float_to_uint(set, -motor->DM_T_MAX, motor->DM_T_MAX, 12);
        motor_send_mailbox.Kp = 0;//只使用力矩控制无需kpkd参数
        motor_send_mailbox.Kd = 0;//只使用力矩控制无需kpkd参数


        motor->motor_can_instance->tx_buff[0] = (uint8_t)(motor_send_mailbox.position_des >> 8);
        motor->motor_can_instance->tx_buff[1] = (uint8_t)(motor_send_mailbox.position_des);
        motor->motor_can_instance->tx_buff[2] = (uint8_t)(motor_send_mailbox.velocity_des >> 4);
        motor->motor_can_instance->tx_buff[3] = (uint8_t)(((motor_send_mailbox.velocity_des & 0xF) << 4) | (motor_send_mailbox.Kp >> 8));
        motor->motor_can_instance->tx_buff[4] = (uint8_t)(motor_send_mailbox.Kp);
        motor->motor_can_instance->tx_buff[5] = (uint8_t)(motor_send_mailbox.Kd >> 4);
        motor->motor_can_instance->tx_buff[6] = (uint8_t)(((motor_send_mailbox.Kd & 0xF) << 4) | (motor_send_mailbox.torque_des >> 8));
        motor->motor_can_instance->tx_buff[7] = (uint8_t)(motor_send_mailbox.torque_des);


        CANTransmit(motor->motor_can_instance, 5);
    }

}