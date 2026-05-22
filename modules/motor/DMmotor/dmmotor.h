#ifndef DM4310_H
#define DM4310_H
#include <stdint.h>
#include "bsp_can.h"
#include "controller.h"
#include "motor_def.h"
#include "daemon.h"

#define DM_MOTOR_CNT 6

// #define DM_P_MIN  (-12.5)//(-3.1415926f)
// #define DM_P_MAX  12.5//3.1415926f
// #define DM_V_MIN  (-30.0f)
// #define DM_V_MAX  30.0f
// #define DM_T_MIN  (-10.0f)
// #define DM_T_MAX   10.0f

typedef struct
{
    uint8_t id;
    uint8_t state;
    float velocity;     //速度
    float last_position;    //上次位置
    float position; //位置
    float torque;   //力矩
    float T_Mos;    //mos温度
    float T_Rotor; //电机温度
    float angle_single_round; //单圈角度
    int32_t total_round; //总圈数
    float total_angle; //总角度
}DM_Motor_Measure_s;

typedef struct
{
    uint16_t position_des;
    uint16_t velocity_des;
    uint16_t torque_des;
    uint16_t Kp;
    uint16_t Kd;
}DMMotor_Send_s;
typedef struct
{
    // DM_Motor_Measure_s measure;
    // Motor_Control_Setting_s motor_settings;
    // Motor_Controller_s motor_controller;//电机控制器
    //
    // CANInstance *motor_can_instance;//电机can实例
    //
    // Motor_Type_e motor_type;//电机类型
    // Motor_Working_Type_e stop_flag;
    //
    // DaemonInstance* motor_daemon;
    // uint32_t lost_cnt;
    // float DM_T_MAX;
    // float DM_V_MAX;
    // float DM_P_MAX;

    DM_Motor_Measure_s measure;//反馈

    Motor_Control_Setting_s motor_settings;//闭环设置
    Motor_Controller_s motor_controller;//闭环

    CANInstance *motor_can_instance;//电机can实例
    //启停
    Motor_Working_Type_e stop_flag;
    //PVTMAX设置
    float DM_T_MAX;
    float DM_V_MAX;
    float DM_P_MAX;
    //通信测试
    float deltatime;
    uint32_t deltatimeline;
    //触发看门狗
    DaemonInstance* motor_daemon;
    uint8_t lost_flag;//丢失标志
}DMMotorInstance;

typedef enum
{
    DM_CMD_MOTOR_MODE = 0xfc,   // 使能,会响应指令
    DM_CMD_RESET_MODE = 0xfd,   // 停止
    DM_CMD_ZERO_POSITION = 0xfe, // 将当前的位置设置为编码器零位
    DM_CMD_CLEAR_ERROR = 0xfb // 清除电机过热错误
}DMMotor_Mode_e;

DMMotorInstance *DMMotorInit(Motor_Init_Config_s *config);

void DMMotorSetRef(DMMotorInstance *motor, float ref);

void DMMotorOuterLoop(DMMotorInstance *motor,Closeloop_Type_e closeloop_type);

void DMMotorEnable(DMMotorInstance *motor);
void DMMotorSetMode(DMMotor_Mode_e cmd, DMMotorInstance *motor);
void DMMotorClearError(DMMotorInstance *motor);
void DMMotorStop(DMMotorInstance *motor);
void DMMotorCaliEncoder(DMMotorInstance *motor);
void DMMotorControlInit();
void DMMotorControl();
#endif // !DMMOTOR