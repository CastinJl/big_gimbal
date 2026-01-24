/**
 * @file referee.C
 * @author kidneygood (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2022-11-18
 *
 * @copyright Copyright (c) 2022
 *
 */
#include "referee_task.h"
#include "robot_def.h"
#include "rm_referee.h"
#include "referee_UI.h"
#include "string.h"
#include "cmsis_os.h"
#include "chassis.h"
#include "super_cap.h"

extern SuperCapInstance *cap;                        // 超级电容
static Referee_Interactive_info_t *Interactive_data; // UI绘制需要的机器人状态数据
static referee_info_t *referee_recv_info;            // 接收到的裁判系统数据
Referee_Interactive_info_t ui_data;
uint8_t UI_Seq;                                      // 包序号，供整个referee文件使用
extern uint8_t loader_flag = 0;      //拨弹模式选择标志位
// @todo 不应该使用全局变量

/**
 * @brief  判断各种ID，选择客户端ID
 * @param  referee_info_t *referee_recv_info
 * @retval none
 * @attention
 */
static void DeterminRobotID()
{
    // id小于7是红色,大于7是蓝色   #define Robot_Red 0    #define Robot_Blue 1
    referee_recv_info->referee_id.Robot_Color = referee_recv_info->GameRobotState.robot_id > 7 ? Robot_Blue : Robot_Red;
    referee_recv_info->referee_id.Robot_ID = referee_recv_info->GameRobotState.robot_id;
    referee_recv_info->referee_id.Cilent_ID = 0x0100 + referee_recv_info->referee_id.Robot_ID; // 计算客户端ID
    referee_recv_info->referee_id.Receiver_Robot_ID = 0;
}

static void MyUIRefresh(referee_info_t *referee_recv_info, Referee_Interactive_info_t *_Interactive_data);
static void UIChangeCheck(Referee_Interactive_info_t *_Interactive_data); // 模式切换检测
static void RobotModeTest(Referee_Interactive_info_t *_Interactive_data); // 测试用函数，实现模式自动变化

referee_info_t *UITaskInit(UART_HandleTypeDef *referee_usart_handle, Referee_Interactive_info_t *UI_data)
{
    referee_recv_info = RefereeInit(referee_usart_handle); // 初始化裁判系统的串口,并返回裁判系统反馈数据指针
    Interactive_data = UI_data;                            // 获取UI绘制需要的机器人状态数据
    referee_recv_info->init_flag = 1;
    return referee_recv_info;
}

void UITask()
{
    //RobotModeTest(Interactive_data); // 测试用函数，实现模式自动变化,用于检查该任务和裁判系统是否连接正常
    MyUIRefresh(referee_recv_info, Interactive_data);
}

static Graph_Data_t UI_shoot_line[10]; // 射击准线
static Graph_Data_t UI_shoot_yuan[2];  // 射击圆心
static Graph_Data_t UI_Energy[6];      // 电容能量条
static String_Data_t UI_State_sta[7];  // 机器人状态,静态只需画一次
static String_Data_t UI_State_dyn[7];  // 机器人状态,动态先add才能change
static uint32_t shoot_line_location[10] = {540, 960, 490, 515, 565};
static String_Data_t UI_prompt_sta[3];  // 操作手提示

void MyUIInit()
{
    if (!referee_recv_info->init_flag)
        vTaskDelete(NULL); // 如果没有初始化裁判系统则直接删除ui任务
    while (referee_recv_info->GameRobotState.robot_id == 0)
        osDelay(100); // 若还未收到裁判系统数据,等待一段时间后再检查

    DeterminRobotID();                                            // 确定ui要发送到的目标客户端
    UIDelete(&referee_recv_info->referee_id, UI_Data_Del_ALL, 0); // 清空UI

    // 绘制发射基准线和基准圆
    UILineDraw(&UI_shoot_line[0], "sl0", UI_Graph_ADD, 7, UI_Color_White, 3, 710, shoot_line_location[0], 1210, shoot_line_location[0]);
    UILineDraw(&UI_shoot_line[1], "sl1", UI_Graph_ADD, 7, UI_Color_White, 3, shoot_line_location[1], 340, shoot_line_location[1], 740);
    UILineDraw(&UI_shoot_line[2], "sl2", UI_Graph_ADD, 7, UI_Color_Yellow, 2, 810, shoot_line_location[2], 1110, shoot_line_location[2]);
    UILineDraw(&UI_shoot_line[3], "sl3", UI_Graph_ADD, 7, UI_Color_Orange, 2, 960, 200, 960, 600);
    UICircleDraw(&UI_shoot_yuan[0],"sy0",UI_Graph_ADD,7,UI_Color_Yellow,2,960,540,80);
    UIGraphRefresh(&referee_recv_info->referee_id, 5, UI_shoot_line[0], UI_shoot_line[1], UI_shoot_line[2], UI_shoot_line[3],UI_shoot_yuan[0]);

    // 绘制车辆状态标志指示，静态
    UICharDraw(&UI_State_sta[0], "ss0", UI_Graph_ADD, 8, UI_Color_Main, 20, 2, 10, 750, "E.spin:");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_sta[0]);
//    UICharDraw(&UI_State_sta[1], "ss1", UI_Graph_ADD, 8, UI_Color_Yellow, 15, 2, 150, 700, "gimbal:");
//    UICharRefresh(&referee_recv_info->referee_id, UI_State_sta[1]);
//    UICharDraw(&UI_State_sta[2], "ss2", UI_Graph_ADD, 8, UI_Color_Orange, 15, 2, 150, 650, "shoot:");
//    UICharRefresh(&referee_recv_info->referee_id, UI_State_sta[2]);
    UICharDraw(&UI_State_sta[3], "ss3", UI_Graph_ADD, 8, UI_Color_Pink, 20, 2, 10, 700, "F.frict:");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_sta[3]);
    UICharDraw(&UI_State_sta[4], "ss4", UI_Graph_ADD, 8, UI_Color_Main, 20, 2, 10, 650, "B.lid:");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_sta[4]);
    UICharDraw(&UI_State_sta[5], "ss5", UI_Graph_ADD, 8, UI_Color_Pink, 20, 2, 10, 600, "Q.heat:");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_sta[5]);
//    UICharDraw(&UI_State_sta[5], "ss5", UI_Graph_ADD, 8, UI_Color_Pink, 15, 2, 150, 800, "load:");
//    UICharRefresh(&referee_recv_info->referee_id, UI_State_sta[5]);

    // 操作手提示
    UICharDraw(&UI_prompt_sta[0], "ss8", UI_Graph_ADD,8, UI_Color_Pink, 30, 4, 1650, 700, "leng jing");
    UICharRefresh(&referee_recv_info->referee_id, UI_prompt_sta[0]);
    UICharDraw(&UI_prompt_sta[1], "ss9", UI_Graph_ADD, 8,UI_Color_Pink, 30, 4, 1650, 800, "zhi hui");
    UICharRefresh(&referee_recv_info->referee_id, UI_prompt_sta[1]);
    UICharDraw(&UI_prompt_sta[2], "ss10", UI_Graph_ADD, 8,UI_Color_Pink, 30, 4, 1650, 600, "bie ji");
    UICharRefresh(&referee_recv_info->referee_id, UI_prompt_sta[2]);
    // 底盘功率显示，静态
//    UICharDraw(&UI_State_sta[6], "ss6", UI_Graph_ADD, 8, UI_Color_Green, 18, 2, 620, 230, "MAX:");
//    UICharRefresh(&referee_recv_info->referee_id, UI_State_sta[6]);

    // 绘制车辆状态标志，动态
    // 由于初始化时xxx_last_mode默认为0，所以此处对应UI也应该设为0时对应的UI，防止模式不变的情况下无法置位flag，导致UI无法刷新
    UICharDraw(&UI_State_dyn[0], "sd0", UI_Graph_ADD, 8, UI_Color_Main, 20, 2, 170, 750, "off");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[0]);
//    UICharDraw(&UI_State_dyn[1], "sd1", UI_Graph_ADD, 8, UI_Color_Yellow, 15, 2, 270, 700, "zeroforce");
//    UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[1]);
//    UICharDraw(&UI_State_dyn[2], "sd2", UI_Graph_ADD, 8, UI_Color_Orange, 15, 2, 270, 650, "off");
//    UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[2]);
    UICharDraw(&UI_State_dyn[3], "sd3", UI_Graph_ADD, 8, UI_Color_Pink, 20, 2, 170, 700, "off");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[3]);
    UICharDraw(&UI_State_dyn[4], "sd4", UI_Graph_ADD, 8, UI_Color_Pink, 20, 2, 170, 650, "off");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[4]);
    UICharDraw(&UI_State_dyn[5], "sd5", UI_Graph_ADD, 8, UI_Color_Pink, 20, 2, 170, 600, "on ");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[5]);
//    UICharDraw(&UI_State_dyn[5], "sd5", UI_Graph_ADD, 8, UI_Color_Pink, 15, 2, 270, 800, "999");
//    UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[5]);

    // 能量条框
//    UIRectangleDraw(&UI_Energy[0], "ss6", UI_Graph_ADD, 7, UI_Color_Green, 2, 720, 140, 1220, 180);
//    UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_Energy[0]);

    // 底盘功率显示,动态
    UIFloatDraw(&UI_Energy[1], "sd6", UI_Graph_ADD, 8, UI_Color_Green, 18, 2, 2, 850, 230, Interactive_data->Chassis_Power_Data.chassis_power_mx * 1000);
    UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_Energy[1]);

    // 电容电压
    // UIFloatDraw(&UI_Energy[2], "sd7", UI_Graph_ADD, 8, UI_Color_Pink, 18, 2,2, 850, 280,Interactive_data->Cap_Data.cap_vol);
    UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_Energy[2]);
    // 电容能量条
//    UILineDraw(&UI_Energy[3], "sd8", UI_Graph_ADD, 8, UI_Color_Pink, 30, 720, 160, (uint32_t)(720 + (Interactive_data->Cap_Data.cap_vol-1200) * 0.416), 160);
//    UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_Energy[3]);

    // 发弹量条框
    UILineDraw(&UI_Energy[4], "ss7", UI_Graph_ADD, 7, UI_Color_Pink, 2, 1325, 500, 1480, 500);
    UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_Energy[4]);
}

static void MyUIRefresh(referee_info_t *referee_recv_info, Referee_Interactive_info_t *_Interactive_data)
{
    UIChangeCheck(_Interactive_data);
    // chassis
    if (_Interactive_data->Referee_Interactive_Flag.chassis_flag == 1)
    {
        // switch (_Interactive_data->chassis_mode)
        //     // 此处注意字数对齐问题，字数相同才能覆盖掉
        // {
        //     case CHASSIS_ZERO_FORCE:
        //          UICharDraw(&UI_State_dyn[0], "sd0", UI_Graph_Change, 8, UI_Color_Main, 20, 2, 170, 750, "off");
        //          break;
        //     case CHASSIS_ROTATE:
        //          UICharDraw(&UI_State_dyn[0], "sd0", UI_Graph_Change, 8, UI_Color_Main, 20, 2, 170, 750, "on ");
        //          break;
        //     case CHASSIS_SIDEWAYS:
        //         UICharDraw(&UI_State_dyn[0], "sd0", UI_Graph_Change, 8, UI_Color_Main, 20, 2, 170, 750, "off");
        //         break;
        //     case CHASSIS_NO_FOLLOW:
        //         UICharDraw(&UI_State_dyn[0], "sd0", UI_Graph_Change, 8, UI_Color_Main, 20, 2, 170, 750, "off");
        //         break;
        //     case CHASSIS_FOLLOW_GIMBAL_YAW:
        //         UICharDraw(&UI_State_dyn[0], "sd0", UI_Graph_Change, 8, UI_Color_Main, 20, 2, 170, 750, "off");
        //         break;
        // }
        UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[0]);
        _Interactive_data->Referee_Interactive_Flag.chassis_flag = 0;
    }
    // gimbal
//    if (_Interactive_data->Referee_Interactive_Flag.gimbal_flag == 1)
//    {
//        switch (_Interactive_data->gimbal_mode)
//        {
//        case GIMBAL_ZERO_FORCE:
//        {
//            UICharDraw(&UI_State_dyn[1], "sd1", UI_Graph_Change, 8, UI_Color_Yellow, 15, 2, 270, 700, "zeroforce");
//            break;
//        }
//        case GIMBAL_FREE_MODE:
//        {
//            UICharDraw(&UI_State_dyn[1], "sd1", UI_Graph_Change, 8, UI_Color_Yellow, 15, 2, 270, 700, "free     ");
//            break;
//        }
//        case GIMBAL_GYRO_MODE:
//        {
//            UICharDraw(&UI_State_dyn[1], "sd1", UI_Graph_Change, 8, UI_Color_Yellow, 15, 2, 270, 700, "gyro     ");
//            break;
//        }
//        }
//        UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[1]);
//        _Interactive_data->Referee_Interactive_Flag.gimbal_flag = 0;
//    }
    // shoot
//    if (_Interactive_data->Referee_Interactive_Flag.shoot_flag == 1)
//    {
//        UICharDraw(&UI_State_dyn[2], "sd2", UI_Graph_Change, 8, UI_Color_Pink, 15, 2, 270, 650, _Interactive_data->shoot_mode == SHOOT_ON ? "on " : "off");
//        UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[2]);
//        _Interactive_data->Referee_Interactive_Flag.shoot_flag = 0;
//    }
    // friction
    if (_Interactive_data->Referee_Interactive_Flag.friction_flag == 1)
    {
        UICharDraw(&UI_State_dyn[3], "sd3", UI_Graph_Change, 8, UI_Color_Pink, 20, 2, 170, 700, _Interactive_data->friction_mode == FRICTION_ON ? "on " : "off");
        UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[3]);
        _Interactive_data->Referee_Interactive_Flag.friction_flag = 0;
    }
    // lid
    if (_Interactive_data->Referee_Interactive_Flag.lid_flag == 1)
    {
        UICharDraw(&UI_State_dyn[4], "sd4", UI_Graph_Change, 8, UI_Color_Pink, 20, 2, 170, 650, _Interactive_data->lid_mode == LID_OPEN ? "on " : "off");
        UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[4]);
        _Interactive_data->Referee_Interactive_Flag.lid_flag = 0;
    }
    // heat
    if (_Interactive_data->Referee_Interactive_Flag.heat_flag == 1)
    {
        // UICharDraw(&UI_State_dyn[5], "sd5", UI_Graph_Change, 8, UI_Color_Pink, 20, 2, 170, 600, _Interactive_data->heat_mode == HEAT_OPEN ? "on " : "off");
        UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[5]);
        _Interactive_data->Referee_Interactive_Flag.heat_flag = 0;
    }
    // loader_mode
//    if (_Interactive_data->Referee_Interactive_Flag.load_flag == 1)
//    {
//        switch (loader_flag)
//        {
//            case 2:
//            {
//                UICharDraw(&UI_State_dyn[5], "sd5", UI_Graph_Change, 8, UI_Color_Pink, 15, 2, 270, 800, " 1 ");
//                break;
//            }
//            case 1:
//            {
//                UICharDraw(&UI_State_dyn[5], "sd5", UI_Graph_Change, 8, UI_Color_Pink, 15, 2, 270, 800, " 3 ");
//                break;
//            }
//            case 0:
//            {
//                UICharDraw(&UI_State_dyn[5], "sd5", UI_Graph_Change, 8, UI_Color_Pink, 15, 2, 270, 800, "999");
//                break;
//            }
//        }
//        UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[5]);
//        _Interactive_data->Referee_Interactive_Flag.load_flag = 0;
//    }
    // power
    if (_Interactive_data->Referee_Interactive_Flag.Power_flag == 1)
    {
        UIFloatDraw(&UI_Energy[1], "sd6", UI_Graph_Change, 8, UI_Color_Green, 18, 2, 2, 850, 230, _Interactive_data->Chassis_Power_Data.chassis_power_mx * 1000);
        UIGraphRefresh(&referee_recv_info->referee_id, 2, UI_Energy[1]);
        _Interactive_data->Referee_Interactive_Flag.Power_flag = 0;
    }
    // UIFloatDraw(&UI_Energy[2], "sd7", UI_Graph_Change, 8, UI_Color_Pink, 18, 2,2, 850, 280,Interactive_data->Cap_Data.cap_vol);
    UIGraphRefresh(&referee_recv_info->referee_id, 1,UI_Energy[2]);
//    UILineDraw(&UI_Energy[3], "sd8", UI_Graph_Change, 8, UI_Color_Pink, 30, 720, 160, (uint32_t)(720 + (Interactive_data->Cap_Data.cap_vol-1200) * 0.416), 160);
//    UIGraphRefresh(&referee_recv_info->referee_id, 1, &UI_Energy[3]);
    //绘制开火建议
    if (_Interactive_data->Referee_Interactive_Flag.aim_flag == 1) {
        UICircleDraw(&UI_shoot_yuan[0], "sy0", UI_Graph_Change, 7, _Interactive_data->aim_fire == 0 ? UI_Color_Yellow : UI_Color_Main, 2, 960, 540, 80);
        UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_shoot_yuan[0]);
        _Interactive_data->Referee_Interactive_Flag.aim_flag = 0;
    }
}

/**
 * @brief  模式切换检测,模式发生切换时，对flag置位
 * @param  Referee_Interactive_info_t *_Interactive_data
 * @retval none
 * @attention
 */
static void UIChangeCheck(Referee_Interactive_info_t *_Interactive_data)
{
    // if (_Interactive_data->chassis_mode != _Interactive_data->chassis_last_mode)
    // {
    //     _Interactive_data->Referee_Interactive_Flag.chassis_flag = 1;
    //     _Interactive_data->chassis_last_mode = _Interactive_data->chassis_mode;
    // }

//    if (_Interactive_data->gimbal_mode != _Interactive_data->gimbal_last_mode)
//    {
//        _Interactive_data->Referee_Interactive_Flag.gimbal_flag = 1;
//        _Interactive_data->gimbal_last_mode = _Interactive_data->gimbal_mode;
//    }

//    if (_Interactive_data->shoot_mode != _Interactive_data->shoot_last_mode)
//    {
//        _Interactive_data->Referee_Interactive_Flag.shoot_flag = 1;
//        _Interactive_data->shoot_last_mode = _Interactive_data->shoot_mode;
//    }

    if (_Interactive_data->friction_mode != _Interactive_data->friction_last_mode)
    {
        _Interactive_data->Referee_Interactive_Flag.friction_flag = 1;
        _Interactive_data->friction_last_mode = _Interactive_data->friction_mode;
    }

    if (_Interactive_data->lid_mode != _Interactive_data->lid_last_mode)
    {
        _Interactive_data->Referee_Interactive_Flag.lid_flag = 1;
        _Interactive_data->lid_last_mode = _Interactive_data->lid_mode;
    }

    // if (_Interactive_data->heat_mode != _Interactive_data->heat_last_mode)
    // {
    //     _Interactive_data->Referee_Interactive_Flag.heat_flag = 1;
    //     _Interactive_data->heat_last_mode = _Interactive_data->heat_mode;
    // }

    if (_Interactive_data->Chassis_Power_Data.chassis_power_mx != _Interactive_data->Chassis_last_Power_Data.chassis_power_mx)
    {
        _Interactive_data->Referee_Interactive_Flag.Power_flag = 1;
        _Interactive_data->Chassis_last_Power_Data.chassis_power_mx = _Interactive_data->Chassis_Power_Data.chassis_power_mx;
    }

//    if (_Interactive_data->Cap_Data.cap_vol != _Interactive_data->Cap_last_Data.cap_vol)
//    {
//        _Interactive_data->Referee_Interactive_Flag.cap_flag = 1;
//        _Interactive_data->Cap_last_Data.cap_vol = _Interactive_data->Cap_Data.cap_vol;
//    }

    if (_Interactive_data->aim_fire != _Interactive_data->aim_last_fire)
    {
        _Interactive_data->Referee_Interactive_Flag.aim_flag = 1;
        _Interactive_data->aim_last_fire = _Interactive_data->aim_fire;
    }

//    if (_Interactive_data->loader_mode != _Interactive_data->loader_last_mode)
//    {
//        _Interactive_data->Referee_Interactive_Flag.load_flag = 1;
//        _Interactive_data->loader_last_mode = _Interactive_data->loader_mode;
//    }
}
