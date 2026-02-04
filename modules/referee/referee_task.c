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

static Referee_Interactive_info_t *Interactive_data; // UI绘制需要的机器人状态数据
static referee_info_t *referee_recv_info;            // 接收到的裁判系统数据
Referee_Interactive_info_t ui_data;
uint8_t UI_Seq;                                      // 包序号，供整个referee文件使用
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
    referee_recv_info->referee_id.Robot_Color = Robot_Red;
    referee_recv_info->referee_id.Robot_ID = 3;
    referee_recv_info->referee_id.Cilent_ID = 0x0100 + 3; // 计算客户端ID
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



//规定第0层是静态的   也就是说是不变的画图     更高的层数用来画动态的图形   注意更高层会覆盖更底层的内容   删除时可以用del函数删除同一图层的内容
/* ================= 全局图形变量定义 ================= */

// 静态图形
static Graph_Data_t UI_Static_Shapes[4];
static String_Data_t UI_Static_Texts[8];
// 动态状态框
static Graph_Data_t UI_Dynamic_Boxes[7];
// 动态条
static Graph_Data_t UI_Dynamic_Bars[1];    // CAP Bar
// 动态数值
static Graph_Data_t UI_Dynamic_Numbers[1]; // Bullet Num
// 受击指示线
static Graph_Data_t UI_Dynamic_HitLines[4];
static uint8_t beat_direction_continuous_cnt[4] = {0}; // 四个方向

static Graph_Data_t UI_shoot_line[10]; // 射击准线
static Graph_Data_t UI_shoot_yuan[2];  // 射击圆心
static Graph_Data_t UI_Energy[6];      // 电容能量条
static String_Data_t UI_State_sta[7];  // 机器人状态,静态只需画一次
static String_Data_t UI_State_dyn[7];  // 机器人状态,动态先add才能change
static uint32_t shoot_line_location[10] = {540, 960, 490, 515, 565};
static String_Data_t UI_prompt_sta[3];  // 操作手提示

void MyUIInit()
{
    // if (!referee_recv_info->init_flag)
    //     vTaskDelete(NULL); // 如果没有初始化裁判系统则直接删除ui任务
    // while (referee_recv_info->GameRobotState.robot_id == 0)
    //     osDelay(100); // 若还未收到裁判系统数据,等待一段时间后再检查

    DeterminRobotID();                                            // 确定ui要发送到的目标客户端
    UIDelete(&referee_recv_info->referee_id, UI_Data_Del_ALL, 0); // 清空UI

    // 绘制发射基准线和基准圆
    UILineDraw(&UI_shoot_line[0], "sl0", UI_Graph_ADD, 7, UI_Color_White, 3, 710, shoot_line_location[0], 1210, shoot_line_location[0]);
    UILineDraw(&UI_shoot_line[1], "sl1", UI_Graph_ADD, 7, UI_Color_White, 3, shoot_line_location[1], 340, shoot_line_location[1], 740);
    UILineDraw(&UI_shoot_line[2], "sl2", UI_Graph_ADD, 7, UI_Color_Yellow, 2, 810, shoot_line_location[2], 1110, shoot_line_location[2]);
    UILineDraw(&UI_shoot_line[3], "sl3", UI_Graph_ADD, 7, UI_Color_Orange, 2, 960, 200, 960, 600);
    UICircleDraw(&UI_shoot_yuan[0],"sy0",UI_Graph_ADD,7,UI_Color_Yellow,2,960,540,80);
    UIGraphRefresh(&referee_recv_info->referee_id, 5, UI_shoot_line[0], UI_shoot_line[1], UI_shoot_line[2], UI_shoot_line[3],UI_shoot_yuan[0]);

    // 静态图形
    // bll  bottom_line_left   blr  bottom_line_right  前进轨迹线
    UILineDraw(&UI_Static_Shapes[0], "bll", UI_Graph_ADD, 0, UI_Color_Purplish_red, 3, 539, 43, 682, 252);
    UILineDraw(&UI_Static_Shapes[1], "blr", UI_Graph_ADD, 0, UI_Color_Purplish_red, 3, 1389, 39, 1232, 250);
    //!!!!!!!!!!!!!!!!!!!!按照手册   不支持一次发送3 4 6个图形   也就是说一次必须只能发1 2 5 7个
    UIGraphRefresh(&referee_recv_info->referee_id, 2, UI_Static_Shapes[0],UI_Static_Shapes[1]);

    // 准星圆点   poi  point
    UICircleDraw(&UI_Static_Shapes[2], "poi", UI_Graph_ADD, 0, UI_Color_Green, 10, 955, 535, 5);
    // CAP 边框   cre  cap_rectangle
    UIRectangleDraw(&UI_Static_Shapes[3], "cre", UI_Graph_ADD, 0, UI_Color_White, 1, 665, 70, 1265, 100);
    UIGraphRefresh(&referee_recv_info->referee_id, 2, UI_Static_Shapes[2],UI_Static_Shapes[3]);

    // 动态层初始化 (Layer 2-9)
    // 先 ADD 一遍，位置设为 0 或者默认状态，方便后面 Change
    // 状态框 (Boxes)
    // 名字映射  rai rectangle autoaim    rsi rectangle single     rco rec continue     rfo  rec follow     rsp   rec spin    rfr  rec friction
    UIRectangleDraw(&UI_Dynamic_Boxes[0], "rai", UI_Graph_ADD, 1, UI_Color_Purplish_red, 4, 0, 0, 0, 0);
    UIRectangleDraw(&UI_Dynamic_Boxes[1], "rsi", UI_Graph_ADD, 1, UI_Color_Purplish_red, 4, 0, 0, 0, 0);
    UIRectangleDraw(&UI_Dynamic_Boxes[2], "rco", UI_Graph_ADD, 1, UI_Color_Purplish_red, 4, 0, 0, 0, 0);
    UIRectangleDraw(&UI_Dynamic_Boxes[3], "rfo", UI_Graph_ADD, 1, UI_Color_Purplish_red, 4, 0, 0, 0, 0);
    UIRectangleDraw(&UI_Dynamic_Boxes[4], "rsp", UI_Graph_ADD, 1, UI_Color_Purplish_red, 4, 0, 0, 0, 0);
    UIRectangleDraw(&UI_Dynamic_Boxes[5], "rfr", UI_Graph_ADD, 1, UI_Color_Purplish_red, 4, 0, 0, 0, 0);
    // 中心瞄准框 rec_center
    UIRectangleDraw(&UI_Dynamic_Boxes[6], "rec", UI_Graph_ADD, 2, UI_Color_Cyan, 2, 697, 363, 1227, 723);//todo:换成自瞄识别范围对应的位置
    UIGraphRefresh(&referee_recv_info->referee_id, 7, UI_Dynamic_Boxes[0],UI_Dynamic_Boxes[1],UI_Dynamic_Boxes[2],UI_Dynamic_Boxes[3],UI_Dynamic_Boxes[4],UI_Dynamic_Boxes[5],UI_Dynamic_Boxes[6]);

    // CAP 储能条  cap line
    UILineDraw(&UI_Dynamic_Bars[0], "cli", UI_Graph_ADD, 3, UI_Color_Green, 30, 650, 70, 655, 100); // 初始长度1   高 30
    UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_Dynamic_Bars[0]);
    // Bullet 数值 (num)
    UIIntDraw(&UI_Dynamic_Numbers[0], "num", UI_Graph_ADD, 4, UI_Color_Purplish_red, 30, 3, 1436, 826, 0);
    UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_Dynamic_Numbers[0]);
    // 受击指示线 lil, lir, lib, lif       line_left, right, back, front
    UILineDraw(&UI_Dynamic_HitLines[0], "lil", UI_Graph_ADD, 5, UI_Color_Orange, 10, 0, 0, 0, 0);
    UILineDraw(&UI_Dynamic_HitLines[1], "lir", UI_Graph_ADD, 5, UI_Color_Orange, 10, 0, 0, 0, 0);
    UIGraphRefresh(&referee_recv_info->referee_id, 2, UI_Dynamic_HitLines[0],UI_Dynamic_HitLines[1]);
    UILineDraw(&UI_Dynamic_HitLines[2], "lib", UI_Graph_ADD, 5, UI_Color_Orange, 10, 0, 0, 0, 0);
    UILineDraw(&UI_Dynamic_HitLines[3], "lif", UI_Graph_ADD, 5, UI_Color_Orange, 10, 0, 0, 0, 0);
    UIGraphRefresh(&referee_recv_info->referee_id, 2, UI_Dynamic_HitLines[2],UI_Dynamic_HitLines[3]);;

}

static void MyUIRefresh(referee_info_t *referee_recv_info, Referee_Interactive_info_t *_Interactive_data)
{
    UIChangeCheck(_Interactive_data);
    // 底盘状态
    if (_Interactive_data->Referee_Interactive_Flag.chassis_flag == 1)
    {
        if (_Interactive_data->chassis_mode == CHASSIS_ROTATE) {
            UIRectangleDraw(&UI_Dynamic_Boxes[3], "rfo", UI_Graph_Change, 1, UI_Color_Purplish_red, 4, 0, 0, 0, 0);
            UIRectangleDraw(&UI_Dynamic_Boxes[4], "rsp", UI_Graph_Change, 1, UI_Color_Purplish_red, 4, 1085, 820, 1224, 870);
        }
        else if (_Interactive_data->chassis_mode == CHASSIS_FOLLOW_GIMBAL_YAW) {
            UIRectangleDraw(&UI_Dynamic_Boxes[3], "rfo", UI_Graph_Change, 1, UI_Color_Purplish_red, 4, 1250, 820, 1365, 870);
            UIRectangleDraw(&UI_Dynamic_Boxes[4], "rsp", UI_Graph_Change, 1, UI_Color_Purplish_red, 4, 0, 0, 0, 0);
        }else {
            UIRectangleDraw(&UI_Dynamic_Boxes[3], "rfo", UI_Graph_Change, 1, UI_Color_Purplish_red, 4, 0, 0, 0, 0);
            UIRectangleDraw(&UI_Dynamic_Boxes[4], "rsp", UI_Graph_Change, 1, UI_Color_Purplish_red, 4, 0, 0, 0, 0);
        }
        UIGraphRefresh(&referee_recv_info->referee_id, 2, UI_Dynamic_Boxes[3],UI_Dynamic_Boxes[4]);
        _Interactive_data->Referee_Interactive_Flag.chassis_flag = 0;
    }

    // 摩擦轮状态框 (FRIC)
    if (_Interactive_data->Referee_Interactive_Flag.friction_flag == 1)
    {
        if (_Interactive_data->friction_mode == FRICTION_ON) {
            UIRectangleDraw(&UI_Dynamic_Boxes[5], "rfr", UI_Graph_Change, 1, UI_Color_Purplish_red, 4, 440, 535, 559, 596);
        }
        else {
            UIRectangleDraw(&UI_Dynamic_Boxes[5], "rfr", UI_Graph_Change, 1, UI_Color_Purplish_red, 4, 0, 0, 0, 0);
        }
        UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_Dynamic_Boxes[5]);
        _Interactive_data->Referee_Interactive_Flag.friction_flag = 0;
    }

   // 射击模式 (SINGLE / CONTINUE)
    if (_Interactive_data->Referee_Interactive_Flag.load_flag == 1)
    {
        if (_Interactive_data->loader_mode == LOAD_BURSTFIRE) {
            UIRectangleDraw(&UI_Dynamic_Boxes[1], "rsi", UI_Graph_Change, 1, UI_Color_Purplish_red, 4, 0, 0, 0, 0);
            UIRectangleDraw(&UI_Dynamic_Boxes[2], "rco", UI_Graph_Change, 1, UI_Color_Purplish_red, 4, 690, 820, 875, 871);
        }
        else if (_Interactive_data->loader_mode == LOAD_1_BULLET) {
            UIRectangleDraw(&UI_Dynamic_Boxes[1], "rsi", UI_Graph_Change, 1, UI_Color_Purplish_red, 4, 510, 820, 649, 870);
            UIRectangleDraw(&UI_Dynamic_Boxes[2], "rco", UI_Graph_Change, 1, UI_Color_Purplish_red, 4, 0, 0, 0, 0);
        }
        else {
            UIRectangleDraw(&UI_Dynamic_Boxes[1], "rsi", UI_Graph_Change, 1, UI_Color_Purplish_red, 4, 0, 0, 0, 0);
            UIRectangleDraw(&UI_Dynamic_Boxes[2], "rco", UI_Graph_Change, 1, UI_Color_Purplish_red, 4, 0, 0, 0, 0);
        }
        UIGraphRefresh(&referee_recv_info->referee_id, 2, UI_Dynamic_Boxes[1],UI_Dynamic_Boxes[2]);
        _Interactive_data->Referee_Interactive_Flag.load_flag = 0;
    }

    // 自瞄状态框 (AUTOAIM)
    if (_Interactive_data->Referee_Interactive_Flag.auto_aim_mode_flag == 1)
    {
        if (_Interactive_data->auto_aim_mode == 1){
            UIRectangleDraw(&UI_Dynamic_Boxes[0], "rai", UI_Graph_Change, 1, UI_Color_Purplish_red, 4, 250, 820, 418, 870);
        }else {
            UIRectangleDraw(&UI_Dynamic_Boxes[0], "rai", UI_Graph_Change, 1, UI_Color_Purplish_red, 4, 0,0,0,0);
        }
        UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_Dynamic_Boxes[0]);
        _Interactive_data->Referee_Interactive_Flag.auto_aim_mode_flag = 0;
    }

    //  刷新弹量数值 (Bullet)
    if (_Interactive_data->Referee_Interactive_Flag.bullet_number_flag == 1)
    {
        int number=Interactive_data->bullet_number;
        if (number<0) number=0;
        UIIntDraw(&UI_Dynamic_Numbers[0], "num", UI_Graph_Change, 4, UI_Color_Purplish_red, 30, 3, 1436, 826, number);
        UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_Dynamic_Numbers[0]);
        _Interactive_data->Referee_Interactive_Flag.bullet_number_flag = 0;
    }

    // 刷新电容 (CAP) 进度条
    if (_Interactive_data->Referee_Interactive_Flag.cap_energy_flag == 1)
    {
        int length = Interactive_data->SuperCapEnergy*6;//电容0到100   长度0到600
        UILineDraw(&UI_Dynamic_Bars[0], "cli", UI_Graph_Change, 3, UI_Color_Green, 30, 650, 70, 651+length, 100);
        UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_Dynamic_Bars[0]);
        _Interactive_data->Referee_Interactive_Flag.cap_energy_flag = 0;
    }

    //开火建议
    if (_Interactive_data->Referee_Interactive_Flag.auto_aim_fire_suggestion_flag == 1)
    {
        if (Interactive_data->auto_aim_fire_suggestion==1)
            UIRectangleDraw(&UI_Dynamic_Boxes[6], "rec", UI_Graph_Change, 2, UI_Color_Purplish_red, 2, 697, 363, 1227, 723);//todo:换成自瞄识别范围对应的位置
        else
            UIRectangleDraw(&UI_Dynamic_Boxes[6], "rec", UI_Graph_Change, 2, UI_Color_Cyan, 2, 697, 363, 1227, 723);//todo:换成自瞄识别范围对应的位置
        UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_Dynamic_Boxes[6]);
        _Interactive_data->Referee_Interactive_Flag.auto_aim_fire_suggestion_flag = 0;
    }

    // 受击指示线刷新
    if (_Interactive_data->Referee_Interactive_Flag.beat_flag == 1) {
        switch (_Interactive_data->armor_beat_id)
        {
        case 0:
                if(beat_direction_continuous_cnt[0] >0 && beat_direction_continuous_cnt[0] %10==0) {//每次更新都是加10   只在更新的时候画线  其余时间保持不更新
                    UILineDraw(&UI_Dynamic_HitLines[3], "lif", UI_Graph_Change, 5, UI_Color_Orange, 10,733,786,1167,786);
                    UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_Dynamic_HitLines[3]);
                    beat_direction_continuous_cnt[0]--;
                }else if (beat_direction_continuous_cnt[0]<=0){
                    UILineDraw(&UI_Dynamic_HitLines[3], "lif", UI_Graph_Change, 5, UI_Color_Orange, 10,0,0,0,0);
                    UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_Dynamic_HitLines[3]);
                    beat_direction_continuous_cnt[0]=0;
                }
            break;
        case 1:
                if(beat_direction_continuous_cnt[1] > 0 && beat_direction_continuous_cnt[1] %10==0) {
                    UILineDraw(&UI_Dynamic_HitLines[0], "lil", UI_Graph_Change, 5, UI_Color_Orange, 10, 624,353,624,725);
                    UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_Dynamic_HitLines[0]);
                    beat_direction_continuous_cnt[1]--;
                }else if (beat_direction_continuous_cnt[1]<=0){
                    UILineDraw(&UI_Dynamic_HitLines[0], "lil", UI_Graph_Change, 5, UI_Color_Orange, 10,0,0,0,0);
                    UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_Dynamic_HitLines[0]);
                    beat_direction_continuous_cnt[1]=0;
                }
            break;
        case 2:
                if(beat_direction_continuous_cnt[2] > 0 && beat_direction_continuous_cnt[2] %10==0) {
                    UILineDraw(&UI_Dynamic_HitLines[2], "lib", UI_Graph_Change, 5, UI_Color_Orange, 10, 731,292,1165,292);
                    UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_Dynamic_HitLines[2]);
                    beat_direction_continuous_cnt[2]--;
                }else if (beat_direction_continuous_cnt[2]<=0){
                    UILineDraw(&UI_Dynamic_HitLines[2], "lib", UI_Graph_Change, 5, UI_Color_Orange, 10,0,0,0,0);
                    UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_Dynamic_HitLines[2]);
                    beat_direction_continuous_cnt[2]=0;
                }
            break;
        case 3:
                if(beat_direction_continuous_cnt[3] > 0 && beat_direction_continuous_cnt[3] %10==0) {
                    UILineDraw(&UI_Dynamic_HitLines[1], "lir", UI_Graph_Change, 5, UI_Color_Orange, 10, 1296,357,1296,729);
                    UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_Dynamic_HitLines[1]);
                    beat_direction_continuous_cnt[3]--;
                }else if (beat_direction_continuous_cnt[3]<=0){
                    UILineDraw(&UI_Dynamic_HitLines[1], "lir", UI_Graph_Change, 5, UI_Color_Orange, 10,0,0,0,0);
                    UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_Dynamic_HitLines[1]);
                    beat_direction_continuous_cnt[3]=0;
                }
            break;
        default:
            break;
        }
        _Interactive_data->Referee_Interactive_Flag.beat_flag = 0;
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
    if (_Interactive_data->chassis_mode != _Interactive_data->chassis_last_mode)
    {
        _Interactive_data->Referee_Interactive_Flag.chassis_flag = 1;
        _Interactive_data->chassis_last_mode = _Interactive_data->chassis_mode;
    }

    if (_Interactive_data->friction_mode != _Interactive_data->friction_last_mode)
    {
        _Interactive_data->Referee_Interactive_Flag.friction_flag = 1;
        _Interactive_data->friction_last_mode = _Interactive_data->friction_mode;
    }

    if (_Interactive_data->loader_mode!= _Interactive_data->loader_last_mode)
    {
        _Interactive_data->Referee_Interactive_Flag.load_flag = 1;
        _Interactive_data->loader_last_mode = _Interactive_data->loader_mode;
    }

    if (_Interactive_data->auto_aim_mode != _Interactive_data->auto_aim_last_mode)//自瞄开启
    {
        _Interactive_data->Referee_Interactive_Flag.auto_aim_mode_flag = 1;
        _Interactive_data->auto_aim_last_mode = _Interactive_data->auto_aim_mode;
    }

    if (_Interactive_data->bullet_number != _Interactive_data->bullet_number_last)
    {
        _Interactive_data->Referee_Interactive_Flag.bullet_number_flag = 1;
        _Interactive_data->bullet_number_last = _Interactive_data->bullet_number;
    }

    if (_Interactive_data->SuperCapEnergy != _Interactive_data->SuperCapLastEnergy)
    {
        _Interactive_data->Referee_Interactive_Flag.cap_energy_flag = 1;
        _Interactive_data->SuperCapLastEnergy = _Interactive_data->SuperCapEnergy;
    }

    if (_Interactive_data->auto_aim_fire_suggestion != _Interactive_data->auto_aim_fire_last_suggestion)
    {
        _Interactive_data->Referee_Interactive_Flag.auto_aim_fire_suggestion_flag = 1;
        _Interactive_data->auto_aim_fire_last_suggestion = _Interactive_data->auto_aim_fire_suggestion;
    }

    if (_Interactive_data->armor_beat_id != _Interactive_data->armor_beat_id_last)
    {
         // 受击指示线刷新
        switch (_Interactive_data->armor_beat_id)
        {
        case 0:
                beat_direction_continuous_cnt[0]+=10;
            break;
        case 1:
                beat_direction_continuous_cnt[1]+=10;
            break;
        case 2:
                beat_direction_continuous_cnt[2]+=10;
            break;
        case 3:
                beat_direction_continuous_cnt[3]+=10;
            break;
        default:
            break;
        }
        _Interactive_data->Referee_Interactive_Flag.beat_flag = 1;
        _Interactive_data->armor_beat_id_last = _Interactive_data->armor_beat_id;
    }

}