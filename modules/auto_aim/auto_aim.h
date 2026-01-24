//
// Created by sph on 2024/1/21.
//

#ifndef BASIC_FRAMEWORK_AUTO_AIM_H
#define BASIC_FRAMEWORK_AUTO_AIM_H

#include "master_process.h"
//弹道解算
typedef  struct
{
    float v0;        //子弹射速
    float velocity[3];//目标xyz速度
    float vx;         //目标相对枪口方向的速度
    float vy;
    float alpha;      //目标初始航向角
    float position_xy[2];//目标xy坐标
    float z;         //目标z坐标
    float fly_time;  //子弹飞行时间
    float fly_time2;  //子弹飞行时间平方
    float extra_delay_time ;
    float theta_0;   //初始目标角度
    float theta_k;   //迭代目标角度
    float dis;       //目标距离
    float dis2;       //目标距离平方
    float err_k;      //迭代误差
    uint8_t k;      //迭代次数
    float h_k;       //迭代高度
    float h_r;       //目标真实高度

    float cmd_yaw;
    float cmd_pitch;
} Trajectory_Type_t;

//整车状态
typedef struct
{
    float x;
    float y;
    float z;
    float yaw;
    float vx;
    float vy;
    float vz;
    float v_yaw;
    float r1;
    float r2;
    float dz;
    uint8_t armor_type;
    uint8_t armor_num;
}Target_State_Type_t;

//预瞄点
typedef struct
{
    float x;
    float y;
    float z;
    float yaw;
}Armor_Pose_Type_t;

typedef struct
{
    Target_State_Type_t target_state; //整车状态
    Armor_Pose_Type_t armor_pose[4]; //四个装甲板状态
    float aim_point[3];                //预瞄点
    float delay_time; //预瞄时间差
    uint8_t suggest_fire;
}Aim_Select_Type_t;


int aim_armor_select(Aim_Select_Type_t *aim_sel, Trajectory_Type_t *trajectory_cal);
float get_fly_time(float x, float vx, float v_x0);
void get_cmd_angle(Trajectory_Type_t *trajectory_cal);
int auto_aim(Aim_Select_Type_t *aim_sel,Trajectory_Type_t *trajectory_cal,RecievePacket_t *receive_packet);

#endif //BASIC_FRAMEWORK_AUTO_AIM_H
