//
// Created by sph on 2024/1/21.
//
#include "auto_aim.h"
#include "arm_math.h"

/**
 * @brief          选择目标装甲板
 * @author         SJQ
 * @param[in]      trajectory_cal:弹道解算结构体
 * @retval         返回空
 */
int aim_armor_select(Aim_Select_Type_t *aim_sel, Trajectory_Type_t *trajectory_cal) {
    aim_sel->delay_time = trajectory_cal->fly_time + trajectory_cal->extra_delay_time;
    aim_sel->target_state.yaw += aim_sel->target_state.v_yaw * aim_sel->delay_time;

    //计算四块装甲板的位置
    int use_1 = 1;
    int i = 0;
    int idx = 0; // 选择的装甲板
    // 为平衡步兵
    if (aim_sel->target_state.armor_num == 2) {
        for (i = 0; i < 2; i++) {
            float tmp_yaw = aim_sel->target_state.yaw + i * PI;
            float r = aim_sel->target_state.r1;

            aim_sel->armor_pose[i].x = aim_sel->target_state.x - r * arm_cos_f32(tmp_yaw);
            aim_sel->armor_pose[i].y = aim_sel->target_state.y - r * arm_sin_f32(tmp_yaw);
            aim_sel->armor_pose[i].z = aim_sel->target_state.z;
            aim_sel->armor_pose[i].yaw = aim_sel->target_state.yaw + i * PI;
        }

//        float yaw_diff_min = fabsf(trajectory_cal->cmd_yaw - aim_sel->armor_pose[0].yaw);
//
//        //因为是平衡步兵 只需判断两块装甲板即可
//        float temp_yaw_diff = fabsf(trajectory_cal->cmd_yaw - aim_sel->armor_pose[1].yaw);
//        if (temp_yaw_diff < yaw_diff_min) {
//            yaw_diff_min = temp_yaw_diff;
//            idx = 1;
//        }
        // 平衡步兵选择两块装甲板中较近的装甲板
        float distance_min = powf(aim_sel->armor_pose[0].x, 2) + powf(aim_sel->armor_pose[0].y, 2);
        float distance_temp = powf(aim_sel->armor_pose[1].x, 2) + powf(aim_sel->armor_pose[1].y, 2);
        if (distance_temp < distance_min) {
            distance_min = distance_temp;
            idx = 1;
        }

    } else if (aim_sel->target_state.armor_num == 3)//前哨站
    {
        for (i = 0; i < 3; i++) {
            float tmp_yaw;
            if (aim_sel->target_state.v_yaw <= 0)
                tmp_yaw = aim_sel->target_state.yaw + i * (2.0 * PI / 3.0);
            else
                tmp_yaw = aim_sel->target_state.yaw - i * (2.0 * PI / 3.0);
            float r = use_1 ? aim_sel->target_state.r1 : aim_sel->target_state.r2;
            aim_sel->armor_pose[i].x = aim_sel->target_state.x - r * cos(tmp_yaw);
            aim_sel->armor_pose[i].y = aim_sel->target_state.y - r * sin(tmp_yaw);
            aim_sel->armor_pose[i].z = use_1 ? aim_sel->target_state.z : aim_sel->target_state.z +
                                                                         aim_sel->target_state.dz;
            aim_sel->armor_pose[i].yaw = aim_sel->target_state.yaw + i * (2.0 * PI / 3.0);
            use_1 = !use_1;
        }

//        //计算枪管到目标装甲板yaw最小的那个装甲板
//        float yaw_diff_min = fabsf(trajectory_cal->cmd_yaw - aim_sel->armor_pose[0].yaw);
//        for (i = 1; i < 3; i++) {
//            float temp_yaw_diff = fabsf(trajectory_cal->cmd_yaw - aim_sel->armor_pose[i].yaw);
//            if (temp_yaw_diff < yaw_diff_min) {
//                yaw_diff_min = temp_yaw_diff;
//                idx = i;
//            }
//        }
        // 选择两块较近的装甲板，再选择两块装甲板与自身位置连线，同旋转中心自身位置连线，夹角较小的为目标装甲板
//         float distance_temp0 = powf(aim_sel->armor_pose[0].x, 2) + powf(aim_sel->armor_pose[0].y, 2);
//         float distance_temp1 = powf(aim_sel->armor_pose[1].x, 2) + powf(aim_sel->armor_pose[1].y, 2);
//         float distance_temp2 = powf(aim_sel->armor_pose[2].x, 2) + powf(aim_sel->armor_pose[2].y, 2);

//        int label_first = 0;
//        int label_second = 1;
//        if (distance_temp0 > distance_temp1) {
//            label_first = 1;
//            if (distance_temp0 > distance_temp2) {
//                label_second = 2;
//            } else label_second = 0;
//        } else {
//            label_first = 0;
//            if (distance_temp1 > distance_temp2) {
//                label_second = 2;
//            } else label_second = 1;
//        }

        // 选择两块较近的装甲板
        float distance[3];
        for (i = 0; i < 3; i++) {
            distance[i] = powf(aim_sel->armor_pose[i].x, 2) + powf(aim_sel->armor_pose[i].y, 2);
        }

        int label_first = 0;
        int label_second = 0;

        for (i = 1; i < 3; i++) {
            if (distance[i] < distance[label_first]) {
                label_second = label_first;
                label_first = i;
            }  else if (distance[i] < distance[label_second] && distance[i] < distance[label_first]) {
                label_second = i;
            }else if (distance[i] < distance[label_second]) {
                label_second = i;
            }
        }

        //再选择两块装甲板与自身位置连线，同旋转中心自身位置连线，夹角较小的为目标装甲板
        float center_length = sqrtf(powf(aim_sel->target_state.x, 2) + powf(aim_sel->target_state.y, 2));

        float cos_theta_first = (aim_sel->armor_pose[label_first].x * aim_sel->target_state.x +
                                 aim_sel->armor_pose[label_first].y * aim_sel->target_state.y) /
                                (sqrtf(powf(aim_sel->armor_pose[label_first].x, 2) +
                                       powf(aim_sel->armor_pose[label_first].y, 2)) * center_length);
        float cos_theta_second = (aim_sel->armor_pose[label_second].x * aim_sel->target_state.x +
                                  aim_sel->armor_pose[label_second].y * aim_sel->target_state.y) /
                                 (sqrtf(powf(aim_sel->armor_pose[label_second].x, 2) +
                                        powf(aim_sel->armor_pose[label_second].y, 2)) * center_length);

        if (cos_theta_first > cos_theta_second)
            idx = label_first;
        else
            idx = label_second;

    } else {
        for (i = 0; i < 4; i++) {
            float tmp_yaw = aim_sel->target_state.yaw + i * PI / 2.0;
            float r = use_1 ? aim_sel->target_state.r1 : aim_sel->target_state.r2;

            aim_sel->armor_pose[i].x = aim_sel->target_state.x - r * arm_cos_f32(tmp_yaw);
            aim_sel->armor_pose[i].y = aim_sel->target_state.y - r * arm_sin_f32(tmp_yaw);
            aim_sel->armor_pose[i].z = use_1 ? aim_sel->target_state.z : aim_sel->target_state.z +
                                                                         aim_sel->target_state.dz;
            aim_sel->armor_pose[i].yaw = tmp_yaw;
            use_1 = !use_1;
        }

//        //计算枪管到目标装甲板yaw最小的那个装甲板
//        float yaw_diff_min = fabsf(trajectory_cal->cmd_yaw - aim_sel->armor_pose[0].yaw);
//        for (i = 1; i < 4; i++) {
//            float temp_yaw_diff = fabsf(trajectory_cal->cmd_yaw - aim_sel->armor_pose[i].yaw);
//            if (temp_yaw_diff < yaw_diff_min) {
//                yaw_diff_min = temp_yaw_diff;
//                idx = i;
//            }
//        }

        // 选择两块较近的装甲板
        float distance[4];
        for (i = 0; i < 4; i++) {
            distance[i] = powf(aim_sel->armor_pose[i].x, 2) + powf(aim_sel->armor_pose[i].y, 2);
        }

        int label_first = 0;
        int label_second = 1;

        if(distance[label_first] > distance[label_second])
        {
            label_first = 1;
            label_second = 0;
        }

        if(distance[2]<distance[label_first]){
            label_second = label_first;
            label_first = 2;
        }
        else if(distance[2]<distance[label_second])
            label_second = 2;


        if(distance[3]<distance[label_first]){
            label_second = label_first;
            label_first = 3;
        }
        else if(distance[3]<distance[label_second])
            label_second = 3;

        //再选择两块装甲板与自身位置连线，同旋转中心自身位置连线，夹角较小的为目标装甲板
        float center_length = sqrtf(powf(aim_sel->target_state.x, 2) + powf(aim_sel->target_state.y, 2));

        float cos_theta_first = (aim_sel->armor_pose[label_first].x * aim_sel->target_state.x +
                                 aim_sel->armor_pose[label_first].y * aim_sel->target_state.y) /
                                (sqrtf(powf(aim_sel->armor_pose[label_first].x, 2) +
                                       powf(aim_sel->armor_pose[label_first].y, 2)) * center_length);
        float cos_theta_second = (aim_sel->armor_pose[label_second].x * aim_sel->target_state.x +
                                  aim_sel->armor_pose[label_second].y * aim_sel->target_state.y) /
                                 (sqrtf(powf(aim_sel->armor_pose[label_second].x, 2) +
                                        powf(aim_sel->armor_pose[label_second].y, 2)) * center_length);

        if (cos_theta_first > cos_theta_second)
            idx = label_first;
        else
            idx = label_second;
    }


    aim_sel->aim_point[0] = aim_sel->armor_pose[idx].x + aim_sel->target_state.vx * aim_sel->delay_time;
    aim_sel->aim_point[1] = aim_sel->armor_pose[idx].y + aim_sel->target_state.vy * aim_sel->delay_time;
    aim_sel->aim_point[2] = aim_sel->armor_pose[idx].z + aim_sel->target_state.vz * aim_sel->delay_time;
    return idx;
}

/**
  * @brief          子弹飞行时间解算
  * @author         SJQ
  * @param[in]      x:瞄准时shooter_link下目标x坐标
  * @param[in]      vx:瞄准时shooter_link下目标x速度
  * @param[in]      v_x0:弹速水平分量
  * @retval         返回空
  */
const float k1 = 0.0761; //标准大气压25度下空气阻力系数
float get_fly_time(float x, float vx, float v_x0) {
    float t = 0;
    float f_ti = 0, df_ti = 0;
    for (int i = 0; i < 5; i++) {
        f_ti = log(k1 * v_x0 * t + 1) / k1 - x - vx * t;
        df_ti = v_x0 / (k1 * v_x0 * t + 1) - vx;
        t = t - f_ti / df_ti;
    }
    return t;
}

/**
 * @brief          解算期望云台角度（考虑子弹飞行时间）
 * @author         SJQ
 * @param[in]      trajectory_cal:弹道解算结构体
 * @retval         返回空
 */
void get_cmd_angle(Trajectory_Type_t *trajectory_cal) {

    arm_power_f32(trajectory_cal->position_xy, 2, &trajectory_cal->dis2);
    arm_sqrt_f32(trajectory_cal->dis2, &trajectory_cal->dis);

    trajectory_cal->dis = trajectory_cal->dis - 0.20;

    trajectory_cal->theta_0 = atan2f(trajectory_cal->z, trajectory_cal->dis);
    trajectory_cal->alpha = atan2f(trajectory_cal->position_xy[1], trajectory_cal->position_xy[0]);
    //将目标的xyz速度转化为平行枪管与垂直枪管的速度
    trajectory_cal->vx = trajectory_cal->velocity[0] * arm_cos_f32(trajectory_cal->alpha)
                         + trajectory_cal->velocity[1] * arm_sin_f32(trajectory_cal->alpha);
    trajectory_cal->vy = -trajectory_cal->velocity[0] * arm_sin_f32(trajectory_cal->alpha)
                         + trajectory_cal->velocity[1] * arm_cos_f32(trajectory_cal->alpha);

    float v_x0 = trajectory_cal->v0 * arm_cos_f32(trajectory_cal->theta_0);//水平方向弹速
    float v_y0 = trajectory_cal->v0 * arm_sin_f32(trajectory_cal->theta_0);//竖直方向弹速

    trajectory_cal->fly_time = get_fly_time(trajectory_cal->dis, trajectory_cal->vx, v_x0);
    arm_power_f32(&trajectory_cal->fly_time, 1, &trajectory_cal->fly_time2);
    trajectory_cal->h_r = trajectory_cal->z + trajectory_cal->velocity[2] * trajectory_cal->fly_time;
    //开始迭代
    trajectory_cal->theta_k = trajectory_cal->theta_0;
    trajectory_cal->k = 0;
    while (trajectory_cal->k < 10) {
        v_y0 = trajectory_cal->v0 * arm_sin_f32(trajectory_cal->theta_k);//竖直方向弹速
        trajectory_cal->h_k = v_y0 * trajectory_cal->fly_time - 0.5 * 9.8 * trajectory_cal->fly_time2;
        trajectory_cal->err_k = trajectory_cal->h_r - trajectory_cal->h_k;
        trajectory_cal->theta_k += 0.1 * trajectory_cal->err_k;
        trajectory_cal->k++;
        if (trajectory_cal->err_k < 0.005) break;
    }

    trajectory_cal->cmd_yaw = atan2f(trajectory_cal->position_xy[1],
                                     trajectory_cal->position_xy[0]);

    trajectory_cal->cmd_pitch = trajectory_cal->theta_k;
}

int auto_aim(Aim_Select_Type_t *aim_sel, Trajectory_Type_t *trajectory_cal, RecievePacket_t *receive_packet) {
    trajectory_cal->extra_delay_time = 0.025;//0.025

    aim_sel->target_state.armor_type = receive_packet->id;
    aim_sel->target_state.armor_num = receive_packet->armors_num;
    aim_sel->target_state.x = receive_packet->x;
    aim_sel->target_state.y = receive_packet->y;
    aim_sel->target_state.z = receive_packet->z;
    aim_sel->target_state.vx = receive_packet->vx;
    aim_sel->target_state.vy = receive_packet->vy;
    aim_sel->target_state.vz = receive_packet->vz;
    aim_sel->target_state.yaw = receive_packet->yaw;
    aim_sel->target_state.v_yaw = receive_packet->v_yaw;
    aim_sel->target_state.r1 = receive_packet->r1;
    aim_sel->target_state.r2 = receive_packet->r2;
    aim_sel->target_state.dz = receive_packet->dz;

    int idx = aim_armor_select(aim_sel, trajectory_cal);

    trajectory_cal->position_xy[0] = aim_sel->aim_point[0];
    trajectory_cal->position_xy[1] = aim_sel->aim_point[1];
    trajectory_cal->z = aim_sel->aim_point[2];
    trajectory_cal->velocity[0] = aim_sel->target_state.vx;
    trajectory_cal->velocity[1] = aim_sel->target_state.vy;
    trajectory_cal->velocity[2] = aim_sel->target_state.vz;

    get_cmd_angle(trajectory_cal);
    return idx;
}
