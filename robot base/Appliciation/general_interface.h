/*** 
 * @Author: peach 1831427532@qq.com
 * @Date: 2022-09-05 14:14:24
 * @LastEditors: peach 1831427532@qq.com
 * @LastEditTime: 2022-09-22 12:46:57
 * @FilePath: \MDK-ARMd:\robot\robot\Appliciation\general_interface.h
 * @Description: 
 * @
 * @Copyright (c) 2022 by peach 1831427532@qq.com, All Rights Reserved. 
 */
/***
 * @Author: peach 1831427532@qq.com
 * @Date: 2022-09-05 14:14:24
 * @LastEditors: peach 1831427532@qq.com
 * @LastEditTime: 2022-09-14 15:06:28
 * @FilePath: \MDK-ARMd:\robot\robot\Appliciation\general_interface.h
 * @Description:
 * @
 * @Copyright (c) 2022 by peach 1831427532@qq.com, All Rights Reserved.
 */
#ifndef __GENERAL_INTERFACE_H_
#define __GENERAL_INTERFACE_H_

#include "motor.h"
#include "chassis.h"
#include "cmsis_os.h"
#include "chassis_control.h"
#include "track_bar_receive.h"
#include "imu_pid.h"
#include "HWT101_imu.h"
#include "read_status.h "
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"
#include "servo.h"
#include "openmv.h"

void Move_CountBar(int id, int num, int speed);
void Move_inch(int dir1,int dir2,int time);
void blue_all(void);


int Get_HW_NEW(void);
int Get_HW_NEW1(void);
int Get_HW_LEFT(void);
int Get_HW_RIGHT(void);
int Get_Home1(void);
int Get_Home(void);
void centern_control(int color);
void control_ball(int ball_color);
void Dump_ball(int color);
void Dump_block(void);
void Dump_RoundRing(int color);
void Restart_Ladder(void);
void run_ladder(int color);
void run_blue(void);
void run_red(void);
void run_home(int color);
void run_round(int color);
#endif
