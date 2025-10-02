/*
 * @Author: rlantic
 * @Date: 2022-03-20 15:29:29
 * @LastEditors: OBKoro1
 * @LastEditTime: 2022-04-06 14:03:25
 * @FilePath: \MDK-ARMd:\program\motor\motor\TASK\read_status.h
 * @Description: 
 * 
 * Copyright (c) 2022 by 用户/公司名, All Rights Reserved. 
 */
/************************************************************************
 *
 * FileName   : read_status.h
 * Version    : v1.0
 * Author     : 妗冨瓙
 * Date       : 2021-09-12
 * Description:
 *******************************************************************************/

#ifndef __READ_STATUS_H_
#define __READ_STATUS_H_

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

#include <string.h>
#include <stdbool.h>

#include "main.h"
typedef enum
{
  err = -1,
  off = 0,
  on = 1,
} status_;

typedef enum
{
  Primary_Head = 0,
  Low_Head = 1,
  Medium_Head
} ScanDir_t;

typedef enum
{
  PrimaryHeight = 0,
  LowestHeight,
  MediumHeight,
  HighestHeight,
} Height_t;

typedef enum
{
  toMedium  = 21,
  toHighest = 21,
  toLowest  = 21,
} QR_HeightVal_t;
typedef enum
{
  Not_Running = 0,
  Red_,
  Blue_,
} Game_Color_t;


int Get_Switch_Status(int id);
int Get_HW_Status(int id);
void Start_Read_Switch(void);
void Exit_Swicth_Read(void);
void Wait_Switches(int dir);
void Single_Switch(int switch_id);
void HWSwitch_Move(int dir, int enable_imu); //使用红外开关进行移动
void Set_SwitchParam(int main, int vertical);
void MV_HW_Scan(int color, int dir, int enable_imu);
void Brick_QR_Mode(int dir, int color, int QR, int imu_enable);
void Start_HeightUpdate(void);
void QR_Scan(int status, int color, int dir, int enable_imu);
void Kiss_Ass(int dir, int enable_imu);
int Get_Height_Switch(int id);
int Get_MV_Servo_Flag(void);
int Get_Height(void);
//只在内部使用的函数
void Inf_Servo_Height(int now_height);


//限制高度信息读取的软件开关
bool Get_HeightAvailable(void);
void Set_HeightAvailable(bool Switch_Status);


//阶梯平台机械臂归位
void begin_servo_status(int flag);


void Set_SwitchSpeed(int speed);
void Judge_Side(int dir);
void Ring_Move(void); //放置圆环时使用
void Disc_Mea(void);  //圆盘机的实现

void Trans_Cons(int val1,int val2);

int Get_Hight_data(void);
void Change_Hight(void);

int Get_target_color(void);

void Ladder_process(int color, int imu_enable,int mv_open);

int change_second(void);

void change_labber_v(int speed);

//实现开关时间的自定义修改
uint16_t Get_SwitchTimeThreshold(void);
void Set_SwitchTimeThreshold(uint16_t TimeVal);
void Wait_Switches_Time(int dir,uint16_t TimeThreshold);

void Set_different_flag(int flag);

void run_fast(void);
void Set_run_fast_mode(int flag);
void Set_different1_flag(int flag);
void Set_once(int flag);
int Get_once(void);
void Set_MV_next(bool flag);
bool Get_MV_next(void);

void Set_Change_Hight_Status(bool flag);


int Get_Hw(int No);
#endif
