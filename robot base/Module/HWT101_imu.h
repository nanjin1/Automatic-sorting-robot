#pragma once

#include "usart.h"
#include "main.h"
#include "time_cnt.h"
#include "motor.h"
#include "uart_handle.h"
#include "stm32f7xx_hal_tim.h"
typedef uint32_t  u32; //32?
typedef uint16_t u16;  //16?
typedef uint8_t  u8;   //8?



typedef struct
{
    float raw_yaw;
    float current_yaw;
} attitude_t;

typedef struct {
    UART_HandleTypeDef * imu_uart; //串口句柄
    float target_angle; //底盘目标角度
    u8 enable_switch; //底盘保持目标角度开关
    float (*get_angle)(void); //获取当前角度
	//以下目前没用上
	
	float compensateZ;
	volatile float* yaw_ptr;
    float init_angle; //初始化角度，补偿上电时的初始角度
}ATK_IMU_t;

extern attitude_t attitude;
extern ATK_IMU_t imu;
extern u8 RxBag[11];
extern u8 data[6];

void Imu_Init(void);
void Data_Rebuild(u8* RxBag);
float Get_Yaw(void);
void Set_InitYaw(float angle);
float angle_limit(float angle);
void Set_IMUStatus(int status);
void get_Infrared(void);

struct Infrared_Sensor    //红外传感器
{
	uint8_t Left_1;
	uint8_t outside_right;
	uint8_t inside_left;
	uint8_t inside_right;
};




