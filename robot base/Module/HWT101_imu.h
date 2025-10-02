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
    UART_HandleTypeDef * imu_uart; //���ھ��
    float target_angle; //����Ŀ��Ƕ�
    u8 enable_switch; //���̱���Ŀ��Ƕȿ���
    float (*get_angle)(void); //��ȡ��ǰ�Ƕ�
	//����Ŀǰû����
	
	float compensateZ;
	volatile float* yaw_ptr;
    float init_angle; //��ʼ���Ƕȣ������ϵ�ʱ�ĳ�ʼ�Ƕ�
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

struct Infrared_Sensor    //���⴫����
{
	uint8_t Left_1;
	uint8_t outside_right;
	uint8_t inside_left;
	uint8_t inside_right;
};




