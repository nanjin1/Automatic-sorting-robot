#include "HWT101_imu.h"
#include "stdio.h"
#include "tim.h"

//#define ABS(X) (((X) > 0) ? (X) : -(X))

u8 gyro_save[10] =     { 0xFF,0xAA,0x00,0x00,0x00};
u8 gyro_rrate[10] =	   { 0xFF,0xAA,0x03,0x09,0x00};
u8 gyro_baud[10] =	   { 0xFF,0xAA,0x04,0x06,0x00};
u8 gyro_unlock[10] =   { 0xFF,0xAA,0x69,0x88,0xB5};
u8 gyro_zero[10] =     { 0XFF,0xAA,0x76,0x00,0x00};
u8 gyro_zeroside[10] = { 0xFF,0xAA,0x48,0x01,0x00};

u8 RxBag[11];
u8 data[6];

attitude_t attitude = 
{
    .raw_yaw = 0,
	.current_yaw = 0,
};

ATK_IMU_t imu =
{
    /*移植时需要修改以下结构体变量即可*/
    .imu_uart = &huart3,        //串口
	.yaw_ptr = &(attitude.raw_yaw), //
    .target_angle = 0,          // pid的目标角度
    .init_angle = 0,            //初始化角度，补偿上电时的初始角度
    .enable_switch = 1,         //使能开关
    .get_angle = Get_Yaw,     //函数指针，返回经过限幅和相差0的角度
	.compensateZ=0
};

void Imu_Init()
{

	/*设置上报内容*/
	//解锁
//	HAL_UART_Transmit(imu.imu_uart,gyro_unlock,sizeof(gyro_unlock[10])+4,65534);
//	while(huart3.gState != HAL_UART_STATE_READY){};
//		
//	//z轴归零
//	HAL_UART_Transmit(imu.imu_uart,gyro_zero,sizeof(gyro_zero[10])+4,0xff);
//	while(huart3.gState != HAL_UART_STATE_READY){};
//	//设置上传速率
//	HAL_UART_Transmit(imu.imu_uart,gyro_rrate,sizeof(gyro_rrate[10])+4,65534);
//	while(huart3.gState != HAL_UART_STATE_READY){};
//	//设置串口波特率
//	HAL_UART_Transmit(imu.imu_uart,gyro_baud,sizeof(gyro_baud[10])+4,65534);
//	while(huart3.gState != HAL_UART_STATE_READY){};
//	//设置零偏模式
//	HAL_UART_Transmit(imu.imu_uart,gyro_baud,sizeof(gyro_baud[10])+4,65534);
//	while(huart3.gState != HAL_UART_STATE_READY){};
//	
//	//保持至Flash
//	HAL_UART_Transmit(imu.imu_uart,gyro_save,sizeof(gyro_save[10])+4,65534);
//	while(huart3.gState != HAL_UART_STATE_READY){};
//	
    //开启串口DMA接收
  HAL_UART_Receive_IT(&huart3,RxBag,sizeof(RxBag));
//	printf("init_ok!\r\n");
	
	
}

//使用串口DMA获得数据
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance==USART3)
	{
		Data_Rebuild(RxBag);
	}
    HAL_UART_Receive_IT(&huart3,RxBag,sizeof(RxBag));
}
float diff;

void Data_Rebuild(u8* RxBag)
{
	if(RxBag[0] == 0x55)
	{
		uint8_t sum = 0;
		for (int i=0; i<10; i++) sum += RxBag[i];
		if (sum == RxBag[10])
		{
			if (RxBag[2] == 0X01)
			{
				attitude.raw_yaw = 180.0 * (short) ((RxBag[9]<<8)|RxBag[8])/32768.0;
			}
		}
	}
	//printf("RAW_YAW:%.2f\r\n",attitude.raw_yaw);
	//printf("CURRENT:%.2f,RAW_YAW:%.2f\n",Get_Yaw(),attitude.raw_yaw);
	//printf("%f\n",Get_Yaw());
}

float Get_Yaw(void)
{
    return attitude.raw_yaw;
}

//设置当前角度为零度
void Set_InitYaw(float target)
{
	//利用数学方法求得一个参考值
	Set_IMUStatus(0);
//	imu.compensateZ =angle_limit(0 - imu.get_angle());
//	imu.target_angle += imu.compensateZ;
	imu.target_angle = angle_limit(imu.get_angle());
	Set_IMUStatus(1);
}

float angle_limit(float angle)
{
	//参考角度是我们通过计算算出来的，所以要对角度进行限制
	limit_label:
    while (angle > 180) angle -= 360;
    while (angle < -180) angle += 360;
    if (ABS(angle) > 180) goto limit_label;
    return angle;
}

//使能状态改变
void Set_IMUStatus(int status)
{
	imu.enable_switch = status;
}

//uint8_t	infrare_open=0;

//volatile struct Infrared_Sensor infrared;

//void get_Infrared(void){     //碰到障碍物时为1
//	infrared.outside_right = !(uint8_t)HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_9);
//	infrared.inside_right = !(uint8_t)HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_15);
//	
//	infrared.inside_left = !(uint8_t)HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_13);
//	infrared.outside_left = !(uint8_t)HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_14);
//}
