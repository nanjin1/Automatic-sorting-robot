#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "main.h"
#include "chassis.h"
#include "chassis_control.h"
#include "motor.h"
#include "imu_pid.h"
#include "read_status.h "
#include "avoid_obs.h"
#include "HWT101_imu.h"
#include "track_bar_receive.h"
#include "servo.h"
#include "openmv.h"
#include "general.h"
#include "general_interface.h"
#include "motor.h"
#define DUBUG_MOTOR 0
int if_OsRunning(void);
void Set_OSRunningFlag(int status);

// todo  出发前做好检查
#define Blue_Route 1
#define Red_Route 0
#define Wait_Dealy_MAX 10000
#define Line_Type 1
#define Encoder_Type 2
#define BOO_Type 3
#define some 1

// todo 在这里定义红蓝半场
int Os_RunningFlag = 0;
uint8_t cmd[3] = {0xff, 0x00, 0x99};
bool if_debug_finished = false;

#define IF_Run 1



void Startdebug(void const *argument)
{
	osDelay(1500);
	set_chassis_status(1);     

	
	while(1)
	{
	//set_debug_motor(1,2);
	//set_speed(0,100,0);
	//motor_debug();
	//Move_inch(0,100,3000);
	//osDelay(1000);
	//Move_inch(0,-100,3000);
	//osDelay(1000);	
	//set_motor(3, 10000);
		//show_speed();
		//printf("%d,%d\r\n",Get_HW_LEFT(),Get_HW_RIGHT());
		//Move_inch(0,40,1000);
		
	}
	
	
	
	
	
	
   	
  //Wait_OKInf(Encoder_Type, Wait_Dealy_MAX);

     while(1)
	 {
//	 HAL_UART_Transmit(&huart2,(uint8_t *)0x44, 1, 0xff); //将cmd发送出去
//		 printf("1111");
//	if(Get_Home2() == 0)
//	{
//	
//		set_speed(0,0,0);
//		break;
//	}
//	else if(Get_Home2() == 1)
//	{
//	
//	set_speed(0,30,0);
//	
//	}		
//		 osDelay(10);
		 
//		printf("====%d=======%d\r\n",Get_HW_LEFT(),Get_HW_RIGHT());
//		 printf("====%dGet_US_Distance\r\n",Get_US_Distance());
		 osDelay(5);
//		 osDelay(10);
//		
//	  if(Get_HW(0)==1)
//	  {
//		  HAL_GPIO_WritePin(FengFeng_GPIO_Port,FengFeng_Pin,GPIO_PIN_RESET);
//		  osDelay(10);
//		  HAL_GPIO_WritePin(FengFeng_GPIO_Port,FengFeng_Pin,GPIO_PIN_SET);
//		run_red();
//	  break;
//	  }
//	  if(Get_HW(2)==1)
//		 {
//			HAL_GPIO_WritePin(FengFeng_GPIO_Port,FengFeng_Pin,GPIO_PIN_RESET);
//		  osDelay(10);
//		  HAL_GPIO_WritePin(FengFeng_GPIO_Port,FengFeng_Pin,GPIO_PIN_SET);
//	 run_blue();
//			break;
//		}
//	 }    

//		if(Get_HW(1))
//		blue_all();
	
//		Turn_angle(1, -90, 0)2;
//		printf("hhhhhhhhhh\n");
//		Turn_angle(1, 90, 0);
//		osDelay(500);
	
    
//	 int all_game=1;
//    Set_OSRunningFlag(true);
//     
//    //   motor_debug();
//    //红场是4号跟6号
//    printf("正在等待指定开关的触发\n");
//	 //move_by_encoder(2,30);
//    while (1)
//    {
//		while ((1))
//        {
//            motor_debug();
//        }
        
//		 if(all_game)
//       {       
//			 if ((Get_SW(3) || Get_SW(5)) && (Get_SW(4) || Get_SW(6)))
//			  {
//					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
//					osDelay(2000);
//					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
//					blue_all();
//					break;
//			  }
//			  else if ((Get_SW(2) && Get_SW(4)) || (Get_HW(2) && (Get_SW(2) || Get_SW(4))))
//			  {
//					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
//					osDelay(2000);
//					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
//					Run4WholeGame(5);
//					break;
//			  }
//			  else
//					osDelay(10);
//	    }
//		 else
//			 osDelay(10);
//    }
//#ifdef Run_Red
//    while (!(Get_SW(3) && Get_SW(5)))
//        osDelay(10);
//    RedGame2Test(5);
//#endif
//#ifdef Run_Blue
//    while (!(Get_SW(2) && Get_SW(4)))
//        osDelay(10);
//    Run4WholeGame(5);
//#endif
//    osDelay(100);
//    printf("结束\n");
//    vTaskDelete(NULL);
}
	 }

void Global_Debug(void)
{
    printf("finish \n");
}
void Go_Home(int color)
{
    if (color != 1 && color != 2)
        return;
    move_by_encoder(2, -80);
    if (color == 1)
        Turn_angle(1, 90, 0);
    else
        Turn_angle(1, -90, 0);
    // direct_move(2, 2, 0, 1);
    move_by_encoder(2, 20);
}
int if_OsRunning(void)
{
    return Os_RunningFlag;
}
void Set_OSRunningFlag(int status)
{
    Os_RunningFlag = status;
}

void Goto_Warehouse(void)
{
    ;
}

osThreadId GameTaskHandle = NULL;            //任务句柄
void GameTaskTaskFunc(void const *argument); //任务实现函数
bool GameTaskTask_Exit = 1;                  //是否退出

void Game_On(void)
{
    if (GameTaskTask_Exit && GameTaskHandle == NULL)
    {
        GameTaskTask_Exit = 0;
        osThreadDef(GameTask, GameTaskTaskFunc, osPriorityHigh, 0, 1024); //定义任务结构体
        GameTaskHandle = osThreadCreate(osThread(GameTask), NULL);        //创建任务
    }
}
void GameTaskTaskFunc(void const *argument)
{
    while (!GameTaskTask_Exit)
    {
        Global_Debug();
    }
    GameTaskHandle = NULL;
    vTaskDelete(NULL);
}
