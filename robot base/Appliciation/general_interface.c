
#include "general_interface.h"
#include "read_status.h "
#include "openmv.h"
#define Wait_Dealy_MAX 15000
#define Line_Type 1
#define Encoder_Type 2
#define YuanPanDelay  10000
#define Nomal 0
#define Rightside 1
#define Leftside 2
#define Low 190
#define High 200

int color = 0;



int flag_pillars = 0;


/**
 * @description: 通过调用红外俩计算板子数量
 * @param {int} id使用哪个红外
 * @param {int} num 数多少个板子
 * @param {int} speed以什么速度运行
 * @return {*}                              可以用，但没必要
 */
void Move_CountBar(int id, int num, int speed)
{
    if (id != 0 && id != 2 && id != 9)
    {
    //    printf("选择的红外id有误");while ()
    {
    }
        return;
    }
    if (id != 9)
    {
        Init_BarCount(id);
        set_imu_status(1);      // TODO 确认陀螺仪开启状态
        set_speed(0, speed, 0); // TODO 传参形式设置速度
        while (Get_BarCount(id) < num)
        {
            osDelay(5);
        }
        osDelay(50);
    }
    else
    {
        // id = 2;
        Init_BarCount(id);
        set_imu_status(1);      // TODO 确认陀螺仪开启状态
        set_speed(0, speed, 0); // TODO 传参形式设置速度
        while (Get_BarCount(id) < num)
        {
            osDelay(5);
        }
        osDelay(200);
    }
    set_speed(0, 0, 0);
    osDelay(100);
}




//延时走路函数，斜走什么的函数都在这里
void Move_inch(int dir1,int dir2,int time)
{
	int temp_time = 50;
	set_speed(dir1 / 3, dir2 / 3, 0);
	osDelay(250);
	set_speed(dir1,dir2,0);
	
	osDelay(50);
	while(temp_time < time)               //osDelay函数的不足以支持长时间的延时，所以就简单实现一个更准确的延时
	{
		temp_time+=5;
		osDelay(5);	
	}
	set_speed(0,0,0);
}



//负责左边的循迹模块读取
int Get_Home1(void)
{
	if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_2)==RESET)
		return 1;
    else
		return 0;

}
/****************************************
 *
 *
 *	循迹模块回家
 *
 ****************************************/
//负责前面的循迹模块读取
int Get_Home2(void)
{
	if(HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_13)==RESET)
		return 1;
    else
		return 0;

}

/****************************************
 *	调车注意事项：
 *调好立桩的关键在于调整好两个红外的位置
 *其次是调整绕园的速度/角度等等
 *
 ****************************************/

//立桩左边红外
int Get_HW_LEFT(void)
{
	return Get_Hw(8);

}
//立桩右边红外
int Get_HW_RIGHT(void)
{
	return Get_Hw(9);

}

/****************************************
 *	走到圆盘机后定位中心的一整套动作
 *	随着版本的更新，我们最终还是选择了最简单粗暴的一种办法
 *	调车注意事项
 *	电池电量，以及具体的延时时间
 ****************************************/

void centren_control(int color)
{
	if(color == 0)
	{
		set_imu_param(100,100,0);
		Move_inch(100,160,1500);
		ActionGroup(81,1);
		Move_inch(0,180,1800);
		ActionGroup(2, 1); //改变存储方向
		set_imu_param(60,33,0);
		while(Get_Hw(1)==0)
		{
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
			set_speed(0,110,0);
		}
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
		set_speed(0,0,0);
		osDelay(70);
		set_imu_status(0);
		Move_inch(50, 0, 1500);
	}
	//红场的代码
	else
	{
		//set_imu_param(100,100,0);
		Move_inch(110,-160,1500);
		ActionGroup(81,1);
		Move_inch(0,-200,2200);
		ActionGroup(2, 1); //改变存储方向
		//set_imu_param(80,33,0);
		while(Get_Hw(2)==0)
		{
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
			set_speed(0,-120,0);
		}
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
		osDelay(45);
		set_speed(0,0,0);
		osDelay(70);
		set_imu_status(0);
		Move_inch(60, -10, 1500);
	}
	Set_InitYaw(0);
}

/****************************************
 *	抓球的一系列动作
 *	0，1，2 红黄蓝
 *	细节就需要需要自己把控了
 *
 ****************************************/

void control_ball(int ball_color)
{
	set_speed(0,0,0);
	ActionGroup(82, 1); //从折叠状态升起机械臂,需要调整
    Wait_Servo_Signal(Wait_Dealy_MAX);

	ActionGroup(2, 1); //改变存储方向
	Wait_Servo_Signal(Wait_Dealy_MAX);
	
	for(uint8_t i=0;i<3;i++)
	{
		MV_SendCmd(1, 0); //使能mv
		osDelay(50);
		MV_SendCmd(2, 1);
		osDelay(50);
	}
	osDelay(280);
	Set_QueryState(1);
	Set_FirstFlag(0);
	
	int timeout_cnt = 0;
	while(1)
	{
		if(Get_FirstFlag()==1)
		{
			ActionGroup(83, 1);//bo
			Wait_Servo_Signal(Wait_Dealy_MAX);
			ActionGroup(82, 1);
			osDelay(200);
			Set_FirstFlag(0);
		}
		timeout_cnt++;
		osDelay(10);
		if(timeout_cnt>=1300) break;
   	}

	Set_QueryState(0);
	osDelay(100);
	Enable_ServoFlag(); //拉高标志位(1);
	
	ActionGroup(3, 1); //改变存储方向
    Wait_Servo_Signal(Wait_Dealy_MAX);
	for(uint8_t i=0;i<3;i++)
	{
		MV_SendCmd(1, 0); //使能mv
		osDelay(50);
		MV_SendCmd(2, ball_color); //设置(红/蓝)目标球
		osDelay(50);
	}
	osDelay(280);
	Set_QueryState(1);
	Set_FirstFlag(0);
	timeout_cnt = 0;
	while(1)
	{
		if(Get_FirstFlag()==1)
		{
			ActionGroup(83, 1);//bo
			Wait_Servo_Signal(Wait_Dealy_MAX);
			ActionGroup(82, 1);
			osDelay(200);
			Set_FirstFlag(0);
		}
		timeout_cnt++;
		osDelay(10);
		if(timeout_cnt>=1300) break;
	}
    Set_QueryState(0);
	osDelay(280);
	Enable_ServoFlag(); //拉高标志位(1);
    MV_SendCmd(0, 0);
	Move_inch(60, 0, 800);
	Set_InitYaw(0);

    ActionGroup(81, 1); //从折叠状态升起机械臂
    Wait_Servo_Signal(Wait_Dealy_MAX);
	set_imu_status(1);
    osDelay(500);
}
/****************************************
 *	倒球定位
 *
 *	避障直接写死
 *	
 ****************************************/
void Dump_ball(int color)
{
	set_imu_status(1);
	if(color == 0) //蓝区
	{
		ActionGroup(0, 1);//收起机械臂
		Move_inch(-100,0,1000);
		Wait_Servo_Signal(Wait_Dealy_MAX);
		osDelay(500);
		Turn_angle(1,180,0);
		Move_inch(0,60,250);	
		Move_inch(-80,0,1100);
		Move_inch(0,120,2200);

		//撞板子修正姿态
		set_imu_status(0);
		set_speed(-70,0,0);
		osDelay(1000);
		set_speed(0,0,0);
		for(uint8_t i=0;i<3;i++)
		{
			Set_InitYaw(0);
			osDelay(50);
		}
		set_imu_status(1);
		
		ActionGroup(35,1);
		Wait_Servo_Signal(Wait_Dealy_MAX);
		set_speed(0,30,0);
		while(1)
		{
			if(Get_Hw(7)==0)
			{
				set_speed(0, 0, 0);
				printf("我找到属于球的B区了\r\n");
				break;
			}
		}
		ActionGroup(4,1);
		Wait_Servo_Signal(Wait_Dealy_MAX);
		osDelay(2000);
		ActionGroup(5,1);
		Wait_Servo_Signal(Wait_Dealy_MAX);
		ActionGroup(36,1);
		Wait_Servo_Signal(Wait_Dealy_MAX);
		for(uint8_t i=0;i<3;i++)
		{
			Set_InitYaw(0);
			osDelay(50);
		}
		set_imu_status(1);
	}
	else //红区
	{
		ActionGroup(0, 1);//收起机械臂
		osDelay(500);
		Move_inch(-90,0,1200);
		Wait_Servo_Signal(Wait_Dealy_MAX);
		osDelay(500);
		Turn_angle(1,180,0);
		Move_inch(0,-60,250);	
		Move_inch(-80,0,1000);
		Move_inch(0,-120,2100);

		//撞板子修正姿态
		set_imu_status(0);
		set_speed(-70,0,0);
		osDelay(1800);
		set_speed(0,0,0);
		for(uint8_t i=0;i<3;i++)
		{
			Set_InitYaw(0);
			osDelay(50);
		}
		set_imu_status(1);
		
		ActionGroup(35,1);
		Wait_Servo_Signal(Wait_Dealy_MAX);
		set_speed(-10,30,0);
		while(1)
		{
			if(Get_Hw(7)==0)
			{
				set_speed(0, 0, 0);
				printf("我找到属于球的B区了\r\n");
				break;
			}
		}
		ActionGroup(4,1);
		Wait_Servo_Signal(Wait_Dealy_MAX);
		osDelay(2000);
		ActionGroup(5,1);
		Wait_Servo_Signal(Wait_Dealy_MAX);
		ActionGroup(36,1);
		Wait_Servo_Signal(Wait_Dealy_MAX);
		for(uint8_t i=0;i<3;i++)
		{
			Set_InitYaw(0);
			osDelay(50);
		}
		set_imu_status(1);
	}
}
//倒积木块定位

void Dump_block(void)
{
	set_imu_status(0);
	Move_inch(0,50, 500);
	Move_inch(-30,0, 800);
	Set_InitYaw(0);
	set_imu_status(1);

	ActionGroup(35,1);//积木块动作组红外	
	Wait_Servo_Signal(Wait_Dealy_MAX);
	set_speed(-10,38,0);
	//osDelay(500);
	while(1)
	{
		
        if(Get_Hw(7)==0)
		{
			set_speed(0, 0, 0);
		
			break;
		}
	}
	ActionGroup(4,1);
	Wait_Servo_Signal(Wait_Dealy_MAX);
	osDelay(2000);
	ActionGroup(5,1);
	Wait_Servo_Signal(Wait_Dealy_MAX);
	
	ActionGroup(34,1);//积木块动作组红外
	Wait_Servo_Signal(Wait_Dealy_MAX);
	set_speed(-12,-60,0);
	while(1)
	{
        if(Get_Hw(7)==1)
		{
			set_speed(0, 0, 0);
	//		printf("我找到属于积木的B区了\r\n");
			break;
		}
	}
	ActionGroup(6,1);
	Wait_Servo_Signal(Wait_Dealy_MAX);
	ActionGroup(5,1);
	Wait_Servo_Signal(Wait_Dealy_MAX);
	ActionGroup(36,1);
	Wait_Servo_Signal(Wait_Dealy_MAX);
}

	
	
//放码垛定位
void Dump_RoundRing(int color)
{
	if(color == 0)//蓝色
	{
		set_imu_status(1);
		ActionGroup(1,1);
		Move_inch(0,130,450);
		Move_inch(130,0,2050);

		osDelay(1000);
		Turn_angle(1,180,0);//转到正确的角度
		osDelay(1000);
		Move_inch(0, 100, 1100);
		
		ActionGroup(52,1);
		Wait_Servo_Signal(Wait_Dealy_MAX);
		//撞板修正
		set_imu_status(0);
		Move_inch(-60, 0, 1500);
		osDelay(200);
		Set_InitYaw(0);
		set_imu_status(1);
		
		set_speed(-12,-30,0);
		while(1)
		{
			if(Get_Hw(3)==0)
			{
				set_speed(0, 0, 0);
				printf("我找到属于蓝码垛的区域了\r\n");
				break;
			}
		}
		ActionGroup(62,1);
		Wait_Servo_Signal(30000);
	}
	else //红区
	{
		set_imu_status(1);
		ActionGroup(1,1);
		Move_inch(0,130,450);
		Move_inch(130,0,2300);
		
		osDelay(1000);
		Turn_angle(1,180,0);//转到正确的角度
		osDelay(1000);
		set_speed(0,120,0);
		osDelay(1400);
		set_speed(0,0,0);

		ActionGroup(52,1);
		Wait_Servo_Signal(Wait_Dealy_MAX);
		//撞板修正
		set_imu_status(0);
		set_speed(-70,0,0);
		osDelay(1000);	
		set_speed(0,0,0);
		set_speed(-50,0,0);//装板子修正姿态
		osDelay(1000);	
		set_speed(0,0,0);
		osDelay(500);
		for(uint8_t i=0;i<3;i++)
		{
			Set_InitYaw(0);
			osDelay(50);
		}
		set_imu_status(1);

		set_speed(-10,-30,0);
		while(1)
		{
			if(Get_Hw(3)==0)
			{
				set_speed(0, 0, 0);
				printf("我找到属于红码垛的区域了\r\n");
				break;
			}
		}
		ActionGroup(62,1);
		Wait_Servo_Signal(30000);
	}
}

//重新来一次阶梯平台这次抓取积木块
	
void Restart_Ladder()
{
	ActionGroup(52,1);
	Wait_Servo_Signal(Wait_Dealy_MAX);
	Move_inch(0,-130,800);
	set_speed(-75,0,0);
	osDelay(1000);
	
	set_speed(0,-60,0);
	
	while(1)
	{
		if(Get_Hw(4)==0)
		{
			set_speed(0,0,0);
		//	printf("2222");
			break;
		}
	}
	
	set_speed(0,40,0);
	
	while(1)
	{
		if(Get_Hw(4)==1)
		{
			set_speed(0,0,0);
		//	printf("2222");
			break;
		}
	}
	set_speed(-75,0,0);
	osDelay(1000);
	
	ActionGroup(53,1);
	Wait_Servo_Signal(Wait_Dealy_MAX);
}


void run_ladder(int color)
{
	//从倒球的仓库走到阶梯平台
	if(color == 0) //蓝区
	{
		Move_inch(0,100,1000);
		ActionGroup(81,1);//最高处扫描防止纠正姿态的时候撞到阶梯平台
		Move_inch(100,0,2200);	
		Turn_angle(1,180,0);
		ActionGroup(52,1);//阶梯平台最高处扫描
		Move_inch(0,100,1000);
		//撞板修正
		set_imu_status(0);
		Move_inch(-65, 0, 1800);
		Set_InitYaw(0);
		set_imu_status(1);

		Wait_Servo_Signal(Wait_Dealy_MAX);
		set_speed(-20,-48,0);
		while(1)
		{
			if(Get_Hw(4)==0)
			{
				set_speed(0,0,0);
				break;
			}
		}
		Wait_Servo_Signal(Wait_Dealy_MAX);
		// set_motor_pid(90,200,9);
		Ladder_process(1,0,1);//开始第一次阶梯平台
		// Restart_Ladder();
		// set_motor_pid(90,100,9);
		// Ladder_process(1,1,1);
	}
	else //红区
	{
		Move_inch(0,100,1000);
		ActionGroup(81,1);//最高处扫描防止纠正姿态的时候撞到阶梯平台
		Move_inch(100,0,2300);	
		Turn_angle(1,180,0);
		osDelay(1000);
		ActionGroup(52,1);//阶梯平台最高处扫描
		Move_inch(0,100,1000);
		//撞板修正
		set_imu_status(0);
		Move_inch(-65, 0, 1600);
		Set_InitYaw(0);
		set_imu_status(1);
		
		Wait_Servo_Signal(Wait_Dealy_MAX);
		set_speed(-9,-48,0);
		while(1)
		{
			if(Get_Hw(4)==0)
			{
				set_speed(0,0,0);
				break;
			}
		}

		// set_motor_pid(90,200,9);
		Ladder_process(0,0,1);//开始第一次阶梯平台
		// Restart_Ladder();
		// set_motor_pid(90,100,9);
		// Ladder_process(1,1,1);
	}
}



void run_home(int color)
{
	extern int GO_home;
	if(color == 0)
	{
		GO_home = 1;
		Move_inch(-30, 0, 500);
		 Set_InitYaw(0);
		 osDelay(500);
		 // ActionGroup(10,1);
		 // Wait_Servo_Signal(Wait_Dealy_MAX);
		 move_by_encoder(2, 640);
		 Wait_OKInf(Encoder_Type, Wait_Dealy_MAX);
		 Turn_angle(1, -90, 0);
		 osDelay(500);
		 move_by_encoder(2, 220);
		 Wait_OKInf(Encoder_Type, Wait_Dealy_MAX);

		 // Turn_angle(2,-90,0);

		 // int i = 0;
		 // while(1)
		 // {
		 // 	Start_home:
		 // 	set_speed(-90,0,0);

		 // 	if(Get_Home1()==0)
		 // 	{
		 // 		set_speed(0,0,0);
		 // 		Turn_angle(2,-90,0);
		 // 		osDelay(100);
		 // 		if(Get_Home1()==1)goto Start_home;

		 // 		while(1)
		 // 		{

		 // 		Move_inch(90,0,150);
		 // 		Turn_angle(2,-90,0);

		 // 		if(Get_Home1()==1)
		 // 			{
		 // 				Move_inch(90,0,220);
		 // 				Turn_angle(2,-90,0);
		 // 				break;
		 // 			}

		 // 		}

		 // 		set_speed(0,0,0);
		 // 		break;

		 // 	}
		 // }

		 // Turn_angle(2,-90,0);
		 // 	set_speed(0,30,0);

		 // while(1)
		 // {
		 // 	if(Get_Home2()==0)
		 // 	{
		 // 		set_speed(0,0,0);
		 // 		set_speed(0,-49,0);
		 // 		osDelay(280);
		 // 		set_speed(0,0,0);
		 // 		break;
		 // 	}
		 // }

		 // Turn_angle(2,-90,0);
		 // osDelay(2000);	
		
		
	}
	else
	{
		GO_home = 1;
		set_speed(-60,0,0);//撞板子修正姿态
		osDelay(1000);
	 	for(uint8_t i=0;i<3;i++)
		{
			Set_InitYaw(0);
			osDelay(50);
		}
		set_speed(-12,-40,0);
		osDelay(1000);
		set_speed(0,0,0);
	while(1)
	{
		
        if(Get_Hw(3)==0)
		{
			set_speed(0, 0, 0);
			
			break;
		}
	}
		ActionGroup(10,1);
		Wait_Servo_Signal(Wait_Dealy_MAX);
			
		move_by_encoder(2,-620);
		Wait_OKInf(Encoder_Type, Wait_Dealy_MAX);
		Turn_angle(2,-90,0);
		osDelay(1500);
	
		move_by_encoder(2,200);
		Wait_OKInf(Encoder_Type, Wait_Dealy_MAX);
		Turn_angle(2,-90,0);
//	osDelay(000);
	
//int i = 0;
		while(1)
		{
			Start_red:
			set_speed(-90,0,0);
			if(Get_Home1()==0)
			{
				set_speed(0,0,0);
				Turn_angle(2,-90,0);
				osDelay(100);
				if(Get_Home1()==1) goto Start_red;
				while(1)
				{
					Move_inch(90,0,160);
					Turn_angle(2,-90,0);
					if(Get_Home1()==1)
					{
						Move_inch(90,0,220);
						Turn_angle(2,-90,0);
						break;
					}
				}		
				set_speed(0,0,0);
				break;	
			}
		}
		Turn_angle(2,-90,0);
		set_speed(0,30,0);
		
		while(1)
		{
			if(Get_Home2()==0)
			{
				set_speed(0,0,0);
				set_speed(0,-50,0);
				osDelay(300);
				set_speed(0,0,0);
				break;
			}
		}
		Turn_angle(2,-90,0);
		osDelay(2000);
	}
	set_speed(0,0,0);
}

//绕圆代码
void run_round(int color)//立桩部分
{
	int cnt = 0;
	int cnt_count = 0;
	int time_count = 0;
	unsigned int delay_count = 0;
	
//	ActionGroup(81,1);
	
	Move_inch(70,0,1050);//从仓库移动出来
	
//	Wait_Servo_Signal(Wait_Dealy_MAX);
	
	set_speed(0,-40,0);
	while(1)
	{
		if(Get_HW_LEFT()==1)
		{
			while(1)
			{
				if(Get_HW_LEFT()==1)
				{
					osDelay(200);
					set_speed(0,0,0);
					break;
				}
			}
			break;
		}
	}
	osDelay(500);
//	Turn_angle(1,-90,0);
	
//	while(1)
//	{
//		if(Get_US_Distance()>=Low&&Get_US_Distance()<=High)
//		{
//			set_speed(0,0,0);
//			osDelay(20);
//			if(Get_US_Distance()>=Low&&Get_US_Distance()<=High)
//			break;
//			else
//			continue;
//		}
//		
//		else if(Get_US_Distance()<=160)
//		{
//			set_speed(-60,0,0);
//		}
//		
//		else if(Get_US_Distance()>=170)
//		{
//			set_speed(60,0,0);
//		}
//	
//	}
//	
//	set_speed(0,-70,0);
//	while(1)
//		{
//			if(Get_HW_LEFT()==1)
//			{
//				osDelay(200);
//				set_speed(0,0,0);
//				break;
//			}
//		}
//	
	
	
	int i = 20;

	extern int stake;
	extern int stake_count;
	int round_count  = 0;
		
//	for(uint8_t i=0;i<3;i++)
//	{
//		 MV_SendCmd(1, 0); //使能mv
//		 osDelay(50);
//		 MV_SendCmd(2, color); //设置蓝色为目标球
//		 osDelay(50);
//	}
//	 Set_QueryState(1);
//	Enable_ServoFlag();
	stake =1;
	stake_count = 0;
//	ActionGroup(71,1);
//	Wait_Servo_Signal(Wait_Dealy_MAX);
	
	for(uint8_t i=0;i<3;i++)
	{
		Set_InitYaw(0);
		osDelay(50);
	}
	
	set_imu_status(0);
	clear_motor_data();
	Set_FirstFlag(0);
	cnt_count = 3;
	while(1)
	{
		Start_round:
		if(Get_HW_LEFT()==1&&Get_HW_RIGHT()==1&&(stake_count ==0||cnt_count == 1||cnt_count == 3))
		{
			set_speed(0,-30,63);//一切满足那就正常转圈嘛
			
			printf("%d,%d\r\n",cnt_count,stake_count);
			if(cnt_count == 1)
			{
				delay_count++;
			}
			if(delay_count >= 100000&&cnt_count == 1)
			{
				cnt_count = 2;
				stake_count = 0;
			}
			if(cnt_count == 2)
			{
				if((Get_Yaw()>=-180&&Get_Yaw()<=-176))
				{
					round_count++;
				}
			}
			if(round_count>=10000000)cnt_count = 3;
			if(cnt_count == 3)
			{
				if((Get_Yaw()>=-180&&Get_Yaw()<=-176)||(Get_Yaw()>=176&&Get_Yaw()<=180))
				{
					printf("                  完成任务。。\r\n\r\n\r\n");
					set_speed(0,0,0);
					set_imu_status(1);
					Turn_angle(2,-180,0);
					break;
				}
			}
		}
		
		else if(Get_HW_LEFT()==0&&Get_HW_RIGHT()==1&&(stake_count ==0||cnt_count == 1||cnt_count == 3))
		{
		while(1)
			{
				if(Get_HW_RIGHT()==0&&Get_HW_LEFT()==1)
				{
					set_speed(0,-20,60+4*i);
					osDelay(15);
				}
				else if(Get_HW_LEFT()==0&&Get_HW_RIGHT()==1)
				{
					set_speed(0,-20,0);
					osDelay(15); 
				}
				else
				break;
			}

		}
	
		else if(Get_HW_RIGHT()==0&&Get_HW_LEFT()==1&&(stake_count ==0||cnt_count == 1||cnt_count == 3))
		{
			
			while(1)
			{
				if(Get_HW_RIGHT()==0&&Get_HW_LEFT()==1)
				{
					set_speed(0,30,50+5*i);
					osDelay(15);
				}
				else if(Get_HW_LEFT()==0&&Get_HW_RIGHT()==1)
				{
					set_speed(0,-20,0);
					osDelay(15); 
				}
				else
				break;
			}
			

		}
		
		else if(Get_HW_RIGHT()==0&&Get_HW_LEFT()==0&&(stake_count ==0||cnt_count == 1||cnt_count == 3))
		{
			set_speed(0,20,0);
			osDelay(5);
		}
		
		else if(stake_count == 1&&(cnt_count == 0||cnt_count == 2))
		{
			time_count++;
			if(time_count == 1)
			{
				cnt_count = 1;
				stake_count = 0;
				goto Start_round;
			}

//			ActionGroup(72, 1);
//			Wait_Servo_Signal(Wait_Dealy_MAX);
//			ActionGroup(71,1);
//			Wait_Servo_Signal(Wait_Dealy_MAX);
//			stake_count = 0;
			
			cnt++;
			
			if(cnt >= 0)
			{
				cnt_count = 3;
			}
			
		}
	}
	
	set_speed(0,-70,0);
	while(1)
		{
			if(Get_HW_LEFT()==1)
			{
				set_speed(0,0,0);
					break;
			}
		}
//	
//	while(1)
//	{
//		if(Get_US_Distance()>=168&&Get_US_Distance()<=178)
//		{
//			set_speed(0,0,0);
//			osDelay(5);
//			if(Get_US_Distance()>=168&&Get_US_Distance()<=178)
//			break;
//			else
//			continue;
//		}
//		
//		else if(Get_US_Distance()<=168)
//		{
//			set_speed(-90,0,0);
//		}
//		
//		else if(Get_US_Distance()>=178)
//		{
//			set_speed(90,0,0);
//		}
//	
//	}
//	Turn_angle(2,-180,0);
//	set_imu_status(0);
//	ActionGroup(73,1);
//	Wait_Servo_Signal(300000);
//	set_imu_status(1);
//	

//	 MV_SendCmd(0, 0);
//	Set_QueryState(0);
}
	

	
	
	



void blue_all(void)
{
	color = 1;

		centren_control(1);//定位到圆盘中心并且避障
	//	control_ball(2);//抓球
	//	Dump_ball(0);
	//	run_ladder(0);//阶梯平台
	//	Dump_RoundRing(0);//放码垛
	//	Dump_block();//放积木
	//	run_home(0);
}

void run_blue(void)
{
	color = 0;
	//路径规划
	centren_control(0);//定位到圆盘中心并且避障
	control_ball(2);//抓球
	Dump_ball(0);//倒球
//	run_round(2);//立桩
	run_ladder(0);//阶梯平台
	Dump_RoundRing(0);//放码垛
	Dump_block();//放积木块
	// run_home(0);
	// osDelay(5000);
	// set_speed(0,0,0);
	// vTaskDelete(NULL);
}


void run_red(void)
{
	color = 1;
	//路径规划
	centren_control(1);//定位到圆盘中心并且避障
	control_ball(0);//抓球
	Dump_ball(1);//倒球
	//run_round(0);
	run_ladder(1);//阶梯平台
	Dump_RoundRing(1);//放码垛
	Dump_block();//放积木块
//	run_home(1);
}
