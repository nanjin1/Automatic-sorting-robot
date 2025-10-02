#include "openmv.h"
#include "servo.h"
#include "chassis.h"
#include "QR_code.h"
#include "read_status.h"
#include "general.h"
#include "uart_handle.h"

#define STOP_SIGNAL 0XAABB
//short Handle_Flag = 0;
//int mv_param;
short AcIDofBar = 1;
int mv_one_flag = 1;
short mv_stop_flag = 0; //判断MV舵控的工作状态
//int Disc_Count = 0;
int rectangle_count = 0;
//int servo_flag = 0;
osThreadId MV_QuertThreadHandle;
static bool MV_QueryTask_Exit = true; //是否不存在MV轮询任务
static bool MV_Query_Stae = false; //是否在MV轮询任务中轮询
//uint8_t Rest_QueryTimes = 0;
int get_mv_param = 0;
int mv_one;
int cnt=0;
int stake = 0; //在立桩使用，识别到球时停止
int stake_count = 0;
//int yellow = 0;
//int yuan_count = 0;

void cnt_clear(void)
{
   cnt=0;
}

int mv_go_flag(void)
{
   return mv_one;
}

static MvGo_t go_switch;
//openMV的电机启动开关
bool Get_go_switch(void)
{
    return go_switch.motor_enable;
}
//openMV的电机启动开关
void Set_go_switch(bool go)
{
    go_switch.motor_enable = go;
}

static int YuanPan_first_flag = 0;
//圆盘机抓球的首个球
void Set_FirstFlag(int state)
{
    YuanPan_first_flag = state;
}
//圆盘机抓球的首个球
int Get_FirstFlag(void)
{
    return YuanPan_first_flag;
}

void MV_QueryTaskFunc(void const *argument);
//创建向openMV的轮询任务
static void MV_QueryTask_Start(void)
{
    if (MV_QueryTask_Exit)
    {
        MV_QueryTask_Exit = false;
        MV_Query_Stae = true;
        osThreadDef(MV_QuertThreadHandle, MV_QueryTaskFunc, osPriorityHigh, 0, 256);
        MV_QuertThreadHandle = osThreadCreate(osThread(MV_QuertThreadHandle), NULL);
    }
}
// void Exit_MV_QueryTask(void)
// {
//     MV_QueryTask_Exit = true;
// }


/**
 * @brief openMV 的查询任务，有任务时
 * @param argument 
 */
void MV_QueryTaskFunc(void const *argument)
{
	while (!MV_QueryTask_Exit)
	{
		//设置开始定时发送
		if (MV_Query_Stae)
		{
			MV_SendCmd(9, 0); //向openmv发送查询指令
			osDelay(100);     // 10HZ
		}
		else
		{
			osDelay(10); //高频率刷新
		}
	}
	vTaskDelete(NULL);
}
//设置是否在MV轮询任务中轮询
void Set_QueryState(int state)
{
    MV_Query_Stae = state;
    if ((MV_Query_Stae == 1) && MV_QueryTask_Exit) //开启轮询且不存在任务则需要创建任务
	{
		MV_QueryTask_Start();
	}
}

void Update_rectangle_count(void)
{
    rectangle_count += 1;
}
int Get_RecCount(void)
{
    return rectangle_count;
}
mvrec_t mv_rec; // mv的结构体
mv_t MV =       //为结构体赋初值
    {
        .mv_uart = &huart4,
        .enable_switch = true,
        .mv_cmd = {0},
        .rec_buffer = {0},
        .rec_len = 0,
        .RX_Status = 0}; //初始化变量

volatile int disc_countval = 0, color_val = 0;

void Set_AcIDofBar(short change)
{
    AcIDofBar = change;
}
void Disc_Report(void)
{
    disc_countval++;
    printf("\n\t圆盘机拨球,当前目标%d,次数：%d \t\n", color_val, disc_countval);
    if ((disc_countval >= 8 && color_val == 0) || (disc_countval >= 4 && color_val == 1))
    {
        disc_countval = 0;
        color_val++;
    }
}

void Set_MV_Mode(bool mode)
{
    MV.enable_switch = mode;
}
bool Get_MV_Mode(void)
{
    return MV.enable_switch;
}

/**********************************************************************
 * @name cmd_encode
 * @brief 根据协议编码发送的内容
 * @param event_id 事件的类型
 * @param param 参数，16位数字
 * @author peach99CPP
 **********************************************************************/
static void cmd_encode(const uint8_t event_id, uint8_t param)
{
	//定义通讯协议
	MV.mv_cmd[0] = START_BYTE; //帧头
	MV.mv_cmd[1] = event_id;   //触发的事件id
	MV.mv_cmd[2] = param;		//参数
	MV.mv_cmd[3] = (uint8_t)(event_id + param); //和校验
	MV.mv_cmd[4] = END_BYTE;                    //帧尾
}
/**********************************************************************
 * @brief 将事件和参数发给MV
 * @param event_id 事件的类型
 * @param param 参数，16位数字
 **********************************************************************/
void MV_SendCmd(const uint8_t event_id, const int param)
{
	cmd_encode(event_id, param);                                 //根据获得的参数编码cmd数组
	HAL_UART_Transmit(MV.mv_uart, MV.mv_cmd, BUFFER_SIZE, 0xff); //将cmd发送出去
	//printf("MV_SendCmd  %d,%d   ok\r\n",event_id,param);
	memset(MV.mv_cmd, 0, sizeof(MV.mv_cmd));                     //将cmd数组重新初始化
}

void MV_rec_decode(void);
/**********************************************************************
 * @name	MV_IRQ
 * @brief	openMV 通讯的中断处理函数
 * @author	peach99CPP
 ***********************************************************************/
void MV_IRQ(void)
{
	uint8_t rec_data = MV.mv_uart->Instance->RDR;
	if (MV.RX_Status == 0)
	{
		if (rec_data == START_BYTE)
		{
			MV.RX_Status = 1; //读到帧头，做标记，直接退出
			MV.rec_len = 0;
			return;
		}
		else if (rec_data == QR_StartByte)
		{
			MV.RX_Status = 2; //接收到转发的二维码的数据
			MV.rec_len = 0;   //初始化接收变量
			QR_LoadData(rec_data, 1);
		}
	}
	else if (MV.RX_Status == 1)
	{
		if (rec_data == END_BYTE && MV.rec_len == BUFFER_SIZE - 2)
		{
			MV_rec_decode();
			Enable_StopSignal();
			MV.RX_Status = 0;
		}
		else
		{
			MV.rec_buffer[MV.rec_len++] = rec_data; //存入数组
			//防止因为出错导致卡死
			if (MV.rec_len == MAX_REC_SIZE)
			{
				MV.RX_Status = 0;
				MV.rec_len = 0;
				memset(MV.rec_buffer, 0, sizeof(MV.rec_buffer));
			}
		}
	}
	else if (MV.RX_Status == 2)
	{
		if (QR_LoadData(rec_data, 0))
		{
			MV.RX_Status = 0; //接收完毕 重启
			MV.rec_len = 0;
		}
	}
}

/**********************************************************************
 * @name	MV_rec_decode
 * @brief	判断接收完成后，对接收的内容进行解码
 * @author	peach99CPP
 ***********************************************************************/
static void MV_rec_decode(void)
{
	if (MV.rec_buffer[0] + MV.rec_buffer[1] == MV.rec_buffer[2])
	{
		mv_rec.event = MV.rec_buffer[0];
		mv_rec.param = MV.rec_buffer[1];
		MV_Decode();
		mv_rec.event = 0, mv_rec.param = 0; //使用完就清除
		MV.rec_len = 0;                     //重置
		MV.RX_Status = 0;
	}
	//处理完之后记得重新初始化结构体中的rec_len和RX_status变量，避免出错
}

/****上面是底层实现，下面是上层的应用****/

/**********************************************************************
 * @name    MV_Decode
 * @brief	根据自己定义的参数含义执行命令
 * @author  peach99CPP
 ***********************************************************************/
void MV_Decode(void)
{
#define pid_p 0.5
#define Ball_Signal 0x06
#define Rectangle_Signal 0x01
#define Ring_Signal 0x02
#define MV_Blob 0X02
	//确保空闲，可以接收指令 此时openmv和舵控都准备好执行指令
    if (!Get_Servo_Flag()) return;
	//处理立桩或阶梯平台:
	if ((mv_rec.event == MV_Blob)&&(Get_FirstFlag()==0)) //立桩处理
	{
		if(stake == 0)
		{
			// ActionGroup(82, 1);
			// yuan_count = 0; 
			Set_FirstFlag(1);
			// printf("拨球\n");
			// yellow = 1;
		}
		else if(stake == 1)	
		{
			set_speed(0,0,0);
			stake_count = 1;
		}
	}
	else if (mv_rec.event == Ball_Signal && Get_MV_once() == 1) //阶梯平台处理
	{
		Set_HeightAvailable(false);//禁用高度变换
		get_mv_param = mv_rec.param;
		Enable_StopSignal();//使能停车信号
		Set_QueryState(0);//关闭mv接收
		Set_MV_once(0);
		Set_Change_Hight_Status(false);
		if(cnt<5)
		{
			extern int color;
			Enable_StopSignal(); //使能停车信号
			if(color == 0)
			{
				switch (Get_Height()) //获取当前的高度信息，根据高度进行抓取
				{
					case LowestHeight:
						cnt++;
						ActionGroup(Lowest, 1);
						break;
					case MediumHeight:
						cnt++;
						ActionGroup(Medium, 1);
						break;
					case HighestHeight:
						cnt++;
						ActionGroup(Highest, 1);
						break;
					default:;
				}
			}
			else
			{
				printf("red\n");
				switch (Get_Height()) //获取当前的高度信息，根据高度进行抓取
				{
					case LowestHeight:
						cnt++;
						ActionGroup(13, 1);
						break;
					case MediumHeight:
						cnt++;
						ActionGroup(33, 1);
						break;
					case HighestHeight:
						cnt++;
						ActionGroup(23, 1);
						break;
					default:;
				}
			}
		}
		else
		{
			Disable_StopSignal();//其他情况就继续走咯
		}
	}
}

/**********************************************************************
 * @name	Get_Stop_Signal
 * @brief	在阶梯平台使用，返回此时是否停止的信号
 * @return	是否应该停车，停车则为1
 * @author	peach99CPP
 ***********************************************************************/
int Get_Stop_Signal(void)
{
    return mv_stop_flag;
}

/**********************************************************************
 * @name	Enable_StopSignal
 * @brief	在阶梯平台使用，使能停车的标志位
 * @author	peach99CPP
 ***********************************************************************/
void Enable_StopSignal(void)
{
    mv_stop_flag = 1;
}

/**********************************************************************
 * @name	Disable_StopSignal
 * @brief	在阶梯平台使用，清除停车标志位
 * @author	peach99CPP
 ***********************************************************************/
void Disable_StopSignal(void)
{
    mv_stop_flag = 0;
}

/**********************************************************************
 * @name    MV_Start
 * @brief	Mv开始响应命令
 * @author  peach99CPP
 ***********************************************************************/
void MV_Start(void)
{
    Set_MV_Mode(true);
    MV_SendCmd(1, 0);
}

/**********************************************************************
 * @name    MV_Stop
 * @brief : 向MV发送停止信号
 * @author  peach99CPP
 ***********************************************************************/
void MV_Stop(void)
{
    Set_MV_Mode(false);
    MV_SendCmd(0, 0);
}
void OpenMV_ChangeRoi(int roi)
{
    MV_SendCmd(11, roi);
    osDelay(100);
}

void Clear_MV_param(void)
{
    get_mv_param = 0;
}

int Get_MV_param(void)
{
    return get_mv_param;
}

void Set_MV_once(int flag)
{
    mv_one_flag = flag;
}

int Get_MV_once(void)
{
    return mv_one_flag;
}
