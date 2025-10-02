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
short mv_stop_flag = 0; //�ж�MV��صĹ���״̬
//int Disc_Count = 0;
int rectangle_count = 0;
//int servo_flag = 0;
osThreadId MV_QuertThreadHandle;
static bool MV_QueryTask_Exit = true; //�Ƿ񲻴���MV��ѯ����
static bool MV_Query_Stae = false; //�Ƿ���MV��ѯ��������ѯ
//uint8_t Rest_QueryTimes = 0;
int get_mv_param = 0;
int mv_one;
int cnt=0;
int stake = 0; //����׮ʹ�ã�ʶ����ʱֹͣ
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
//openMV�ĵ����������
bool Get_go_switch(void)
{
    return go_switch.motor_enable;
}
//openMV�ĵ����������
void Set_go_switch(bool go)
{
    go_switch.motor_enable = go;
}

static int YuanPan_first_flag = 0;
//Բ�̻�ץ����׸���
void Set_FirstFlag(int state)
{
    YuanPan_first_flag = state;
}
//Բ�̻�ץ����׸���
int Get_FirstFlag(void)
{
    return YuanPan_first_flag;
}

void MV_QueryTaskFunc(void const *argument);
//������openMV����ѯ����
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
 * @brief openMV �Ĳ�ѯ����������ʱ
 * @param argument 
 */
void MV_QueryTaskFunc(void const *argument)
{
	while (!MV_QueryTask_Exit)
	{
		//���ÿ�ʼ��ʱ����
		if (MV_Query_Stae)
		{
			MV_SendCmd(9, 0); //��openmv���Ͳ�ѯָ��
			osDelay(100);     // 10HZ
		}
		else
		{
			osDelay(10); //��Ƶ��ˢ��
		}
	}
	vTaskDelete(NULL);
}
//�����Ƿ���MV��ѯ��������ѯ
void Set_QueryState(int state)
{
    MV_Query_Stae = state;
    if ((MV_Query_Stae == 1) && MV_QueryTask_Exit) //������ѯ�Ҳ�������������Ҫ��������
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
mvrec_t mv_rec; // mv�Ľṹ��
mv_t MV =       //Ϊ�ṹ�帳��ֵ
    {
        .mv_uart = &huart4,
        .enable_switch = true,
        .mv_cmd = {0},
        .rec_buffer = {0},
        .rec_len = 0,
        .RX_Status = 0}; //��ʼ������

volatile int disc_countval = 0, color_val = 0;

void Set_AcIDofBar(short change)
{
    AcIDofBar = change;
}
void Disc_Report(void)
{
    disc_countval++;
    printf("\n\tԲ�̻�����,��ǰĿ��%d,������%d \t\n", color_val, disc_countval);
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
 * @brief ����Э����뷢�͵�����
 * @param event_id �¼�������
 * @param param ������16λ����
 * @author peach99CPP
 **********************************************************************/
static void cmd_encode(const uint8_t event_id, uint8_t param)
{
	//����ͨѶЭ��
	MV.mv_cmd[0] = START_BYTE; //֡ͷ
	MV.mv_cmd[1] = event_id;   //�������¼�id
	MV.mv_cmd[2] = param;		//����
	MV.mv_cmd[3] = (uint8_t)(event_id + param); //��У��
	MV.mv_cmd[4] = END_BYTE;                    //֡β
}
/**********************************************************************
 * @brief ���¼��Ͳ�������MV
 * @param event_id �¼�������
 * @param param ������16λ����
 **********************************************************************/
void MV_SendCmd(const uint8_t event_id, const int param)
{
	cmd_encode(event_id, param);                                 //���ݻ�õĲ�������cmd����
	HAL_UART_Transmit(MV.mv_uart, MV.mv_cmd, BUFFER_SIZE, 0xff); //��cmd���ͳ�ȥ
	//printf("MV_SendCmd  %d,%d   ok\r\n",event_id,param);
	memset(MV.mv_cmd, 0, sizeof(MV.mv_cmd));                     //��cmd�������³�ʼ��
}

void MV_rec_decode(void);
/**********************************************************************
 * @name	MV_IRQ
 * @brief	openMV ͨѶ���жϴ�����
 * @author	peach99CPP
 ***********************************************************************/
void MV_IRQ(void)
{
	uint8_t rec_data = MV.mv_uart->Instance->RDR;
	if (MV.RX_Status == 0)
	{
		if (rec_data == START_BYTE)
		{
			MV.RX_Status = 1; //����֡ͷ������ǣ�ֱ���˳�
			MV.rec_len = 0;
			return;
		}
		else if (rec_data == QR_StartByte)
		{
			MV.RX_Status = 2; //���յ�ת���Ķ�ά�������
			MV.rec_len = 0;   //��ʼ�����ձ���
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
			MV.rec_buffer[MV.rec_len++] = rec_data; //��������
			//��ֹ��Ϊ�����¿���
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
			MV.RX_Status = 0; //������� ����
			MV.rec_len = 0;
		}
	}
}

/**********************************************************************
 * @name	MV_rec_decode
 * @brief	�жϽ�����ɺ󣬶Խ��յ����ݽ��н���
 * @author	peach99CPP
 ***********************************************************************/
static void MV_rec_decode(void)
{
	if (MV.rec_buffer[0] + MV.rec_buffer[1] == MV.rec_buffer[2])
	{
		mv_rec.event = MV.rec_buffer[0];
		mv_rec.param = MV.rec_buffer[1];
		MV_Decode();
		mv_rec.event = 0, mv_rec.param = 0; //ʹ��������
		MV.rec_len = 0;                     //����
		MV.RX_Status = 0;
	}
	//������֮��ǵ����³�ʼ���ṹ���е�rec_len��RX_status�������������
}

/****�����ǵײ�ʵ�֣��������ϲ��Ӧ��****/

/**********************************************************************
 * @name    MV_Decode
 * @brief	�����Լ�����Ĳ�������ִ������
 * @author  peach99CPP
 ***********************************************************************/
void MV_Decode(void)
{
#define pid_p 0.5
#define Ball_Signal 0x06
#define Rectangle_Signal 0x01
#define Ring_Signal 0x02
#define MV_Blob 0X02
	//ȷ�����У����Խ���ָ�� ��ʱopenmv�Ͷ�ض�׼����ִ��ָ��
    if (!Get_Servo_Flag()) return;
	//������׮�����ƽ̨:
	if ((mv_rec.event == MV_Blob)&&(Get_FirstFlag()==0)) //��׮����
	{
		if(stake == 0)
		{
			// ActionGroup(82, 1);
			// yuan_count = 0; 
			Set_FirstFlag(1);
			// printf("����\n");
			// yellow = 1;
		}
		else if(stake == 1)	
		{
			set_speed(0,0,0);
			stake_count = 1;
		}
	}
	else if (mv_rec.event == Ball_Signal && Get_MV_once() == 1) //����ƽ̨����
	{
		Set_HeightAvailable(false);//���ø߶ȱ任
		get_mv_param = mv_rec.param;
		Enable_StopSignal();//ʹ��ͣ���ź�
		Set_QueryState(0);//�ر�mv����
		Set_MV_once(0);
		Set_Change_Hight_Status(false);
		if(cnt<5)
		{
			extern int color;
			Enable_StopSignal(); //ʹ��ͣ���ź�
			if(color == 0)
			{
				switch (Get_Height()) //��ȡ��ǰ�ĸ߶���Ϣ�����ݸ߶Ƚ���ץȡ
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
				switch (Get_Height()) //��ȡ��ǰ�ĸ߶���Ϣ�����ݸ߶Ƚ���ץȡ
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
			Disable_StopSignal();//��������ͼ����߿�
		}
	}
}

/**********************************************************************
 * @name	Get_Stop_Signal
 * @brief	�ڽ���ƽ̨ʹ�ã����ش�ʱ�Ƿ�ֹͣ���ź�
 * @return	�Ƿ�Ӧ��ͣ����ͣ����Ϊ1
 * @author	peach99CPP
 ***********************************************************************/
int Get_Stop_Signal(void)
{
    return mv_stop_flag;
}

/**********************************************************************
 * @name	Enable_StopSignal
 * @brief	�ڽ���ƽ̨ʹ�ã�ʹ��ͣ���ı�־λ
 * @author	peach99CPP
 ***********************************************************************/
void Enable_StopSignal(void)
{
    mv_stop_flag = 1;
}

/**********************************************************************
 * @name	Disable_StopSignal
 * @brief	�ڽ���ƽ̨ʹ�ã����ͣ����־λ
 * @author	peach99CPP
 ***********************************************************************/
void Disable_StopSignal(void)
{
    mv_stop_flag = 0;
}

/**********************************************************************
 * @name    MV_Start
 * @brief	Mv��ʼ��Ӧ����
 * @author  peach99CPP
 ***********************************************************************/
void MV_Start(void)
{
    Set_MV_Mode(true);
    MV_SendCmd(1, 0);
}

/**********************************************************************
 * @name    MV_Stop
 * @brief : ��MV����ֹͣ�ź�
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
