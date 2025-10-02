#include "usmart.h"
#include "usmart_str.h"
////////////////////////////�û�������///////////////////////////////////////////////
//������Ҫ�������õ��ĺ�����������ͷ�ļ�(�û��Լ�����)
#include "delay.h"
#include "motor.h"
#include "chassis.h"
#include "imu_pid.h"
#include "track_bar_receive.h"
#include "chassis_control.h"
#include "openmv.h"
#include "servo.h"
#include "HWT101_imu.h"
#include "read_status.h"
#include "servo.h"
#include "general.h"
#include "avoid_obs.h"
#include "general_interface.h"
#include "read_status.h"
/****��������ģʽ�Ŀ���*****/

#define DEBUG_MOTOR 0   //���Ե��?
#define DEBUG_TRACKER 1 //����ѭ����
#define DEBUG_IMU 1     //����������
#define DEBUG_CHASSIS 0 //�����˶�
#define DEBUG_SWITCH 0  //�ᴥ���ء����⿪��
#define DEBUG_OPENMV 1  // openmvͨѶ
#define Debug_Servo 0   //���ͨ�?
extern void Global_Debug(void);
/******����ʾ�����ø�������****/
//�������б���ʼ��(�û��Լ�����)
//�û�ֱ������������Ҫִ�еĺ�����������Ҵ�?
struct _m_usmart_nametab usmart_nametab[] =
    {
#if USMART_USE_WRFUNS == 1 //���ʹ���˶�д����?
        (void *)read_addr,
        "u32 read_addr(u32 addr)",
        (void *)write_addr,
        "void write_addr(u32 addr,u32 val)",
#endif
#if DEBUG_MOTOR == 0
        (void *)set_debug_motor,
        "void set_debug_motor(int status, int motor_id)",
        (void *)set_debug_speed,
        "void set_debug_speed(int speed)",
        (void *)set_motor_pid,
        "void set_motor_pid(int kp, int ki, int kd)",
        (void *)set_motor_maxparam,
        "void set_motor_maxparam(int integrate_max, int control_output_limit)",
        (void *)set_motor_pid1,
        "void set_motor_pid1(int kp, int ki, int kd)",
        (void *)set_motor,
        "void set_motor(int motor_id, int control_val)",
		(void *)set_motor_debug,
		"void set_motor_debug(int x,int y,int z)",
#endif
#if DEBUG_TRACKER == 1
        (void *)set_track_pid,
        "void set_track_pid(int kp, int ki, int kd)",
        (void *)track_status,
        "void track_status(int id, int status)",
#endif
#if DEBUG_IMU == 1
        (void *)set_imu_param,
        "void set_imu_param(int p,int i,int d)",
		(void *) Circle_around,
		"void Circle_around(int x_speed,int w_speed)",
/**�����ǲ���**/
#endif
#if DEBUG_OPENMV == 1
        (void *)Set_QueryState,
        "void Set_QueryState(int state)",

#endif
#if DEBUG_CHASSIS == 1
        /**�����˶�����**/
        (void *)Comfirm_Online,
        "void Comfirm_Online(int dir)",
        (void *)move_slantly,
        "void move_slantly(int dir, int speed, uint16_t delay)",
/***Ŀǰ�ò���

***/
#endif
#if Debug_Servo == 1

        (void *)Ass_Door, //����ʱƨ���ϵ���
        "void Ass_Door(int status)",
        (void *)Lateral_infrared, //��ߺ���Ŀ���
        "void Lateral_infrared(int status)",
        (void *)Baffle_Control, //���Ƶ���
        "void Baffle_Control(int up_dowm)",
        (void *)Single_Control, //���Ƶ������?
        "void Single_Control(int id, int control_mode, int angle, int  time, int delay)",
       
#endif
        //�˶�����
        (void *)set_speed,
        "void set_speed(int x, int y, int w)",
        (void *)move_by_encoder,
        "void move_by_encoder(int  direct, int val)",
        // mv����
        (void *)MV_Start,
        "void MV_Start(void)",
        (void *)MV_Stop,
        "void MV_Stop(void)",
        (void *)MV_SendCmd,
        "void MV_SendCmd(const uint8_t event_id, const int param)",
        //������
        (void *)Wait_Switches,
        "void Wait_Switches(int dir)",
        (void *)set_imu_status,
        "void set_imu_status(int status)",
        (void *)Set_InitYaw,
        "void Set_InitYaw(int target)",
        (void *)Turn_angle,
        "void Turn_angle(int mode, int angle, int track_enabled)",
        //������
        (void *)ActionGroup,
        "void ActionGroup(uint8_t groupId, uint16_t run_times)",
        (void *)Move_CountBar,
        "void Move_CountBar(int id, int num, int speed)",

        (void *)Start_HeightUpdate,
        "void Start_HeightUpdate(void)",
        (void *)Judge_Side,
        "void Judge_Side(int dir)",
        (void *)Start_Read_Switch,
        "void Start_Read_Switch(void)",
        
        (void *)blue_all,
        "void blue_all(void)",
        (void *)Wait_Switches_Time,
        "void Wait_Switches_Time(int dir,uint16_t TimeThreshold)",
        (void *)change_w_error,
        "void change_w_error(int change)",
       
        (void *)Get_HW1,
        "void Get_HW1(int id)",
        (void *)HWSwitch_Move,
        "void HWSwitch_Move(int dir, int enable_imu)",
		(void *)Circle_around,
        "void Circle_around(int x_speed, int y_speed, int z_speed)",
		(void*) Set_InitYaw,
			"void Set_InitYaw(float target)",
		(void*)Set_IMUStatus,
			"void Set_IMUStatus(int status)",
		(void*)go_yuan,
			"void go_yuan(void)",
		(void*)run_blue,
			"void run_blue(void)",
		(void*)run_red,
			"void run_red(void)",
		(void*) Dump_ball,
		"void Dump_ball(int color)",
		(void*) control_ball,
		"void control_ball(int ball_color)",
		(void*) Dump_block,
		"void Dump_block(void)",
		(void*) Dump_RoundRing,
		"void Dump_RoundRing(int color)",
		(void*) Restart_Ladder,
		"void Restart_Ladder(void)",
		(void*) run_ladder,
		"void run_ladder(int color)",
		(void*) run_home,
		"void run_home(int color)",
		(void*) control_ball,
		"void control_ball(int ball_color)",
		(void*) Ladder_process,
			"void Ladder_process(int color, int imu_enable,int mv_open)",
		(void*) run_round,
			"void run_round(int color)",
		(void*) Move_inch,
			"void Move_inch(int dir1,int dir2,int time)"
		
		


		
};
///////////////////////////////////END///////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////
//�������ƹ�������ʼ��
//�õ������ܿغ���������
//�õ�����������
struct _m_usmart_dev usmart_dev =
    {
        usmart_nametab,
        usmart_init,
        usmart_cmd_rec,
        usmart_exe,
        usmart_scan,
        sizeof(usmart_nametab) / sizeof(struct _m_usmart_nametab), //��������
        0,                                                         //��������
        0,                                                         //����ID
        1,                                                         //������ʾ����,0,10����;1,16����
        0,                                                         //��������.bitx:,0,����;1,�ַ���
        0,                                                         //ÿ�������ĳ����ݴ��?,��ҪMAX_PARM��0��ʼ��
        0,                                                         //�����Ĳ���,��ҪPARM_LEN��0��ʼ��
};
