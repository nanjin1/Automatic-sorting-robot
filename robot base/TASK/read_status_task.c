/* ************************************************************
 *
 * FileName   : read_status.c
 * Version    : v1.0
 * Author     : peach99CPP
 * Date       : 2021-09-12
 * Description:
 ******************************************************************************
 */
#include "read_status.h "
#include "QR_code.h"
#include "chassis.h"
#include "HWT101_imu.h"
#include "openmv.h"
#include "servo.h"
#include "general.h"
#include "track_bar_receive.h"
#include "Wait_BackInf.h"
#include "imu_pid.h"
#include "chassis_control.h"
#include "time_cnt.h"
#define Height_HW1 5
#define RED_TARGET 1
#define BLUE_TARGET 0
#define Encoder_Type 2
#define Wait_Dealy_MAX 8000
int different = 0;
int different1 = 0;
int once_mv_close = 1;
bool add_big_speed =false;

// 17 18 19最低 最高 中间
osThreadId Read_Swicth_tasHandle;             //任务句柄
void Read_Swicth(void const *argument);       //函数声明
osThreadId Height_UpadteTask;                 //任务句柄
void HeightUpdate_Task(void const *argument); //函数声明

ScanDir_t Height_Mode = Primary_Head;
Height_t Current_Height = PrimaryHeight;
Game_Color_t Current_Color = Not_Running;




int Time_constant_before = 800, Time_constant_after = 1000;
int MV_stop;
int servo_enable = 1;
int Target_color = 0;
int now_second = 0;
int last_second = 0;
int now_minute = 0;
int last_minute = 0;
uint16_t SwitchExitTime = 2000;
int run_fast_flag = 0;
void Trans_Cons(int val1, int val2)
{
    if (val1 >= 0)
        Time_constant_before = val1;
    if (val2 >= 0)
        Time_constant_after = val2;
    printf("\n\t当前方案为延时 %dms后进行高度变换\n\t变换后延时%dms后继续前进\t\n", Time_constant_before, Time_constant_after);
}




void Set_run_fast_mode(int flag)
{
    run_fast_flag = flag;
}

void Set_once(int flag)
{
    once_mv_close = flag;
}

int Get_once(void)
{
    return once_mv_close;
}

int Height_id = 1;
int read_task_exit = 1, Height_task_exit = 1; //任务退出标志
short Height_Flag = 0;

bool QR_Brick = true; //是否位于三种高度 模式

bool mv_next_flag = false;

short swicth_status[8]; //开关状态，只在内部进行赋值
short HW_Switch[10];    //红外开关的状态
int MIN_ = 40;
int VERTICAL = 10;
int MIN_SPEED = 80; //速度继续增大 防止在侧向移动时动力不足 撞击挡板使用

bool update_finish = false;
bool HeightAvailable = true;
bool Change_Hight_finish = false;

#define Wait_Servo_Done 20000 //等待动作组完成的最大等待时间

#define SWITCH(x) swicth_status[(x)-1] //为了直观判断开关编号
#define HW_SWITCH(X) HW_Switch[(X)-1]  // 0到3下标就是红外开关的位置

#define Height_SWITCH(x) HW_Switch[(x) + 4 - 1] //高度的开关，第4和第5下标分配给高度红外

#define Side_SWITCH(X) HW_Switch[(X) + 6 - 1] //侧边红外的安装位置,有两个，分配下标为6 和7

bool Get_HeightAvailable(void)
{
    return HeightAvailable;
}
void Set_HeightAvailable(bool Switch_Status)
{
    HeightAvailable = Switch_Status;
}

void Recover_EnableStatus(void)
{
    Set_MV_Mode(true);
    Set_QR_Status(true);
}
int Get_target_color(void)
{
    return Target_color;
}

void Set_different_flag(int flag)
{
    different = flag;
}

void Set_different1_flag(int flag)
{

    different1 = flag;
}

void Set_MV_next(bool flag)
{
    mv_next_flag = flag;
}

bool Get_MV_next(void)
{
    return mv_next_flag;
}

int Get_Hight_data(void) //更新高度数据
{
    if (Current_Height == HighestHeight)
    {
        return 1;
    }
    else
        return 0;
}

void Change_Hight() //根据现有高度改变动作组
{
	
	
	  		switch (Get_Height()) //获取当前的高度信息，根据高度不同执行不同的动作组
            {
            case LowestHeight:
				
			
                ActionGroup(53, 1);//最底部阶梯平台扫描动作（今年需要重调）
			    Wait_Servo_Signal(Wait_Servo_Done);
                break;
            case MediumHeight:			
				
                ActionGroup(54, 1);//中间部分阶梯平台扫描动作（今年需要重调）
			    Wait_Servo_Signal(Wait_Servo_Done);
                break;
            case HighestHeight:
				
                ActionGroup(52, 1);//最高处阶梯平台扫描动作（去年调过）
			    Wait_Servo_Signal(Wait_Servo_Done);
			    printf("wchu;aoil;elelelle\r\n");
			    break;
            default:;
	  
	  
	  
			}
}

bool Get_Change_Hight_Status(void)
{
    return Change_Hight_finish;
}

void Set_Change_Hight_Status(bool flag)
{
    Change_Hight_finish = flag;
}

/**********************************************************************
 * @Name    Get_Update_Result
 * @declaration :获取高度更新的结果
 * @param   None
 * @retval   : 是否更新完成
 * @author  peach99CPP
 ***********************************************************************/
bool Get_Update_Result(void)
{
    return update_finish;
}

/**********************************************************************
 * @Name    Set_Update_Status
 * @declaration : 设置高度更新值
 * @param   status: [输入/出]  设置后的状态
 * @retval   : 无
 * @author  peach99CPP
 ***********************************************************************/
void Set_Update_Status(bool status)
{
    update_finish = status;
}

/**********************************************************************
 * @Name    Wait_Update_finish
 * @declaration : 等待更新结束 用这个卡住任务 否则不会停车
 * @param   None
 * @retval   : 无
 * @author  peach99CPP
 ***********************************************************************/
void Wait_Update_finish(void)
{
    while (Get_Update_Result() != true)
    {
        set_speed(0, 0, 0);
        osDelay(5);
    }
}

/**********************************************************************
 * @Name    Set_NeedUp
 * @declaration : 设置是否为二维码模式
 * @param   if_on: [输入/出] 是或否
 * @retval   : 无
 * @author  peach99CPP
 ***********************************************************************/
void Set_NeedUp(bool if_on) //在此处设置是否需要动态变换高度
{
    QR_Brick = if_on;
}
/**********************************************************************
 * @Name    Return_IFQR
 * @declaration : 返回是否处于二维码工作模式
 * @param   None
 * @retval   : true则为二维码模式
 * @author  peach99CPP
 ***********************************************************************/
bool Return_IFQR(void)
{
    return QR_Brick;
}


/**********************************************************************
 * @Name    Return_ReverseID
 * @declaration : 获取相对的开关编号
 * @param   id: [输入/出] 当前的开关编号
 * @retval   : 同侧的相对编号
 * @author  peach99CPP
 ***********************************************************************/
int Return_ReverseID(int id)
{
    if (id == 1)
        return 2;
    else if (id == 2)
        return 1;
    else if (id == 5)
        return 6;
    else if (id == 6)
        return 5;
    else
        return 1; //避免出错，返回0容易引起数组越界 todo最好在此分支增加一个错误报告的打印数据
}


/**********************************************************************
 * @Name    Judge_Side
 * @declaration :
 * @param   color_mode: [输入/出]
 **			 dir: [输入/出]
 * @retval   :
 * @author  peach99CPP
 ***********************************************************************/
void Judge_Side(int dir)
{
    if (dir == 5)
    {
        Current_Height = MediumHeight;
    }
    else if (dir == 6)
    {
        Current_Height = LowestHeight;
    }
}

/**********************************************************************
 * @Name    Start_HeightUpdate
 * @declaration :
 * @param   None
 * @retval   :
 * @author  peach99CPP
 ***********************************************************************/
void Start_HeightUpdate(void)
{
    if (Height_task_exit)
    {
        Height_task_exit = 0;
        Height_Flag = 0;
        osThreadDef(Height_UpadteTask, HeightUpdate_Task, osPriorityHigh, 0, 256);
        Height_UpadteTask = osThreadCreate(osThread(Height_UpadteTask), NULL);
    }
}

//处理变换高度时间的函数
int change_second(void)
{
    int true_second = 0;
    true_second = (now_minute * 60 + now_second) - (last_minute * 60 + last_second);
    return true_second;
}

/**********************************************************************
 * @Name    HeightUpdate_Task
 * @declaration :
 * @param   argument: [输入/出]
 * @retval   :
 * @author  peach99CPP
 ***********************************************************************/
void HeightUpdate_Task(void const *argument)//这个任务的作用是，通过红外判断当前高度并开启变换机械臂高度的状态
{
    Height_Flag = 0;
    // todo 后续增加参数或者其他赋值的变量来指定使用哪个红外（使用双高度红外的情况下）
    while (!Height_task_exit)
    {
        if (Current_Height == LowestHeight)
        {
            Height_id = 1; //使用1号来判断高度模式 /
            // todo 此处记得检查是否已经设置好对应高度的切换动作组
            printf("\n************Current Height:LowestHeight***************\n");
            while (!Height_task_exit)
            {

                if ((Get_Hw(6)==1) && Height_Flag == 0 && Get_Servo_Flag() == true && Get_HeightAvailable()) //只进行一次
                {
                    Height_Flag = 1;
                    Current_Height = HighestHeight;
                    last_second = Get_second();
                    last_minute = Get_minute();//获取当前时间
                    printf("\n************Current Height:HighestHeight***************\n%d\t %d\t\n", last_second, last_minute);
					osDelay(100);
                    Enable_StopSignal();//使能停车信号
                    Set_Change_Hight_Status(true); //开始变换机械臂高度
                    Set_HeightAvailable(false);//防止在最高平台再进来一次if
                }
                if (Height_Flag == 1 && Get_Servo_Flag() == true && (Get_Hw(6)==0) && Get_HeightAvailable()) //只进行一次
                {
                    uint8_t delta_time = 0;
                    now_minute = Get_minute();
                    now_second = Get_second();//获取当前时间
                    delta_time = change_second();//得到在中间位置的时间
                    if (delta_time >= 2)//如果在最高平台大于3秒
                    {
						osDelay(600);
                        Current_Height = MediumHeight;
						Set_Change_Hight_Status(true);//开始变换机械臂高度
                        Height_Flag = 2;
                        printf("\n************Current Height:MediumHeight***************\n");
						
						printf("666\r\n");
                        Enable_StopSignal();//使能停车信号
                        
                        Set_HeightAvailable(false);//防止在最高平台再进来一次if
                    }
                    else
                    {
                        printf("fail to  convert\t  delta time :%d\n", delta_time);
                    }
                }
                osDelay(1);
            }
        }
        else if (Current_Height == MediumHeight)
        {
             Height_id = 1; //使用1号来判断高度模式 /
            // todo 此处记得检查是否已经设置好对应高度的切换动作组
            printf("\n************Current Height:MediumHeight***************\n");
            while (!Height_task_exit)
            {

                if ((Get_Hight_HW_new(Height_id) == 1) && Height_Flag == 0 && Get_Servo_Flag() == true && Get_HeightAvailable()) //只进行一次
                {
                    Height_Flag = 1;
                    Current_Height = HighestHeight;
                    last_second = Get_second();
                    last_minute = Get_minute();//获取当前时间
                    printf("\n************Current Height:HighestHeight***************\n%d\t %d\t\n", last_second, last_minute);
						
                    Enable_StopSignal();//使能停车信号
                    Set_Change_Hight_Status(true); 
                    Set_HeightAvailable(false);//防止在最高平台再进来一次if
                }
                if (Height_Flag == 1 && Get_Servo_Flag() == true && (Get_Hight_HW_new(Height_id) == 0) && Get_HeightAvailable()) //只进行一次
                {
                    uint8_t delta_time = 0;
                    now_minute = Get_minute();
                    now_second = Get_second();//获取当前时间
                    delta_time = change_second();//得到在中间位置的时间
                    if (delta_time >= 3)//如果在最高平台大于3秒
                    {
                        Current_Height = LowestHeight;
                        Height_Flag = 2;
                        printf("\n************Current Height:LowestHeight***************\n");
                        Enable_StopSignal();//使能停车信号
                        Set_Change_Hight_Status(true);
                        Set_HeightAvailable(false);//防止在最高平台再进来一次if
                    }
                    else
                    {
                        printf("fail to  convert\t  delta time :%d\n", delta_time);
                    }
                }
                osDelay(1);
            }
        }
        osDelay(5);
    }
    Height_Flag = 0;
    Current_Height = PrimaryHeight;
    vTaskDelete(NULL);
}

/**********************************************************************
 * @Name    Exit_Height_Upadte
 * @declaration :
 * @param   None
 * @retval   :
 * @author  peach99CPP
 ***********************************************************************/
void Exit_Height_Upadte(void)
{
    Height_task_exit = 1;
}

/**********************************************************************
 * @Name    Start_Read_Switch
 * @declaration : 启动轻触开关任务
 * @param   None
 * @retval   : 无
 * @author  peach99CPP
 ***********************************************************************/
void Start_Read_Switch(void)
{
    if (read_task_exit)
    {
        read_task_exit = 0; //标记任务开启
        //初始化数组
        memset(swicth_status, 0, sizeof(swicth_status));
        memset(HW_Switch, 0, sizeof(HW_Switch));

        /* definition and creation of Read_Swicth_tas */
        osThreadDef(Read_Swicth_tas, Read_Swicth, osPriorityAboveNormal, 0, 128);
        Read_Swicth_tasHandle = osThreadCreate(osThread(Read_Swicth_tas), NULL);
    }
}

int Get_Hw(int No)
{
	switch (No)
	{
		case 1:
			if((uint8_t)HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_2)==0)
				return 1;
			else
				return 0;
			break;
		case 2:
			if((uint8_t)HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_0)==0)
				return 1;
			else
				return 0;
			break;
		case 3:
			if((uint8_t)HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_12)==0)
				return 1;
			else
				return 0;
			break;
		case 4:
			if((uint8_t)HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_13)==0)
				return 1;
			else
				return 0;
			break;
		case 5:
			if((uint8_t)HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_14)==0)
				return 1;
			else
				return 0;
			break;
		case 6:
			if((uint8_t)HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_15)==0)
				return 1;
			else
				return 0;
			break;
		case 7:
			if((uint8_t)HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_8)==0)
				return 1;
			else
				return 0;
			break;
		case 8:
			if((uint8_t)HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_9)==0)
				return 1;
			else
				return 0;
			break;
		case 9:
			if((uint8_t)HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_8)==0)
				return 1;
			else
				return 0;
			break;
		default:
        break; // 处理未匹配的情况
	}
	
}


















/**********************************************************************
 * @Name    Read_Swicth
 * @declaration : 任务函数实现核心函数
 * @param   argument: [输入/出] 无意义
 * @retval   : 无
 * @author  peach99CPP
 ***********************************************************************/
void Read_Swicth(void const *argument)
{
    while (!read_task_exit)
    {
        //因为GPIO口被配置成pullup，所以只有在低时才是轻触开关导通状态
        if (Get_SW(1) == 1)
            SWITCH(1) = on; //开关状态枚举
        else
            SWITCH(1) = off;

        if (Get_SW(2) == 1)
            SWITCH(2) = on;
        else
            SWITCH(2) = off;

        if (Get_SW(3) == 1)
            SWITCH(3) = on;
        else
            SWITCH(3) = off;

        if (Get_SW(4) == 1)
            SWITCH(4) = on;
        else
            SWITCH(4) = off;

        if (Get_SW(5) == 1)
            SWITCH(5) = on;
        else
            SWITCH(5) = off;

        if (Get_SW(6) == 1)
            SWITCH(6) = on;
        else
            SWITCH(6) = off;

        if (Get_SW(7) == 1)
            SWITCH(7) = on;
        else
            SWITCH(7) = off;

        if (Get_SW(8) == 1)
            SWITCH(8) = on;
        else
            SWITCH(8) = off;
        //红外开关部分
        if (HAL_GPIO_ReadPin(HW_S1_GPIO_Port, HW_S1_Pin) == GPIO_PIN_SET)
            HW_SWITCH(1) = off;
        else
            HW_SWITCH(1) = on;

        if (HAL_GPIO_ReadPin(HW_S2_GPIO_Port, HW_S2_Pin) == GPIO_PIN_SET)
            HW_SWITCH(2) = off;
        else
            HW_SWITCH(2) = on;

        if (HAL_GPIO_ReadPin(HW_S3_GPIO_Port, HW_S3_Pin) == GPIO_PIN_SET)
            HW_SWITCH(3) = off;
        else
            HW_SWITCH(3) = on;
        // if (HAL_GPIO_ReadPin(HW_S4_GPIO_Port, HW_S4_Pin) == GPIO_PIN_SET)
        // HW_SWITCH(4) = off;
        // else
        // HW_SWITCH(4) = on;
        //定高红外
        if (HAL_GPIO_ReadPin(HW_Height1_GPIO_Port, HW_Height1_Pin) == GPIO_PIN_SET)
            HW_SWITCH(4) = off;
        else
            HW_SWITCH(4) = on;

        if (HAL_GPIO_ReadPin(HW_Height2_GPIO_Port, HW_Height2_Pin) == GPIO_PIN_SET)
            HW_SWITCH(5) = off;
        else
            HW_SWITCH(5) = on;
        //侧边红外
        if (HAL_GPIO_ReadPin(Side_HW1_GPIO_Port, Side_HW1_Pin) == GPIO_PIN_SET)
            Side_SWITCH(1) = off;
        else
            Side_SWITCH(1) = on;
        if (HAL_GPIO_ReadPin(Side_HW2_GPIO_Port, Side_HW2_Pin) == GPIO_PIN_SET)
            Side_SWITCH(2) = off;
        else
            Side_SWITCH(2) = on;

        osDelay(10); //对请求的频率不高,所以可以10ms来单次刷新
    }
    memset(swicth_status, err, sizeof(swicth_status)); //清空到未初始状态，用于标记此时任务未运行
    vTaskDelete(NULL);                                 //从任务列表中移除该任务
    Read_Swicth_tasHandle = NULL;                      //句柄置空
}

/**********************************************************************
 * @Name    Exit_Swicth_Read
 * @declaration : 退出查询开关状态的任务
 * @param   None
 * @retval   : 无
 * @author  peach99CPP
 ***********************************************************************/
void Exit_Swicth_Read(void)
{
    read_task_exit = 1; //此变量为1 将使得任务函数循环条件不满足
}

/**********************************************************************
 * @Name    Get_Switch_Status
 * @declaration : 获取指定ID开关的通断状态
 * @param   id: [输入/出] 电机编号（1-8）
 * @retval   : 该开关的通断状态
 * @author  peach99CPP
 ***********************************************************************/
int Get_Switch_Status(int id)
{
    if (read_task_exit)
        return err; //确保当前任务处在进行中
    if (id < 1 || id > 10)
        return off; // todo当有开关数量更新时记得修改此处的值
    //上面两个是为了避免出错的判断条件
    return SWITCH(id); //返回对应开关的状态
}

/**********************************************************************
 * @Name    Get_HW_Status
 * @declaration :获取指定ID号红外开关的状态
 * @param   id: [输入/出] 红外开关编号
 * @retval   : 状态
 * @author  peach99CPP
 ***********************************************************************/
int Get_HW_Status(int id)
{
    // todo当有开关数量更新时记得修改此处的值
    if (read_task_exit || (id < 1 || id > 8)) //输入值限制避免出错
        return err;
    return HW_SWITCH(id);
}

/**********************************************************************
 * @Name    Get_Side_Switch
 * @declaration :获取侧边边界开关的状态
 * @param   id: [输入/出] 开关编号
 * @retval   : 状态  扫到了就为on
 * @author  peach99CPP
 ***********************************************************************/
int Get_Side_Switch(int id)
{
    if (id < 0 || id > 5) // todo当有开关数量更新时记得修改此处的值
        return off;
    return Side_SWITCH(id);
}
int Get_Height_Switch(int id)
{
    if (id < 1 || id > 2)
        return off; // todo 记得在更新元器件数量后更新次此处的限制范围
    return Height_SWITCH(id);
}

/**********************************************************************
 * @Name    Get_Height
 * @declaration : 获取此时高度以计算应该调用的动作组编号
 * @param   None
 * @retval   : 动作组编号
 * @author  peach99CPP
 ***********************************************************************/
int Get_Height(void)
{
    return Current_Height;
}

/**
 * @name: Set_SwitchSpeed
 * @brief: 设置撞击挡板的速度
 * @param {int} speed  想要以什么速度
 * @return {*}
 */
void Set_SwitchSpeed(int speed)
{
    MIN_SPEED = speed;
}
/**********************************************************************
 * @Name    Wait_Switches
 * @declaration :碰撞轻触开关的实现全过程
 * @param   dir: [输入/出] 方向 1正Y 3 正X 4 负Y
 * @retval   : 无
 * @author  peach99CPP
 ***********************************************************************/
void Wait_Switches(int dir)
{
    /*关于运行时速度的变量,不宜过高否则不稳定*/
    int Switch_Factor = 15;

    // if (read_task_exit)
    //     Start_Read_Switch();

    //    track_status(1, 0); //关闭循迹版，避免造成方向上的影响
    //    track_status(2, 0);

    volatile static short flag1, flag2;
    short x_pn, y_pn;
    int w1, w2, w1_factor, w2_factor;
    w1_factor = 1, w2_factor = -1;

    //关于参数的解析，根据方向来判定速度的分配方向
    if (dir == 1) //正X
    {
        // todo 经测试 速度60满足要求
        w1 = 6, w2 = 0;
        x_pn = 1, y_pn = 0;
        MIN_SPEED = 60; // TODO 增大碰撞时候的力度
    }
    else if (dir == 2) //负x方向
    {
        w1 = 1, w2 = 7; //视情况改变
        x_pn = 1, y_pn = 0;
        return;
    }
    else if (dir == 3) //负X方向
    {
        w1 = 1, w2 = 7;
        x_pn = -1, y_pn = 0;
        MIN_SPEED = 200;
    }
    else if (dir == 4) //负Y方向
    {
        w1 = 3, w2 = 5;
        x_pn = -1, y_pn = 0;
    }
    uint16_t overtimeval = Get_SwitchTimeThreshold();
    int w_speedVal = 0;
//开始靠近
Closing:
    flag1 = flag2 = 0;
    set_speed(MIN_SPEED * x_pn, MIN_SPEED * y_pn, 0); //设置一个基础速度，此速度与方向参数有关
    //等待开关都开启
    int dlt = 0;
    do
    {
        flag1 = Get_SW(w1); //获取状态
        flag2 = Get_SW(w2);
        if (flag1 == on || flag2 == on) //当有开关触碰到时，关闭陀螺仪
            set_imu_status(0);          //关闭陀螺仪,否则设置w速度无意义
        /*下面这一句语句，只在单个开关开启时会有作用*/
        w_speedVal = Switch_Factor * (flag1 * w1_factor + flag2 * w2_factor);
        if (w_speedVal != 0)
        {
            set_speed(0, 0, w_speedVal);
        }
        else
        {
            set_speed(MIN_SPEED * x_pn, MIN_SPEED * y_pn, 0); //设置一个基础速度，此速度与方向参数有关
        }
        //任务调度
        osDelay(10);
        dlt += 10;              //超时保护机制 防止出现卡死的情况
        if (dlt >= overtimeval) //防止卡死 设置最长等待时间
        {
            printf("%ds时间阈值到 超时退出\n", (overtimeval / 1000));
            dlt = 0;
            goto switch_exit;
        }
    } while (flag1 == off || flag2 == off); //只有两个都接通，才退出该循环
    osDelay(50);
    if (flag1 == off || flag2 == off)
    {
        MIN_SPEED -= 10.0; //不使用除法了 尝试使用减法
        if (ABS(MIN_SPEED) < 15)
            goto switch_exit; //防止卡死在这里
        goto Closing;         //继续回到靠近的程序
    }
switch_exit:
    //    Exit_Swicth_Read(); //用完了就关闭任务
    set_speed(0, 0, 0); //速度置0 防止此时出现移动
//    Set_InitYaw(0);     //修正车身角度
    set_imu_status(1);  //修正后再打开陀螺仪
    // todo：调用完函数根据实际需要进行陀螺仪角度的修正
}

/**********************************************************************
 * @Name    HWSwitch_Move
 * @brief  单独使用红外来移动到平台的一侧
 * @param   dir: [输入/出]  贴边移动的方向 1 2 为左右 5 6为侧边的 具体参考阶梯平台
 * @param 	 enable_imu: [输入/出]  是否使能陀螺仪
 * @retval   : 无
 * @author  peach99CPP
 ***********************************************************************/
void HWSwitch_Move(int dir, int enable_imu)
{
#define Speed_Factor_HW 1.2
    Set_IMUStatus(enable_imu);

    if (dir == 0)
    {
        while (Get_HW(dir) == off)
        {
            set_speed(0, -MIN_ * Speed_Factor_HW, 0); //把速度增大 避免出现无法启动的现象
            osDelay(5);
        }
        osDelay(100);
        set_speed(VERTICAL, MIN_ * Speed_Factor_HW, 0);
        while (Get_HW(dir) == on)
        {
            osDelay(10);
        }

        osDelay(150);
    }
    else if (dir == 1)
    {
        while (Get_HW(dir) == off)
        {
            set_speed(0, -43 * Speed_Factor_HW, 0); //把速度增大 避免出现无法启动的现象
            osDelay(5);
        }
        while (Get_HW(dir) == on)
        {
            set_speed(0, 43 * Speed_Factor_HW, 0); //把速度增大 避免出现无法启动的现象
            osDelay(5);
        }
        osDelay(100);
        set_speed(-VERTICAL, -MIN_ * Speed_Factor_HW, 0);

        osDelay(150);
    }
    else if (dir == 2)
    {

        while (Get_HW(dir) == off)
        {
            set_speed(0, MIN_ * Speed_Factor_HW, 0); //把速度增大 避免出现无法启动的现象
            osDelay(5);
        }
        osDelay(100);
        set_speed(VERTICAL, -MIN_ * Speed_Factor_HW, 0);
        while (Get_HW(dir) == on)
        {
            osDelay(10);
        }
        osDelay(150);
    }
    else if (dir == 3) //仓库左边的红外
    {
        if (different == 0)
        {
			  while(Get_HW_others(dir) == off)
			  {
			       set_speed(0, 40 * Speed_Factor_HW, 0); //把速度增大 避免出现无法启动的现象
                osDelay(5);
			  }
            while (Get_HW_others(dir) == on)
            {
                set_speed(-5, -40 * Speed_Factor_HW, 0); //把速度增大 避免出现无法启动的现象
                osDelay(5);
            }
        }
        else if (different == 1)
        {
            while (Get_HW_others(dir) == off)
            {
                set_speed(-15, -43 * Speed_Factor_HW, 0); //把速度增大 避免出现无法启动的现象45
                osDelay(5);
            }
        }
        else if (different == 2)
        {			  
            while (Get_HW_others(dir) == off)
            {
                set_speed(0, 40 * Speed_Factor_HW, 0); //把速度增大 避免出现无法启动的现象
                osDelay(5);
            }
        }
        osDelay(100);

        //        set_speed(-VERTICAL, MIN_ * Speed_Factor_HW, 0);
        //        while (Get_HW_others(dir) == on)
        //        {
        //            osDelay(100);
        //        }
    }
    else if (dir == 4) //仓库右边的红外
    {
        while (Get_HW_others(dir) == off)
        {
            set_speed(0, MIN_ * Speed_Factor_HW, 0); //把速度增大 避免出现无法启动的现象
            osDelay(5);
        }
        osDelay(100);
        set_speed(-VERTICAL, MIN_ * Speed_Factor_HW, 0);
        while (Get_HW_others(dir) == on)
        {
            osDelay(10);
        }
        osDelay(150);
    }
    set_speed(0, 0, 0);
    // osDelay(150);
    Set_IMUStatus(false);
    Set_InitYaw(0);
}

/**********************************************************************
 * @Name    Single_Switch
 * @declaration :检测单边开关
 * @param   switch_id: [输入/出]  开关号 1-8
 * @retval   : 无
 * @author  peach99CPP
 ***********************************************************************/
void Single_Switch(int switch_id)
{
    //    Set_IMUStatus(false); //直接抵着墙撞击，无需陀螺仪稳定角度
    short x, y; //不同方向的速度因子
    short x_vertical, y_vertical;
    int status;         //存储开关状态的变量
    if (read_task_exit) //确保开关的开启状态
        Start_Read_Switch();
    switch (switch_id) //分配速度方向和垂直板子的速度方向
    {
    case 1:
        x = -1, y = 0;
        x_vertical = 0, y_vertical = 1;
        break;
    case 2:
        x = 1, y = 0;
        x_vertical = 0, y_vertical = 1;
        break;
    case 3:
        x = 0, y = 1;
        x_vertical = -1, y_vertical = 0;
        break;
    case 4:
        x = 0, y = -1;
        x_vertical = -1, y_vertical = 0;
        break;
    case 5:
        x = 0, y = 1;
        x_vertical = 1, y_vertical = 0;
        break;
    case 6:
        x = 0, y = -1;
        x_vertical = 1, y_vertical = 0;
        break;
    case 7:
        x = -1, y = 0;
        x_vertical = 0, y_vertical = -1;
        break;
    case 8:
        x = 1, y = 0;
        x_vertical = 0, y_vertical = -1;
        break;
    default:
        x = 0, y = 0;
        x_vertical = 0, y_vertical = 0;
    }
RECLOSE:
    while (Get_Switch_Status(switch_id) != on)
    {
        set_speed(x_vertical * VERTICAL, y_vertical * VERTICAL, 0);
        osDelay(5);
    }
    osDelay(500);
    if (Get_Switch_Status(switch_id) != on)
        goto RECLOSE;
    set_speed(x * MIN_ + x_vertical * VERTICAL, y * MIN_ + y_vertical * VERTICAL, 0); //给一个速度,经测试需要在垂直方向上也给一个速度值避免车身被反弹
    do
    {
        status = Get_Switch_Status(switch_id); //获取状态
        if (status == err)
            Start_Read_Switch(); //防止此时任务退出而卡死在循环里
        osDelay(20);             //任务调度
    } while (status == on);      //直到开关断开，此时说明到达边界
    set_speed(0, 0, 0);          //停车
}

/**********************************************************************
 * @Name    Set_SwitchParam
 * @declaration : 调试所用途的函数接口
 * @param   main: [输入/出] 主要速度，沿着边沿移动的速度
 **			 vertical: [输入/出]  垂直与边沿的速度，来确保紧贴的状态
 * @retval   :
 * @author  peach99CPP
 ***********************************************************************/
void Set_SwitchParam(int main, int vertical)
{
    //调试速度的API
    MIN_ = main;         //沿着板子水平方向的速度
    VERTICAL = vertical; //垂直板子的速度，确保紧贴着。
}
/**********************************************************************
 * @Name    QR_Scan
 * @declaration :使用二维码进行阶梯平台的扫描
 * @param   status: [输入/出]  是否开启
 **			 color: [输入/出]  要抓的颜色，用于判断高度  代表红色 2代表蓝色
 **			 dir: [输入/出]    方向
 * @retval   :
 * @author  peach99CPP
 ***********************************************************************/
void QR_Scan(int status, int color, int dir, int enable_imu)
{
    ;
}

/**
 * @name:
 * @brief:
 * @param {*}
 * @return {*}
 */
void Ring_Move(void)
{
#define Ring_Action 42  //对应动作组的编号
#define Ring_HW_ID 1    //复用的红外的编号
#define Side_Factor 0.3 //为保持方向给一个垂直速度
    Set_IMUStatus(false);
    set_speed(MIN_ * Side_Factor, MIN_, 0);    //开始动起来
    while (Get_Side_Switch(Ring_HW_ID) == off) //复用高度的红外进行位置的判断
        osDelay(1);                            //给系统调度时间
    set_speed(0, 0, 0);                        //到达位置马上停车
    ActionGroup(Ring_Action, 1);              //执行对应的动作组
    while (!Get_Servo_Flag())                  //等待返回信息
        osDelay(5);
    Set_InitYaw(0); //每次这里都对的很准 所以直接用这里的角度来修正车身
    Set_IMUStatus(true);
}

void Disc_Mea(void)
{
    int mode = Get_TargetColor(); // 1红2蓝
    MV_Stop();                    //关闭
    Set_MV_Mode(false);           //防止误响应 关闭
    ActionGroup(25, 1);          //执行预备动作组
    Wait_Servo_Signal(2000);      //等待动作组
    ActionGroup(40, 1);          //复位爪子
    Wait_Servo_Signal(2000);      //等待动作组
    MV_Start();                   //开启openmv
    printf("\n开始靠近圆盘机\n"); // log输出
    Wait_Switches(1);             //撞击挡板修正角度以及定位
    Set_IMUStatus(false);
    set_speed(0, VERTICAL, 0); //确保撞击是直的。
    if (mode == 1)
    {
        printf("\n 红色执行45号动作组\n");
        MV_SendCmd(6, Get_TargetColor()); // 1红2蓝 3黄
        ActionGroup(45, 1);              //执行第二个动作组
    }
    else if (mode == 2)
    {
        printf("\n 蓝色执行35号动作组\n");
        MV_SendCmd(7, Get_TargetColor()); // 1红2蓝 3黄
        ActionGroup(35, 1);
    }
    printf("\n开始扫描\n");   // log输出
    Wait_Servo_Signal(2000);  //等待动作组完成
    osDelay(10);              //缓冲时间
    osDelay(25 * 1000);       // 圆盘机等待时间
    printf("\n收起\n");       // log输出
    while (!Get_Servo_Flag()) //确认此时不是在繁忙状态
        osDelay(5);
    ActionGroup(24, 1);     //结束的动作组
    Wait_Servo_Signal(2000); //等待动作组完成
    MV_Stop();               //结束 彻底关闭Mv 避免误响应
    Set_IMUStatus(true);     //恢复陀螺仪使能状态
}

void begin_servo_status(int flag)
{
    if (flag == 1)
    {

        if (Current_Height == LowestHeight)
        {
			 Wait_Servo_Signal(Wait_Servo_Done);
            ActionGroup(53, 1);
            Wait_Servo_Signal(Wait_Servo_Done);
            printf("回到原来的位置\r\n");
            Set_MV_once(1); //打开接收通道
            Disable_StopSignal();
            Set_HeightAvailable(true);
            osDelay(1);
        }
        else if (Current_Height == HighestHeight)
        {
			 Wait_Servo_Signal(Wait_Servo_Done);
            ActionGroup(52, 1);
            Wait_Servo_Signal(Wait_Servo_Done);
            printf("回到原来的位置\r\n");
            Set_MV_once(1); //打开接收通道
			
            Disable_StopSignal();
			
            Set_HeightAvailable(true);
			
            osDelay(1);
			
        }
		else
		{
			 Wait_Servo_Signal(Wait_Servo_Done);
			 ActionGroup(54, 1);
            Wait_Servo_Signal(Wait_Servo_Done);
            printf("回到原来的位置\r\n");
            Set_MV_once(1); //打开接收通道
			
            Disable_StopSignal();
			
            Set_HeightAvailable(true);
			
            osDelay(1);
		
		
		
		}
    }
    else
    {
        printf("舵控取消归位\r\n");
    }
}

/***********************************************************************
 * @brief 进行阶梯平台的扫描
 * @param color 根据半场确定要抓的颜色 0代表红色 1代表蓝色
 * @param imu_enable 陀螺仪使能
 * @param mv_open 打开mv，关QR
 * @author dorla
 ***********************************************************************/
void Ladder_process(int color, int imu_enable,int mv_open)
{
	ActionGroup(53,1);
	Wait_Servo_Signal(Wait_Servo_Done);
	Height_Flag = 0;
    Judge_Side(6);
    Start_HeightUpdate();
    Set_IMUStatus(imu_enable); //打开陀螺仪
	for(uint8_t i=0;i<3;i++)
	{
		MV_SendCmd(1, 0); //使能mv
		osDelay(50);
		MV_SendCmd(6, color); //设置目标
		osDelay(50);
	}
    Set_QueryState(mv_open); //开启mv查询命令
    Target_color = color+1 ;
	Set_QR_Target(Target_color);
	Set_QR_once(mv_open);//开启QR查询命令
    Disable_StopSignal();//停车信号关闭
    Set_MV_once(1);
	osDelay(500);
	Set_HeightAvailable(true); 
	Set_Change_Hight_Status(false);//允许启用高度变换开关，但确定当前不需要进行高度变换
	
Start_ladder:
    while (Get_Stop_Signal() == false && Get_Hw(5) == on && Get_Servo_Flag() == true)//如果停车信号关闭、没出阶梯平台、机械臂完成动作组，则车子继续走
    {
		cnt_clear();//清除计时器
        osDelay(50);
       // set_speed(-VERTICAL, 30, 0);//开始向右运动
		if(mv_open == 0)
		{
			set_speed(0, 30, 0);//开始向右运动
		}
		else
		{
			set_speed(-15, 20, 0);
		}
//        if (once_run)//如果头一次进来这个函数，即在对准阶梯平台时，就先把mv开启
//        {
//            Set_MV_once(1);
//			 once_run=0;
//        }
    }
    set_speed(0, 0, 0);

    if (Get_Hw(5) == off) //到达阶梯平台末尾，结束
    {
        Set_MV_once(0); //关闭接收通道
        goto Exit_Label;
    }
    else if (Get_Change_Hight_Status() == true) //改变机械臂状态
    {
		Set_QR_once(1);
		Set_MV_once(0);
        Change_Hight();//改变动作组
        Set_QR_once(0);
		Set_MV_once(1);
		
        Set_Change_Hight_Status(false);//防止再进一次
        Disable_StopSignal();
        Set_HeightAvailable(true);
        goto Start_ladder;
    }
    else
    {
        while ((Get_Servo_Flag() == false)&&(Get_Stop_Signal() ==true)) //舵机运行还未结束，停车
        {
            osDelay(5);
        }
        if ((Get_MV_param() == 0x01)||(Get_QR_finish() == 0))
        {
			printf("识别到矩形\r\n");    
            
            ActionGroup(61, 1);
            Wait_Servo_Signal(Wait_Servo_Done); //放矩形的动作进行
			Clear_QR_Color();//清除原有二维码数据
			Set_QR_finish(1);                //开启二维码通道
            Clear_MV_param();//清除mv接收到的形状数据，防止再进来一次
            begin_servo_status(1);
			MV_SendCmd(6, color);
			Set_MV_once(1);
			if(mv_open==0)
			{
			  Set_QR_once(0);
			}
            Set_QueryState(mv_open);//开启mv识别
			Set_HeightAvailable(true);
        }
        else if ((Get_MV_param() == 0x02)&&(Get_QR_finish() == 1))
        {
			printf("识别到圆环\r\n");  
            ActionGroup(60, 1);
            Wait_Servo_Signal(Wait_Servo_Done); //放圆环的动作进行
            Clear_MV_param();//清除mv接收到的形状数据，防止再进来一次
            begin_servo_status(1);
			MV_SendCmd(6, color);
			Set_MV_once(1);
			if(mv_open==0)
			{
			  Set_QR_once(0);
			}
            Set_QueryState(mv_open);//开启mv识别
			Set_HeightAvailable(true);
        }
		
		Set_Change_Hight_Status(true);
        goto Start_ladder;
    }
Exit_Label:
    Set_QueryState(0);
}

void change_labber_v(int speed)
{
    MIN_ = speed;
}
uint16_t Get_SwitchTimeThreshold(void)
{
    return SwitchExitTime;
}
void Set_SwitchTimeThreshold(uint16_t TimeVal)
{

    SwitchExitTime = TimeVal;
}

void Wait_Switches_Time(int dir, uint16_t TimeThreshold)
{
    if (TimeThreshold == 0)
        return;
    Set_SwitchTimeThreshold(TimeThreshold);
    Wait_Switches(dir);
}



void run_fast(void)
{
	int change_angle_cnt=0;
    int cnt_run = 0;
    if (run_fast_flag == 0)
    {
        set_speed(60, 0, 0);
        while (!Get_SW(0) && !Get_SW(6) && cnt_run <= 400)
        {
            cnt_run++;
            osDelay(10);
			  printf("4\r\n");
        }
        set_speed(120, 0, 0);
        while (!Get_SW(0) && !Get_SW(6) && cnt_run <= 800)
        {
            cnt_run++;
            osDelay(10);
			  printf("8\r\n");
        }
		  printf("10\r\n");
    }
    if (run_fast_flag == 1)
    {
        set_speed(-80, 0, 0);
        while (!Get_SW(0) && !Get_SW(6) && cnt_run <= 200)
        {
            cnt_run++;
            osDelay(10);
        }
        set_speed(-50, 0, 0);
        while (!Get_SW(0) && !Get_SW(6) && cnt_run <= 400)
        {
            cnt_run++;
            osDelay(10);
        }
    }
	  if (run_fast_flag == 2)
    {
        set_speed(-60, 0, 0);
        while (!Get_SW(0) && !Get_SW(6) && cnt_run <= 200)
        {
            cnt_run++;
            osDelay(10);
        }
        set_speed(-30, 0, 0);
        while (!Get_SW(0) && !Get_SW(6) && cnt_run <= 400)
        {
            cnt_run++;
            osDelay(10);
        }
    }
	 if(run_fast_flag == 3)
	 {
		 uint8_t overTimeCounter = 0;
		 while(!Get_SW(7))
		 {
			 set_speed(-100,0,0);
		    osDelay(10);
         change_angle_cnt++;
			 if(change_angle_cnt==200)
			 {
			  set_imu_status(0);
			 }
				 
			 if(change_angle_cnt>200 &&!Get_SW(7))
			 {
				 overTimeCounter++;
				 set_speed(0,0,-50);
				 osDelay(500);
				 set_speed(0,0,0);
				 set_imu_status(1);
				 osDelay(500);
				 change_angle_cnt = 0;
			 }
			 if(overTimeCounter >=4)
			 {
				 goto Exit_Caused_OverTime;
			 }
		 }
Exit_Caused_OverTime:
		 set_speed(0,0,0);
	 }
	 
    set_speed(0, 0, 0);
}
