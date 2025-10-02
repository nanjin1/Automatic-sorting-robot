#include "sin.h"
#include "math.h"

#define PI 3.1415926
TIM_HandleTypeDef TIM10_Handler;      //????? 
struct sin_param sin1={0,0,120,5};
float sin_generator(struct sin_param *param)
{
 float output;
 
 param->actual_t = param->time * param->angular_velocity;
 
 output = param->gain * sin(param->actual_t * PI/180);
 
 ++param->time;
 
 if (param->actual_t >= 360)
  param->time = 0;
 
 return output;
}
//void Timer10_Init()
//{ //???5 1ms
//   __HAL_RCC_TIM10_CLK_ENABLE();
//     
//    TIM10_Handler.Instance=TIM10;                          //?????4
//    TIM10_Handler.Init.Prescaler=215;                     //??
//    TIM10_Handler.Init.CounterMode=TIM_COUNTERMODE_UP;    //?????
//    TIM10_Handler.Init.Period=999;                        //?????
//    TIM10_Handler.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
//    HAL_TIM_Base_Init(&TIM10_Handler);
//  HAL_NVIC_SetPriority(TIM1_UP_TIM10_IRQn,0,0);    //???????,?????3,????3
//    HAL_NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);          //??ITM4??  
//    HAL_TIM_Base_Start_IT(&TIM10_Handler); //?????4????4?? 
//}

//void TIM10_IRQHandler(void)
//{  
//    if(__HAL_TIM_GET_IT_SOURCE(&TIM10_Handler,TIM_IT_UPDATE)==SET)//????
//    {
//  sin1.time++;
//    }
//    __HAL_TIM_CLEAR_IT(&TIM10_Handler, TIM_IT_UPDATE);//???????
//}




