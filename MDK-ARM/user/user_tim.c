/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "system.h"

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
/* TIM3 init function */
void MX_TIM3_Init(void)
{
    htim3.Instance=TIM3;                          //通用定时器3
    htim3.Init.Prescaler=599;                     //分频系数
    htim3.Init.CounterMode=TIM_COUNTERMODE_UP;    //向上计数器
    htim3.Init.Period=699;                        //自动装载值
    htim3.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;//时钟分频因子
    HAL_TIM_Base_Init(&htim3);


}

void MX_TIM4_Init(void)
{
    htim4.Instance=TIM4;                          //通用定时器3
    htim4.Init.Prescaler=1200-1;                     //分频系数
    htim4.Init.CounterMode=TIM_COUNTERMODE_UP;    //向上计数器
    htim4.Init.Period=699;                        //自动装载值
    htim4.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;//时钟分频因子
    HAL_TIM_Base_Init(&htim4);


}


void TIM3_IRQHandler(void)
{
    Task_Dispatch_tim();
}

void TIM4_IRQHandler(void)
{
    system_time++;
}
