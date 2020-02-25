#ifndef __SYSTEM_H
#define __SYSTEM_H
#include "main.h"

#define    INFANTRY_DEBUG_ID     3


/****************usuer**********************/
#include "mytype.h"
#include "bsp_uart.h"
#include "bsp_can.h"
#include "pid.h"
#include "user_tim.h"
#include "chassic.h"
#include "Calculate.h"
#include "task_gimbal.h"
#include "shoot.h"
#include "JY901.H"
#include "vision.h"
#include "magazine.h"
#include "revolver.h"


#define System_base_tim &htim3 //当前系统基于的时钟指针
#define gpio_on HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, GPIO_PIN_SET)
#define gpio_off HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, GPIO_PIN_RESET)

#define FALSE 0
#define TRUE 1


#define  error_num 1//待测


typedef enum
{
    RC_offline=0
}error;

enum
{
    NOW=0,
    LAST=1,
};
typedef enum
{
    RC   = 0,  
    KEY  = 1,  

} eRemoteMode;  // 遥控方式


typedef enum
{
	  SYSTEM_STARTING  = 0,
	  SYSTEM_RUNNING   = 1,

} eSystemState;

extern eSystemState systemState;
extern uint32_t system_time;

extern long system_tick;
extern uint8_t Rx[22];

void System_Run(void);
void System_Stop(void);
void Task_Dispatch_tim(void);
void init_system(void);
void Task_error(void);
//控制
void SYSTEM_Reset( void );
void SYSTEM_OutCtrlProtect( void );
void SYSTEM_UpdateSystemState( void );
void SYSTEM_UpdateRemoteMode( void );
eRemoteMode SYSTEM_GetRemoteMode( void );
eSystemState SYSTEM_GetSystemState( void );

#endif
