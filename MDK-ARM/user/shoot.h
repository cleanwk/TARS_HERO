#ifndef  _SHOOT_H_
#define  _SHOOT_H_
//#include "stdint.h"
#include "stm32f4xx_hal.h"
#include "system.h"

/*****************高射频必须降低射速，拨弹必须和摩擦轮配合***********************/

#define LEFT_RIGHT 0//０代表左CH1右CH4，１反之


#define laser_on    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_SET)   //激光开启宏定义
#define laser_off   HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_RESET) //激光关闭宏定义



typedef struct 
{
int fric_spd[2];
int target_fric_spd[2];

} fric_shoot_t;

extern uint8_t fric_changing_flag;
extern fric_shoot_t fric_speed;
extern int shoot_target[4][2];
void shoot_fric_ctrl(void);
void shoot_StopMotor(void);
void FRIC_KeyLevel_Ctrl(void);
void shoot_init(void);
void Shoot_Ctrl(void);
bool FRIC_RcSwitch(void);
bool IF_FRIC_READY(void);

#endif
