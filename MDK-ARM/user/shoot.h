#ifndef  _SHOOT_H_
#define  _SHOOT_H_
//#include "stdint.h"
#include "stm32f4xx_hal.h"
#include "system.h"
#include "pid.h"
/*****************����Ƶ���뽵�����٣����������Ħ�������***********************/

#define LEFT_RIGHT 0//��������CH1��CH4������֮


#define laser_on    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_SET)   //���⿪���궨��
#define laser_off   HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_RESET) //����رպ�


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
void fric_set_output(int  fric_spd1, int  fric_spd2);
void Shoot_Ctrl(void);
float pid_calc(pid_t* pid, float get, float set);
void abs_limit(float *a, float ABS_MAX);
bool FRIC_RcSwitch(void);
bool IF_FRIC_READY(void);

void Rubber_KeyLevel_Ctrl(void);
void shoot_rubber_ctrl(void);  //���ý���ת��


#endif
