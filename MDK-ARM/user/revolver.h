#ifndef _REVOLVER_H
#define _REVOLVER_H

#include "system.h"

typedef enum
{
	REVOL_POSI_MODE  = 0,
	REVOL_SPEED_MODE = 1,
}eRevolverCtrlMode;

typedef enum
{
	SHOOT_NORMAL       =  0,//射击模式选择,默认不动
	SHOOT_SINGLE       =  1,//单发
	SHOOT_TRIPLE       =  2,//三连发
	SHOOT_HIGHTF_LOWS  =  3,//高射频低射速
	SHOOT_MIDF_HIGHTS  =  4,//中射频高射速
	SHOOT_BUFF         =  5,//打符模式
	SHOOT_AUTO         =  6,//自瞄自动射击
}eShootAction;

void Task_Revolver(void);

void REVOLVER_InitArgument(void);
void REVOLVER_Rc_Ctrl(void);
void REVOLVER_Key_Ctrl(void);
void SHOOT_NORMAL_Ctrl(void);
void SHOOT_SINGLE_Ctrl(void);
void SHOOT_TRIPLE_Ctrl(void);
void SHOOT_HIGHTF_LOWS_Ctrl(void);
void SHOOT_MIDF_HIGHTS_Ctrl(void);
void Revolver_Angle_Rest(void);
void SHOOT_AUTO_Ctrl(void);
bool Revolver_Heat_Limit(void);
void REVOLVER_Rest(void);
void REVOL_PositionLoop(void);
void REVOL_SpeedLoop(void);
uint16_t JUDGE_usGetShootCold(void);
void REVOLVER_KeySpeedCtrl(void);
void REVOLVER_KeyPosiCtrl(void);
void REVOLVER_CANbusCtrlMotor(int16_t v);

#endif
