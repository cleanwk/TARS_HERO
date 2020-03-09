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
void REVOLVER_Rest(void); //重启拨盘
void Revolver_Angle_Rest(void);  //拨盘角度重置
void REVOLVER_InitArgument(void); //初始化拨盘
void REVOLVER_CANbusCtrlMotor(int16_t v);

/*********拨盘总控制*************/
void Task_Revolver(void);
void REVOLVER_Rc_Ctrl(void);
void REVOLVER_Key_Ctrl(void);

/******键盘模式拨盘速度环和位置环的控制*******/
void REVOLVER_KeySpeedCtrl(void);
void REVOLVER_KeyPosiCtrl(void);


/******底盘键盘模式各类模式小函数*******/
void SHOOT_NORMAL_Ctrl(void);
void SHOOT_SINGLE_Ctrl(void);
void SHOOT_TRIPLE_Ctrl(void);
void SHOOT_HIGHTF_LOWS_Ctrl(void);
void SHOOT_MIDF_HIGHTS_Ctrl(void);
void SHOOT_AUTO_Ctrl(void);

/****卡弹处理*****/
void REVOL_PositStuck(void);  //位置式环卡弹处理（单发模式）
void REVOL_SpeedStuck(void);  //速度环式卡弹处理（待完成）

/*****PID控制*******/
void REVOL_PositionLoop(void);
void REVOL_SpeedLoop(void);

bool Revolver_Heat_Limit(void);
uint16_t JUDGE_usGetShootCold(void);





#endif
