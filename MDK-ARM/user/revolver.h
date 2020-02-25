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
	SHOOT_NORMAL       =  0,//���ģʽѡ��,Ĭ�ϲ���
	SHOOT_SINGLE       =  1,//����
	SHOOT_TRIPLE       =  2,//������
	SHOOT_HIGHTF_LOWS  =  3,//����Ƶ������
	SHOOT_MIDF_HIGHTS  =  4,//����Ƶ������
	SHOOT_BUFF         =  5,//���ģʽ
	SHOOT_AUTO         =  6,//�����Զ����
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
