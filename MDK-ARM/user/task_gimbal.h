#ifndef _TASK_GIMBAL_H
#define _TASK_GIMBAL_H
#include "system.h"



#define YAW 0
#define PITCH 1

#define OUTER 0
#define INNER 1

#define PID_Out_Max 23333
#define PID_Iterm_Max 18000


/* yaw����Ԥ���� */
#define    YAW_UP		(1)//6623
#define    YAW_DOWN    (-1)//6020

#define    YAW_POSITION    YAW_DOWN

#define     AUTO_PRE    1   //���Ӿ�Ԥ��,1���Ԥ��

#define KF_ANGLE 0
#define KF_SPEED 1


/*---------------------�����ǽ��ٶȲ���------------------------*/
	#define PALST_COMPS_YAW        (0)                //����
	#define PALST_COMPS_PITCH      (0)                 //����

/*---------------------��̨ģʽѡ��------------------------*/
typedef enum
{
	MECH = 0,    //��еģʽ
	GYRO = 1,    //������ģʽ
} eGimbalCtrlMode;




/*---------------------��̨����ģʽ------------------------*/
/*   
    ��ͨ             	NORMAL
    ����,pitchˮƽ   	LEVEL
    ����ģʽ            AUTO
    
   
*/
typedef enum
{
	GIMBAL_NORMAL  = 0,//����ģʽ,����ģʽѡ��
	GIMBAL_LEVEL   = 1,//���ֿ���,��̨ˮƽ
    GIMBAL_AUTO    = 2,//����
    GIMBAL_AROUND  = 3,//180
    GIMBAL_TURN    = 4,//Q E  90���ͷ
	
}eGimbalAction;



typedef struct  //�Ӿ�Ŀ���ٶȲ���
{
  int delay_cnt;//����������֡Ŀ�겻�����ʱ��,�����ж��ٶ��Ƿ�Ϊ0
  int freq;
  int last_time;//�ϴ��ܵ�Ŀ��Ƕȵ�ʱ��
  float last_position;//�ϸ�Ŀ��Ƕ�
  float speed;//�ٶ�
  float last_speed;//�ϴ��ٶ�
  float processed_speed;//�ٶȼ�����
}speed_calc_data_t;




extern eGimbalCtrlMode  modeGimbal;
extern eGimbalAction actGimbal;
extern uint8_t if_yaw_auto_pre;

extern float Gimbal_PID_list[2][2][3][2][3];        //pitch/yaw  mech/gyro  ģʽ   OUTER/INNER   KP/KI/KD
extern float pitch_out;
extern float yaw_out;

void Gimbal_value(void);

void GIMBAL_InitArgument(void);

void GIMBAL_StopMotor(void);

int GIMBAL_InitCtrl(void);

void GIMBAL_Rc_Ctrl(void);
void GIMBAL_Key_Ctrl(void);

void GIMBAL_PID(void);
void GIMBAL_PID_Init(void);
void GIMBAL_PID_Mode(void);
void task_PID_out(void);
void sent_gimbal_to_down(int16_t v);

float constrain_float(float amt, float low, float high);
int16_t GIMBAL_GetOffsetAngle(void);
void Gimbal_Chass_Separ_Limit(void);
void GIMBAL_NORMAL_Mode_Ctrl(void);
void GIMBAL_LEVEL_Mode_Ctrl(void);
void GIMBAL_AUTO_Mode_Ctrl(void);
uint8_t GIMBAL_IfPitchLevel(void);

float Target_Speed_Calc(speed_calc_data_t *S, uint32_t time, float position);
bool GIMBAL_AUTO_PITCH_Sen(void);
bool GIMBAL_AUTO_PITCH_Sen_SK(void);

bool GIMBAL_AUTO_PITCH_Sentry(void);

#endif
