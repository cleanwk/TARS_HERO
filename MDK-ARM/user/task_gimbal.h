#ifndef _TASK_GIMBAL_H
#define _TASK_GIMBAL_H
#include "system.h"



#define YAW 0
#define PITCH 1

#define OUTER 0
#define INNER 1

#define PID_Out_Max 23333
#define PID_Iterm_Max 18000


/* yaw上下预编译 */
#define    YAW_UP		(1)//6623
#define    YAW_DOWN    (-1)//6020

#define    YAW_POSITION    YAW_DOWN

#define     AUTO_PRE    1   //０视觉预测,1电控预测

#define KF_ANGLE 0
#define KF_SPEED 1


/*---------------------陀螺仪角速度补偿------------------------*/
	#define PALST_COMPS_YAW        (0)                //待测
	#define PALST_COMPS_PITCH      (0)                 //待测

/*---------------------云台模式选择------------------------*/
typedef enum
{
	MECH = 0,    //机械模式
	GYRO = 1,    //陀螺仪模式
} eGimbalCtrlMode;




/*---------------------云台操作模式------------------------*/
/*   
    普通             	NORMAL
    补弹,pitch水平   	LEVEL
    自瞄模式            AUTO
    
   
*/
typedef enum
{
	GIMBAL_NORMAL  = 0,//正常模式,进行模式选择
	GIMBAL_LEVEL   = 1,//弹仓开启,云台水平
    GIMBAL_AUTO    = 2,//自瞄
    GIMBAL_AROUND  = 3,//180
    GIMBAL_TURN    = 4,//Q E  90°调头
	
}eGimbalAction;



typedef struct  //视觉目标速度测量
{
  int delay_cnt;//计算相邻两帧目标不变持续时间,用来判断速度是否为0
  int freq;
  int last_time;//上次受到目标角度的时间
  float last_position;//上个目标角度
  float speed;//速度
  float last_speed;//上次速度
  float processed_speed;//速度计算结果
}speed_calc_data_t;




extern eGimbalCtrlMode  modeGimbal;
extern eGimbalAction actGimbal;
extern uint8_t if_yaw_auto_pre;

extern float Gimbal_PID_list[2][2][3][2][3];        //pitch/yaw  mech/gyro  模式   OUTER/INNER   KP/KI/KD
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
