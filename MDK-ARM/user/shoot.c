#include "shoot.h"

/**************使用变量******************/
#define    FRIC_STEP0    0
#define    FRIC_STEP1    1
#define    FRIC_STEP2    2//遥控模式下的开启标志位
pid_t M3508E_PID;
pid_t M3508F_PID;//左右摩擦轮pid变量
//速度选择
#define FRI_OFF  	0
#define FRI_LOW  	1		//左键射速，英雄默认左键为正常射速
//#define FRI_MID  	2		//B键射速，
#define FRI_HIGH 	2		//B键 英雄高射速
#define FRI_MAD  	3		//吊射射速，英雄暂定高低，吊射三种
//#define FRI_MAX     5		//极限射速
fric_shoot_t fric_speed;
//遥控模式下的一些标志位
uint8_t Friction_Switch = 0;

uint8_t fric_changing_flag;
float fric_out1,fric_out2;//英雄左右摩擦轮最终输出值
int shoot_ramp=5;
int shoot_mode;
int Fric_Speed_Level;
#if		INFANTRY_DEBUG_ID == 3
int shoot_target[4][2]=       //默认8000，高速时11000，（待测），吊射10000（待测）
{
    {0,0},
    {8000,8000},
    {11000,11000},
    {10000,10000}    
};    
#endif

/*************************************/
/**
  * @brief  摩擦轮参数初始化
  * @param  void
  * @retval void
  * @attention 
*/
void shoot_init(void)
{
    Fric_Speed_Level=FRI_OFF;
    fric_speed.target_fric_spd[0]=1000;
	fric_speed.target_fric_spd[1]=1000;
    laser_off;
    PID_struct_init(&M3508E_PID,POSITION_PID,1.0,0.1,0,2000,2000);//左右摩擦轮pid参数设置
		PID_struct_init(&M3508F_PID,POSITION_PID,1.0,0.1,0,2000,2000);
}

/*
  * @brief  摩擦轮失控保护
  * @param  void
  * @retval void
  * @attention 缓慢减小PWM输出
  */
void shoot_StopMotor(void)
{

}
/************************摩擦轮总控制*****************************/

/**
  * @brief  摩擦轮控制函数
  * @param  void
  * @retval void
  * @attention 摩擦轮停止转动,红外关闭,摩擦轮从关闭到开启,云台抬头归中
  */
float fric_rc_debug = 300;
void Shoot_Ctrl(void)
{
    if (SYSTEM_GetRemoteMode( ) == RC)//遥控模式
		{
			
			
			if (FRIC_RcSwitch())//判断状态切换,跟弹仓开关逻辑相同
			{	//切换为关
				if (Fric_Speed_Level !=FRI_LOW)
				{
					Fric_Speed_Level =FRI_LOW;
				}
				else//切换为开
				{
					Fric_Speed_Level = FRI_LOW;//遥控模式下的速度选择，低射速，方便检录发光弹,告诉pitch要抬头
				}
			}			            
		}
		else				//键盘模式,可调射速
		{
			FRIC_KeyLevel_Ctrl();
		}
        fric_speed.target_fric_spd[0] = shoot_target[Fric_Speed_Level][0];
        fric_speed.target_fric_spd[1] = shoot_target[Fric_Speed_Level][1];//摩擦轮目标值大于0,标志着遥控模式下开启,告诉pitch要抬头
        shoot_fric_ctrl();        
}
/**
  * @brief  摩擦轮遥控控制
  * @param  void
  * @retval 是否转换当前状态
  * @attention 跟弹仓开关逻辑相同
  */
bool FRIC_RcSwitch(void)
{
	static uint8_t Friction_Switch;
    if (IF_RC_SW2_DOWN)
	{
		if (IF_RC_SW1_UP)
		{
			if (Friction_Switch == FRIC_STEP1)
			{
				Friction_Switch = FRIC_STEP2;
			}
			else if (Friction_Switch == FRIC_STEP2)
			{
				Friction_Switch = FRIC_STEP0;
			}
		}
		else
		{
			Friction_Switch = FRIC_STEP1;
		}
	}
	else
	{
		Friction_Switch = FRIC_STEP0;
	}

	if (Friction_Switch == FRIC_STEP2)
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}

/**
  * @brief  摩擦轮等级控制,键盘专用,根据等级自动设置射速
  * @param  当前等级
  * @retval void
  * @attention 键盘模式下不关摩擦轮
  */

void FRIC_KeyLevel_Ctrl(void)
{

    if (IF_MOUSE_PRESSED_LEFT)//非自瞄，正常打击
	{
		Fric_Speed_Level = FRI_LOW;//英雄用低射速
		
	}
//	if (GIMBAL_IfBuffHit() == TRUE)
//	{
//		Fric_Speed_Level = FRI_MAD;//打符模式,最高射速		
//	}
	else if (IF_KEY_PRESSED_B)//推家射速（高速）
	{
		Fric_Speed_Level = FRI_HIGH;				
		
	}
	//else if (IF_KEY_PRESSED_B)
	//{
		//Fric_Speed_Level = FRI_MID;//高射频低射速模式,推家肉搏用

	//}		
    else if(GIMBAL_AUTO_PITCH_Sentry() == TRUE)//射速提高
    {
			Fric_Speed_Level = FRI_MAD;

    }	
	else//防止出错，出错默认高射速
	{
		Fric_Speed_Level = FRI_HIGH;	
		
	}

}

/*
  * @brief  设置摩擦轮转速
  * @param  void
  * @retval void
  * @attention 
  */
void shoot_fric_ctrl(void)
{
    int temp[2];
    
    for(int i=0;i<2;i++)
    {
        temp[i]=fric_speed.target_fric_spd[i]-fric_speed.fric_spd[i];   
        if(temp[i]>0)
        {
            if (temp[i]>shoot_ramp)
            {
                fric_speed.fric_spd[i] += shoot_ramp;
            }
            else
            {
                fric_speed.fric_spd[i] += temp[i];
            }
        }
        else if (temp[i]<0)
        {
            if (temp[i]<-shoot_ramp)
            {
                fric_speed.fric_spd[i] -= shoot_ramp;
            }
            else
            {
                fric_speed.fric_spd[i] += temp[i];
            }
        }
    }
//    #if LEFT_RIGHT==0
//    TIM1->CCR1=fric_speed.fric_spd[0];
//    TIM1->CCR4=fric_speed.fric_spd[1];
//    #elif LEFT_RIGHT==1
//    TIM1->CCR4=fric_speed.fric_spd[0];
//    TIM1->CCR1=fric_speed.fric_spd[1];
//    #endif
		
		
}

bool IF_FRIC_READY(void)
{
    if(fric_speed.fric_spd[0]==fric_speed.target_fric_spd[0]
    && fric_speed.fric_spd[1]==fric_speed.target_fric_spd[1]
    && Fric_Speed_Level!=FRI_OFF)
        return 1;
    else
        return 0;
 }
void fric_set_output(int  fric_spd1, int  fric_spd2)
    
{

  //LEFT_FRICTION = fric_spd1;
  //RIGHT_FRICTION = fric_spd2;

fric_out1=pid_calc(&M3508E_PID, M3508[4].rotor_speed,fric_spd1);
fric_out2=pid_calc(&M3508F_PID, M3508[5].rotor_speed,fric_spd2);
Friction_set_voltage( 0,  fric_out1, fric_out2,0);//原定波胆盘数据和摩擦轮一起发出，现在结构变了。怎么发再定
}