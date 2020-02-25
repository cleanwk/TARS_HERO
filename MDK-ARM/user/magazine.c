#include "magazine.h"

#if    INFANTRY_DEBUG_ID==3
#define Magazine_Close_Angle   0//待测
#define Magazine_Open_Angle    90//待测
#endif

//弹仓开关标志位
#define MAGA_STEP0    0		//失能标志
#define MAGA_STEP1    1		//SW1复位标志
#define MAGA_STEP2    2		//弹仓开关标志
uint8_t Magazine_Switch;

uint8_t Magazine_flag;
uint8_t Magazine_opened_flag;
int16_t Magazine_Target;//PWM目标值
int16_t Magazine_Real;//PWM真实值
int16_t Magazine_Ramp = 100;//弹仓斜坡,控制变化速度

/**
  * @brief  弹仓舵机停止转动
  * @param  void
  * @retval void
  * @attention 失控保护,置0弹仓不动,此时可手动打开弹仓
  */
void Magazine_StopCtrl(void)
{
    TIM1->CCR2 = 0;
}

void Magazine_init(void)
{
    //角度初始化,使目标值与测量值都为关闭值
		Magazine_Target = Magazine_Close_Angle;
		Magazine_Real = Magazine_Close_Angle;

}
/**
  * @brief  弹仓总控
  * @param  void
  * @retval void
  * @attention 失控保护,置0弹仓不动,此时可手动打开弹仓
  */
void Magazine_Ctrl(void)
{
    static uint8_t opened_tick;
        if (SYSTEM_GetRemoteMode() == RC)
        {  if (Magezine_Rc_Switch())//判断是否要弹仓改变当前状态
            {
                //改变当前状态的判断
                Magazine_flag=~Magazine_flag;
            }
        }	
		else if (SYSTEM_GetRemoteMode() == KEY)
		{
			Magazine_Key_Ctrl();
		}
        
		if(Magazine_Real==Magazine_Close_Angle
            &&Magazine_Target==Magazine_Close_Angle)
        {
            opened_tick++;
            if(opened_tick>1)
            {
                Magazine_opened_flag=0;
                opened_tick=0;
            }    
        }
        else
        {
            Magazine_opened_flag=1;
        }
            
}
    
/**
  * @brief  遥控模式,判断是否下达了状态转换指令,进入一次之后立刻变成FALSE
  * @param  void
  * @retval 是否下达了改变状态的指令
  * @attention 逻辑较复杂,好好想想
  */
bool Magezine_Rc_Switch(void)
{
	if (IF_RC_SW2_MID)//遥控模式
	{
		if (IF_RC_SW1_UP)//开启弹仓条件1
		{
			if (Magazine_Switch == MAGA_STEP1)//开启弹仓条件2
			{
				Magazine_Switch = MAGA_STEP2;
			}
			else if (Magazine_Switch == MAGA_STEP2)//弹仓关闭
			{
				Magazine_Switch = MAGA_STEP0;//切断联系
			}
		}
		else		//标志SW1是否有复位的情况,在复位的情况下才能再次进入STERP2
		{
			Magazine_Switch = MAGA_STEP1;//保障SW1在下次变换之前一直不能用
		}
	}
	else//s2不在中间,不允许弹仓开启
	{
		Magazine_Switch = MAGA_STEP0;//可能是摩擦轮开启也可能是切换成键盘模式
	}
	
	
	if (Magazine_Switch == MAGA_STEP2)
	{
		return TRUE;//只有SW1重新变换的时候才为TRUE
	}
	else
	{
		return FALSE;
	}
}  

/**
  * @brief  键盘模式
  * @param  void
  * @retval void
  * @attention 
  */
void Magazine_Key_Ctrl(void)
{
    static uint8_t Maga_tick=0;
    static uint8_t Maga_Key_state[2]={0,0};
    static uint16_t Maga_time[2]={0,0};
    
    Maga_time[NOW]=system_time;
    if(IF_KEY_PRESSED_R)
    {
        Maga_Key_state[NOW]=1;
    }
    else
    {
        Maga_Key_state[NOW]=0;
    }
    
    if(Maga_Key_state[NOW]==0&&Maga_Key_state[LAST]==1)
        Maga_tick++;
    
    if(Maga_tick)
    {
        if(Maga_time[NOW]-Maga_time[LAST]>500)//500ｍｓ没再次得到下降沿，取消本次计数
        {
            Maga_tick=0;
        }
    }
    
    if(Maga_tick>1)
    {
        Magazine_flag=~Magazine_flag;
        Maga_tick=0;
    }
    Maga_time[LAST]=Maga_time[NOW];
}


/**
  * @brief  弹仓舵机开合
  * @param  开合flag　
  * @retval void
  * @attention 
  */
void Magazine_Servo(uint8_t flag)
{
    int16_t temp;
    if(flag)
    {
        Magazine_Target=Magazine_Open_Angle;
    }
    else
    {
        Magazine_Target=Magazine_Close_Angle;
    }
    temp=Magazine_Target-Magazine_Real;
    if(temp>0)
    {
        if(temp>Magazine_Ramp)
        {
            Magazine_Real+=Magazine_Ramp;
        }
        else
        {
            Magazine_Real+=temp;
        }
    }
    else if(temp<0)
    {
        if(temp<-Magazine_Ramp)
        {
            Magazine_Real-=Magazine_Ramp;
        }
        else
        {
            Magazine_Real+=temp;
        }
    }
    
	TIM1->CCR2 = Magazine_Real;
    
}


