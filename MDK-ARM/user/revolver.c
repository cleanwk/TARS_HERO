#include "revolver.h"


/******拨盘,控制逻辑与云台类似*********/
#define     ROTATION 　         1           //顺逆时针 

#define 	REVOL_SPEED_GRID    11			//拨盘格数
#define    	AN_BULLET           26807		//单个子弹电机位置增加值 8191/11*36
#define     REVOL_SPEED_FRE     196.36        //电机速度和射频的关系REVOL_SPEED_RATIO/REVOL_SPEED_GRID


//拨盘电机模式,单级与串级
eRevolverCtrlMode Revolver_mode;
//发射模式
eShootAction actShoot;

//拨盘目标角度累计和,用于位置PID计算
float  Revolver_Angle_Target_Sum;
float  Revolver_Target_Sum_temp;//打符模式下的目标值暂存，此时目标角度用斜坡处理
float  Revolver_Ramp = AN_BULLET/40;//40ms转一格,一定不能超过50ms

//累计和
float Revolver_Angle_Measure_Sum;//拨盘测量角度累计和,用于位置PID
int16_t Revolver_Angle_Measure_Prev;//上次剩下的累加和角度,用于圈数计算判断


/*******************拨盘参数**********************/
//拨盘测量速度
int16_t Revolver_Speed_Measure;
//拨盘测量角度
int16_t Revolver_Angle_Measure;
//拨盘速度误差
float Revolver_Speed_Error;
//拨盘角度误差
float Revolver_Angle_Error[2];//  inner/outer
//拨盘目标角度
float  Revolver_Angle_Target;
//拨盘目标转速
float  Revolver_Speed_Target;
/****************射频控制******************/
#define SHOOT_LEFT_TIME_MAX  20	//左键连按切换间隔

//拨盘速度环射频
int16_t Revolver_Freq;
//位置环射击间隔,实时可变,数值越小位置环射击间隔越短
uint32_t Shoot_Interval = 0;

//射击间隔总响应，模式切换时及时重置成当前时间
uint32_t  Revol_Posit_RespondTime = 0;

/*点射*/
uint8_t Revol_Switch_Left = 0;

/*****************PID参数*****************/
float pTermRevolSpeed, iTermRevolSpeed;	
float pTermRevolAngle[2], iTermRevolAngle[2];//  inner/outer
float Revolver_Speed_kpid[3];//	kp/ki/kd
float Revolver_Angle_kpid[2][3];//  inner/outer    kp/ki/kd

/**********限幅*************/
//最终输出限幅
float Revolver_Output_Max;
float iTermRevolSpeedMax;//速度环微分限幅
float iTermRevolPosiMax;//位置环微分限幅

//拨盘电机输出量
float Revolver_Final_Output;

/********射击**********/
//发射子弹数,按一下加一颗,发一颗减一次
int16_t Key_ShootNum;//鼠标射击计数
int16_t ShootNum_Allow = 0;//还剩几颗弹可以打
uint16_t Residue_Heat;//剩余可用热量,热量限制控制
uint16_t Shoot_HeatLimit;//当前等级最大热量上限
uint16_t Shoot_HeatIncSpeed;//当前单发子弹热量增加值

/**
  * @brief  拨盘参数初始化
  * @param  void
  * @retval void
  * @attention 
  */
void REVOLVER_InitArgument(void)
{
	/* 目标值 */
	Revolver_Final_Output = 0;
	Revolver_Speed_Target = 0;
	
	/* PID参数 */
	  //速度环
	Revolver_Speed_kpid[kp] = 15;
	Revolver_Speed_kpid[ki] = 0;
	Revolver_Speed_kpid[kd] = 0;
	  //位置环
	Revolver_Angle_kpid[OUTER][kp] = 0.4;
	Revolver_Angle_kpid[OUTER][ki] = 0;
	Revolver_Angle_kpid[OUTER][kd] = 0;
	Revolver_Angle_kpid[INNER][kp] = 6;
	Revolver_Angle_kpid[INNER][ki] = 0;
	Revolver_Angle_kpid[INNER][kd] = 0;
	
	/* 限幅 */
	iTermRevolSpeedMax  = 250;
	iTermRevolPosiMax   = 2500;
	Revolver_Output_Max = 9999;
	
	/* 射击 */
	Key_ShootNum    = 0;
	Shoot_HeatLimit = 240;//热量初始化
	Revolver_Freq   = 0;//射频初始化
	
	/* 位置环目标角度 */
	Revolver_Angle_Target_Sum = Revolver_Angle_Measure;
	Revolver_Target_Sum_temp  = Revolver_Angle_Measure;
}


uint8_t revol_remot_change = TRUE;
void Task_Revolver(void)
{
    if (SYSTEM_GetRemoteMode() == RC)
			{
				REVOLVER_Rc_Ctrl();		
			}
			else
			{
				REVOLVER_Key_Ctrl();
				revol_remot_change = TRUE;
			}
     //热量限制
		if(Revolver_Heat_Limit() == FALSE) //一定要放在速度、位置控制之前
            //打符时不限热量
		{
			REVOLVER_Rest();//发弹清零,哪怕鼠标继续点击也不给发弹
		}
		
		if(Revolver_mode == REVOL_SPEED_MODE)
		{
			REVOL_SpeedLoop();
		}
		else if(Revolver_mode == REVOL_POSI_MODE)
		{
			REVOL_PositionLoop();
		}
		
		if(IF_FRIC_READY())//摩擦轮开启
		{
			REVOLVER_CANbusCtrlMotor(Revolver_Final_Output);
		}
		else
		{
			Revolver_Speed_Target = 0;//摩擦轮关闭,拨盘不给动
			Revolver_Angle_Rest();//摩擦轮关闭，屏蔽这期间的打弹指令
			REVOL_SpeedLoop();
			REVOLVER_CANbusCtrlMotor(Revolver_Final_Output);
		}       
}
#define REVOL_STEP0    0		//失能标志
#define REVOL_STEP1    1		//SW1复位标志
#define REVOL_STEP2    2		//弹仓开关标志
uint8_t	Revolver_Switch = 0;//弹仓遥控模式开关标志位转换
bool REVOLVER_Rc_Switch(void)
{
//	if (IF_RC_SW2_MID || IF_RC_SW2_DOWN)//机械或陀螺仪模式
	if (IF_RC_SW2_MID)//机械模式
	{
		if (IF_RC_SW1_DOWN)
		{
			if (Revolver_Switch == REVOL_STEP1)
			{
				Revolver_Switch = REVOL_STEP2;
			}
			else if (Revolver_Switch == REVOL_STEP2)
			{
				Revolver_Switch = REVOL_STEP0;
			}
		}
		else		
		{
			Revolver_Switch = REVOL_STEP1;
		}
	}
	else
	{
		Revolver_Switch = REVOL_STEP0;
	}
	
	if (Revolver_Switch == REVOL_STEP2)
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}
/**
  * @brief  拨盘重启
  * @param  void
  * @retval void
  * @attention 枪口超热量重置
  */
void REVOLVER_Rest(void)
{
	Key_ShootNum = 0;//位置环发弹清零
	Revolver_Speed_Target = 0;//速度环停止转动
	
	//速度环位置重置
	Revolver_Angle_Target_Sum   = Revolver_Angle_Measure;//位置环目标角度重置
	Revolver_Angle_Measure_Sum  = Revolver_Angle_Measure;//位置环转过角度重置
	Revolver_Angle_Measure_Prev = Revolver_Angle_Measure;//上次位置重置
	Revolver_Target_Sum_temp 	= Revolver_Angle_Measure;
	
	//PID积分清零
	iTermRevolSpeed = 0;
	iTermRevolAngle[INNER] = 0;
}

/**
  * @brief  拨盘角度清零
  * @param  void
  * @retval void
  * @attention 模式切换时用,防止下次切回去会突然动一下
  */
void Revolver_Angle_Rest(void)
{
	Key_ShootNum = 0;
	Revolver_Angle_Target_Sum   = Revolver_Angle_Measure;//位置环目标角度重置
	Revolver_Angle_Measure_Sum  = Revolver_Angle_Measure;//位置环转过角度重置
	Revolver_Angle_Measure_Prev = Revolver_Angle_Measure;//上次位置重置
	Revolver_Target_Sum_temp 	= Revolver_Angle_Target_Sum;
}
/**
  * @brief  拨盘的遥控模式
  * @param  void
  * @retval void
  * @attention 遥控用速度环
  */
void REVOLVER_Rc_Ctrl(void)
{	
	/*******************点射版**********************/
	
	if(IF_RC_SW2_MID)//机械模式下单发，方便测试弹道
	{
		Revolver_mode = REVOL_POSI_MODE;//位置环
		if(REVOLVER_Rc_Switch() == TRUE)
		{
			Key_ShootNum++;//打一颗
		}
		
		if(revol_remot_change == TRUE)//刚从键盘模式切换过来，清空发弹数据
		{
			revol_remot_change = FALSE;
			Revolver_Angle_Rest();//防止突然从键盘切到遥控狂转
		}
		
		if(Key_ShootNum != 0)
		{
			Key_ShootNum--;
			Revolver_Target_Sum_temp += AN_BULLET;
		}
		
		if(Revolver_Angle_Target_Sum != Revolver_Target_Sum_temp)//缓慢转过去
		{
			Revolver_Angle_Target_Sum = RAMP_float(Revolver_Target_Sum_temp, Revolver_Angle_Target_Sum, Revolver_Ramp);
		}
		
//		REVOL_PositStuck();//卡弹判断及倒转
	}
	/**************连发版*******************/
	else if(IF_RC_SW2_DOWN)//陀螺仪模式
	{
		Revolver_mode = REVOL_SPEED_MODE;//速度环
		
		if(IF_RC_SW1_DOWN)//sw1下打弹
		{
			Revolver_Freq = 5;//射频选择
			//速度环转速设置
			Revolver_Speed_Target = REVOL_SPEED_FRE*Revolver_Freq;
		}
		else	//遥控模式关闭拨盘
		{
			Revolver_Speed_Target = 0;
			Revolver_Freq = 0;
		}

//		REVOL_SpeedStuck();//卡弹判断及倒转
	}
	/********************************************/
}

/*******键盘模式************/

/**
  * @brief  拨盘的键盘模式
  * @param  void
  * @retval void
  * @attention 键盘用位置环控制
  */
void REVOLVER_Key_Ctrl(void)
{
	Revolver_mode = REVOL_POSI_MODE;//防止出错用,默认位置环
	
	SHOOT_NORMAL_Ctrl();//确定射击模式
	
	/*- 确定射击间隔和射击模式 -*/
	switch(actShoot)
	{
		case SHOOT_NORMAL:
			//射击模式选择,默认不打弹
			SHOOT_NORMAL_Ctrl();
		break;
		
		case SHOOT_SINGLE:
			//按一下左键单发,长按连发
			SHOOT_SINGLE_Ctrl();
		break;
		
		case SHOOT_TRIPLE:
			//长按连发
			SHOOT_TRIPLE_Ctrl();
		break;
		
		case SHOOT_HIGHTF_LOWS:
			//B高射频
			SHOOT_HIGHTF_LOWS_Ctrl();
		break;
		
		case SHOOT_MIDF_HIGHTS:
			//Z推家
			SHOOT_MIDF_HIGHTS_Ctrl();
		break;
		
		case SHOOT_BUFF:
//			//打符自动打弹
		break;
		
		case SHOOT_AUTO:
			//右键自瞄时自动打弹
//			SHOOT_AUTO_Ctrl();
		break;		
	}
	
	/*- 开始发弹,计数减 -*/
	if(Revolver_mode == REVOL_SPEED_MODE && IF_FRIC_READY() )
	{
		REVOLVER_KeySpeedCtrl();
	}
	else if(Revolver_mode == REVOL_POSI_MODE && IF_FRIC_READY())
	{
		REVOLVER_KeyPosiCtrl();
	}
}

/************************底盘键盘模式各类模式小函数****************************/
/**
  * @brief  键盘模式下发弹模式选择
  * @param  void
  * @retval void
  * @attention  普通模式清零计算,且不发弹
  */
void SHOOT_NORMAL_Ctrl(void)
{
	static uint32_t shoot_left_time = 0;//计算左键连按时间,时间过长切换成连发
	
	/*------ 左键抬起后才能打下一颗 -------*/
	if (IF_MOUSE_PRESSED_LEFT)
	{
		if (Revol_Switch_Left == 1)
		{
			Revol_Switch_Left = 2;
		}
		else if (Revol_Switch_Left == 2)
		{
			Revol_Switch_Left = 0;
		}
	}
	else if(!IF_MOUSE_PRESSED_LEFT)		
	{
		Revol_Switch_Left = 1;
		shoot_left_time = 0;//左键重新计时
	}
	/*------------------------------------*/
	
	if(IF_MOUSE_PRESSED_LEFT &&	shoot_left_time <= SHOOT_LEFT_TIME_MAX	//左键发弹
			&& !IF_KEY_PRESSED_B && !IF_KEY_PRESSED_Z /*&& !GIMBAL_IfAutoHit()*/)
	{
		Revolver_mode = REVOL_POSI_MODE;//位置环打弹
		shoot_left_time++;//判断长按,切换
		actShoot = SHOOT_SINGLE;	
	}
	else if(IF_MOUSE_PRESSED_LEFT && shoot_left_time > SHOOT_LEFT_TIME_MAX	//连按大于200ms
				&& !IF_KEY_PRESSED_B && !IF_KEY_PRESSED_Z /*&& !GIMBAL_IfAutoHit()*/)
	{
//		shoot_left_time++;
		Revolver_mode = REVOL_POSI_MODE;
		actShoot = SHOOT_TRIPLE;//连发模式
	}
	else if(IF_KEY_PRESSED_B	//高射频低射速
				&& !IF_MOUSE_PRESSED_LEFT && !IF_KEY_PRESSED_Z /*&& !GIMBAL_IfAutoHit()*/)
	{
		Revolver_mode = REVOL_POSI_MODE;
		actShoot = SHOOT_HIGHTF_LOWS;
		shoot_left_time = 0;
	}
	else if(IF_KEY_PRESSED_Z	//高射频极低射速,推家专用
				&& !IF_MOUSE_PRESSED_LEFT && !IF_KEY_PRESSED_B)
	{
		Revolver_mode = REVOL_POSI_MODE;
		actShoot = SHOOT_MIDF_HIGHTS;
		shoot_left_time = 0;
	}
	else if(actGimbal == GIMBAL_AUTO /*&& VisionRecvData.centre_lock==TRUE*/  //视觉说可以打了
				 && !IF_KEY_PRESSED_Z && !IF_KEY_PRESSED_B)
	{
		Revolver_mode = REVOL_POSI_MODE;
		actShoot = SHOOT_AUTO;
		shoot_left_time = 0;
	}
//	else if(GIMBAL_IfBuffHit() == TRUE && GIMBAL_IfManulHit() == FALSE)//打符模式且非手动打符模式
//	{
//		Revolver_mode  = REVOL_POSI_MODE;
//		actShoot = SHOOT_BUFF;
//		shoot_left_time = 0;
//	}
	else
	{
		actShoot = SHOOT_NORMAL;
		Shoot_Interval  = 0;//重置射击间隔
		Revol_Posit_RespondTime = system_time;//重置响应
		shoot_left_time = 0;
		Key_ShootNum = 0;
	}
//	
//	if(GIMBAL_IfBuffHit() == FALSE)//退出了打符模式
//	{
//		First_Into_Buff = TRUE;	
//		Buff_Shoot_Begin = FALSE;
//		buff_fire = FALSE;
//	}
}

/**
  * @brief  单发控制
  * @param  void
  * @retval void
  * @attention  
  */
void SHOOT_SINGLE_Ctrl(void)
{
	long  CurrentTime = 0;
	static uint32_t  RespondTime = 0;//响应间隔计时
	
	CurrentTime = system_time;
	
	Shoot_Interval = 125;//1000/8;//最快一秒8发
	
	if(RespondTime < CurrentTime
			&& Revol_Switch_Left == 2//与弹仓开关同理
				&& Key_ShootNum == 0)
	{
		RespondTime = CurrentTime + Shoot_Interval;
		Key_ShootNum++;
	}
}

/**
  * @brief  连发控制
  * @param  void
  * @retval void
  * @attention  
  */
void SHOOT_TRIPLE_Ctrl(void)
{
	Revolver_mode = REVOL_SPEED_MODE;

	if(JUDGE_usGetShootCold() <= 40)
	{
		Revolver_Freq = 8;//射频选择
	}
	else if(JUDGE_usGetShootCold() <= 60 && JUDGE_usGetShootCold() > 40)
	{
		Revolver_Freq = 8;//10;//射频选择
	}
	else if(JUDGE_usGetShootCold() <= 80 && JUDGE_usGetShootCold() > 60)
	{
		Revolver_Freq = 8;//12;//射频选择
	}
	else if(JUDGE_usGetShootCold() >= 160)//占领碉堡
	{
		Revolver_Freq = 14;//12;//射频选择
	}
	else
	{
		Revolver_Freq = 8;//射频选择
	}
	
	//速度环转速设置
	Revolver_Speed_Target = REVOL_SPEED_FRE*Revolver_Freq;
}

/**
  * @brief  高射频低射速控制
  * @param  void
  * @retval void
  * @attention  
  */
void SHOOT_HIGHTF_LOWS_Ctrl(void)
{
    long CurrentTime;
	static uint32_t  RespondTime = 0;//响应间隔计时	
	CurrentTime=system_time;
	Shoot_Interval =5; //1000/20;//确定射频
	
	if(RespondTime < CurrentTime
			&& Key_ShootNum == 0)
	{
		RespondTime = CurrentTime + Shoot_Interval;
		Key_ShootNum++;
	}
}

/**
  * @brief  中射频高射速控制
  * @param  void
  * @retval void
  * @attention  
  */
void SHOOT_MIDF_HIGHTS_Ctrl(void)
{
	long  CurrentTime = 0;
	static uint32_t  RespondTime = 0;//响应间隔计时	
	
	CurrentTime = system_tick;//当前系统时间
	
	Shoot_Interval = 10;//TIME_STAMP_1000MS/10;//确定射频
	
	if(RespondTime < CurrentTime
			&& Key_ShootNum == 0)
	{
		RespondTime = CurrentTime + Shoot_Interval;
		Key_ShootNum++;
	}
}

/**
  * @brief  自瞄射击控制
  * @param  void
  * @retval void
  * @attention  
*/
void SHOOT_AUTO_Ctrl(void)
{
	long CurrentTime;
	static uint32_t RespondTime_Stop    = 0;//响应间隔计时，静止
	static uint32_t RespondTime_MobiPre = 0;//响应间隔计时，移动预测
	CurrentTime = system_tick;

/***********************************************************************/
	if( if_yaw_auto_pre == TRUE)	//开启了预测			
	{
		Shoot_Interval = 6;//1000/15;//TIME_STAMP_50MS;//确定射频
		if(if_yaw_auto_pre==TRUE				//自己算预测到了位置
				&& RespondTime_MobiPre < CurrentTime
					&& Key_ShootNum == 0 
						&& IF_MOUSE_PRESSED_LEFT)//左键按下
		{
			RespondTime_MobiPre = CurrentTime + Shoot_Interval;
			Key_ShootNum ++;
		}
		else//开启了预测但预测不到位，禁止打弹
		{
			Key_ShootNum = 0;
		}
	}
	else if(if_yaw_auto_pre == FALSE)	//没开预测
	{
		Shoot_Interval = 100;//TIME_STAMP_1000MS/5;//确定射频
		if(if_yaw_auto_pre==TRUE		//自己算预测到了位置
				&& RespondTime_Stop < CurrentTime
					&& Key_ShootNum == 0
						&& IF_MOUSE_PRESSED_LEFT)//左键按下				
		{
			RespondTime_Stop = CurrentTime + Shoot_Interval;//TIME_STAMP_500MS;//每隔1s三连发一次		
			Key_ShootNum += 3;
//			Key_ShootNum ++;
		}
	}
}

/**
  * @brief  键盘模式拨盘速度环控制
  * @param  void
  * @retval void
  * @attention 推家模式超高射频,后续记得加入一个标志位用来告诉摩擦轮调低射速
  */
void REVOLVER_KeySpeedCtrl(void)
{
//	REVOL_SpeedStuck();//卡弹判断及倒转
}

/**
  * @brief  键盘模式拨盘位置环控制
  * @param  void
  * @retval void
  * @attention 
  */
void REVOLVER_KeyPosiCtrl(void)
{
	long  CurrentTime ;
//	static uint32_t  RespondTime = 0;//响应间隔计时
	
	CurrentTime = system_time;
	
	if(Key_ShootNum != 0 && Revol_Posit_RespondTime < CurrentTime)
	{
		Revol_Posit_RespondTime = CurrentTime + Shoot_Interval;
		Key_ShootNum--;//发弹计数减
		Revolver_Target_Sum_temp += AN_BULLET;//拨盘位置加
		
//		posishoot_time = xTaskGetTickCount();//单发指令下达时的系统时间,用于发射延时测试
	}		
	
	if(Revolver_Angle_Target_Sum != Revolver_Target_Sum_temp)//缓慢转过去
	{
		Revolver_Angle_Target_Sum = RAMP_float(Revolver_Target_Sum_temp, Revolver_Angle_Target_Sum, Revolver_Ramp);
	}
	
//	REVOL_PositStuck();//卡弹判断及倒转,放前面更合理一点
}
/**
  * @brief  统计转过角度总和
  * @param  void
  * @retval void
  * @attention 切换了模式之后记得清零 
  */
void REVOL_UpdateMotorAngleSum(void)
{		 
	//临界值判断法
	if (ABS(Revolver_Angle_Measure - Revolver_Angle_Measure_Prev) > 4095)//转过半圈
	{		
		//本次测量角度小于上次测量角度且过了半圈,则说明本次过了零点
		if (Revolver_Angle_Measure < Revolver_Angle_Measure_Prev)//过半圈且过零点
		{
			//已经转到下一圈,则累计转过 8191(一圈) - 上次 + 本次
			Revolver_Angle_Measure_Sum += 8191 - Revolver_Angle_Measure_Prev + Revolver_Angle_Measure;
		}
		else
		{
			//超过了一圈
			Revolver_Angle_Measure_Sum -= 8191 - Revolver_Angle_Measure + Revolver_Angle_Measure_Prev;
		}
	}
	else      
	{
		//未过临界值,累加上转过的角度差
		Revolver_Angle_Measure_Sum += Revolver_Angle_Measure - Revolver_Angle_Measure_Prev;
	}

	//记录此时电机角度,下一次计算转过角度差用,用来判断是否转过1圈
	Revolver_Angle_Measure_Prev = Revolver_Angle_Measure;
}


/***********************PID控制**********************/

/**
  * @brief  速度环PID控制
  * @param  void
  * @retval void
  * @attention  遥控只有速度环
  */
void REVOL_SpeedLoop(void)
{  
	Revolver_Speed_Error = Revolver_Speed_Target - Revolver_Speed_Measure;

	//典型单级PID算法
	pTermRevolSpeed   = Revolver_Speed_Error * Revolver_Speed_kpid[kp];
	iTermRevolSpeed  += Revolver_Speed_Error * Revolver_Speed_kpid[ki];
	iTermRevolSpeed   = constrain_float( iTermRevolSpeed, -iTermRevolSpeedMax, iTermRevolSpeedMax );

	Revolver_Final_Output = constrain_float( pTermRevolSpeed + iTermRevolSpeed, -Revolver_Output_Max, +Revolver_Output_Max );
}

/**
  * @brief  位置环PID控制
  * @param  void
  * @retval void
  * @attention  键盘模式
  */
void REVOL_PositionLoop(void)
{
	//获取转过的总角度值
	REVOL_UpdateMotorAngleSum();
	
	//外环计算
	Revolver_Angle_Error[OUTER] = Revolver_Angle_Target_Sum - Revolver_Angle_Measure_Sum;
	pTermRevolAngle[OUTER] = Revolver_Angle_Error[OUTER] * Revolver_Angle_kpid[OUTER][kp];

	//内环计算
	Revolver_Angle_Error[INNER]  =  pTermRevolAngle[OUTER] - Revolver_Speed_Measure;
	pTermRevolAngle[INNER]   = Revolver_Angle_Error[INNER] * Revolver_Angle_kpid[INNER][kp];		
	iTermRevolAngle[INNER]  += Revolver_Angle_Error[INNER] * Revolver_Angle_kpid[INNER][ki] * 0.001f;
	iTermRevolAngle[INNER]   = constrain_float( iTermRevolAngle[INNER], -iTermRevolPosiMax, iTermRevolPosiMax );

	Revolver_Final_Output = constrain_float( pTermRevolAngle[INNER] + iTermRevolAngle[INNER] , -Revolver_Output_Max, Revolver_Output_Max);

}


void REVOLVER_CANbusCtrlMotor(int16_t v)
{
    CAN_TxHeaderTypeDef tx_header;
	uint8_t             tx_data[8];
	
	tx_header.StdId = 0x1ff;
	tx_header.IDE   = CAN_ID_STD;
	tx_header.RTR   = CAN_RTR_DATA;
	tx_header.DLC   = 2;
    tx_data[0] = (v>>8)&0xff;
	tx_data[1] =    (v)&0xff;
    
    HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data,(uint32_t*)CAN_TX_MAILBOX0);
}

/**
  * @brief  枪管热量限制
  * @param  void
  * @retval 热量是否超限
  * @attention  超限要重置一下拨盘,根据剩余可发弹量来做闭环
  *             如果做双枪管则此函数不适用
  */
bool Revolver_Heat_Limit(void)
{return 1;}

uint16_t JUDGE_usGetShootCold(void)
{
    return 100;
}
