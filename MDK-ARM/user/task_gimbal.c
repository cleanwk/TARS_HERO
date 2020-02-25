#include "task_gimbal.h"


/*************云台系统************/
eGimbalCtrlMode  modeGimbal;
eGimbalAction actGimbal;
#if		INFANTRY_DEBUG_ID == 3

/*---------------------云台相关变量------------------------*/ 
	#define Mech_Min_Pitch     126   //down              
	#define Mech_Mid_Pitch     704                     
	#define Mech_Max_Pitch     1496    //up           
	#define Mech_Right_Yaw     5974     //right max         
	#define Mech_Mid_Yaw       6474                     
	#define Mech_Left_Yaw      6974     //left max        

	//云台抬头角度,用于摩擦轮开启
	#define CLOUD_FRIC_PIT_UP  (Mech_Mid_Pitch + 200)   


	//云台底盘分离角度,用于yaw限位
	#define CLOUD_SEPAR_ANGLE  800//小心会跟扭头模式冲突//待测

	
	//吊射角度
	//float base_mech_pitch = 3960;                       //待测

    
float down_sb_pitch = 530;//待测
float up_sb_pitch   = 0;//待测
 
#endif
/*************灵敏度************/

//机械模式下比例系数,控制摇杆响应速度
float kRc_Mech_Pitch, kRc_Mech_Yaw;

//陀螺仪模式下比例系数,控制摇杆响应速度
float kRc_Gyro_Pitch, kRc_Gyro_Yaw;
float krc_gyro_yaw = 0.018;

//机械模式下比例系数,控制键盘响应速度
float kKey_Mech_Pitch, kKey_Mech_Yaw;

//陀螺仪模式下比例系数,控制键盘响应速度
float kKey_Gyro_Pitch, kKey_Gyro_Yaw;
float kkey_gyro_yaw = 0.38;

float angleMpuPitch,	angleMpuYaw,	angleMpuRoll;//陀螺仪角度值
short palstanceMpuPitch,	palstanceMpuYaw,	palstanceMpuRoll;//陀螺仪角速度值


//期望角度
float Cloud_Angle_Target[2][2];//  pitch/yaw    mech/gyro

//测量角度
float Cloud_Angle_Measure[2][2];//  pitch/yaw    mech/gyro

//测量角速度
float Cloud_Palstance_Measure[2][2];//  pitch/yaw    mech/gyro

/*******************PID参数**********************/
Cascade_pid_t Pitch_mech_pid;
Cascade_pid_t Yaw_mech_pid;
Cascade_pid_t Yaw_gyro_pid;
float pitch_out;
float yaw_out;
float Gimbal_PID_list[2][2][3][2][3];        //pitch/yaw  mech/gyro  模式   OUTER/INNER   KP/KI/KD
/**************限幅****************/
//限制云台电机电流最大输出量


/**************斜坡***************/
float Slope_Mouse_Pitch, Slope_Mouse_Yaw;//摩擦轮开启时抬头快慢
	
//上电斜坡变量
float Slope_Begin_Pitch, Slope_Begin_Yaw;//刚上电时移动快慢

//键盘陀螺仪模式下鼠标统计yaw偏移量,此值会自己缓慢减小,防止甩头过快
float Mouse_Gyro_Yaw, Mouse_Gyro_Pitch;

//键盘陀螺仪模式下QEC扭头快慢
float Slope_Turn_Yaw;
float Slope_Back_Yaw;

//开启摩擦轮抬头斜坡
float Slope_Fric_Pitch;

/***************自瞄******************/
uint8_t if_yaw_auto_pre;
//误差
float Auto_Error_Yaw[2];//    now/last
float Auto_Error_Pitch[2];
float Auto_Distance;//距离单目

//自瞄斜坡
float Slope_Auto_Yaw;
float Slope_Auto_Pitch;

//自瞄突然开启,卡尔曼滤波开启延时
uint16_t Auto_KF_Delay = 0;

float   debug_y_sp_k;// = 38;//35;//30;//移动预测系数,越大预测越多
float   debug_y_sen_sp_k;//哨兵预测系数
float   debug_y_sen_brige_sp_k;//桥头哨兵
float   debug_p_sp_k;//移动预测系数,越大预测越多
float   debug_auto_err_y;// = 10;//15;//10;//15;//yaw角度过大关闭预测
float   debug_auto_err_p;//pitch角度过大关闭预测
int     debug_kf_delay;// = 150;//100;//200;//120;//150;//预测延时开启
float   debug_kf_speed_y_low;//yaw速度过低关闭预测
float   debug_kf_speed_y_low_sen;//抬头打哨兵时减小最低可开预测量
float   debug_kf_speed_y_high;//yaw速度过高关闭预测
float   debug_kf_speed_p_low;//pitch速度过低关闭预测
float   debug_kf_y_angcon;// = 130;//125;//115;//135;//yaw预测量限幅
float   debug_kf_p_angcon;//pitch预测量限幅

float   Vision_Angle_Speed_Yaw, Vision_Angle_Speed_Pitch;//卡尔曼滤波速度测量值
float   *yaw_kf_result, *pitch_kf_result;//二阶卡尔曼滤波结果,0角度 1速度
/*************卡尔曼滤波**************/
/*一阶卡尔曼*/
//云台角度误差卡尔曼
extKalman_t Gimbal_Pitch_Mech_Error_Kalman;//定义一个kalman指针

extKalman_t Gimbal_Yaw_Mech_Error_Kalman;//定义一个kalman指针
extKalman_t Gimbal_Yaw_Gyro_Error_Kalman;//定义一个kalman指针

extKalman_t Vision_Distance_Kalman;

speed_calc_data_t Vision_Yaw_speed_Struct;
speed_calc_data_t Vision_Pitch_speed_Struct;

kalman_filter_init_t yaw_kalman_filter_para = {
  .P_data = {2, 0, 0, 2},
  .A_data = {1, 0.002/*0.001*/, 0, 1},//采样时间间隔
  .H_data = {1, 0, 0, 1},
  .Q_data = {1, 0, 0, 1},
  .R_data = {200, 0, 0, 400}//500 1000
};//初始化yaw的部分kalman参数

kalman_filter_init_t pitch_kalman_filter_para = {
  .P_data = {2, 0, 0, 2},
  .A_data = {1, 0.002/*0.001*/, 0, 1},//采样时间间隔
  .H_data = {1, 0, 0, 1},
  .Q_data = {1, 0, 0, 1},
  .R_data = {200, 0, 0, 400}
};//初始化pitch的部分kalman参数

kalman_filter_t yaw_kalman_filter;
kalman_filter_t pitch_kalman_filter;

/*自动打弹用的一些标志位*/
bool Mobi_Prediction_Yaw = FALSE;//预测是否开启标志位
bool Mobi_Pre_Yaw_Fire = FALSE;//默认预测没到位，禁止开枪

uint16_t mobpre_yaw_left_delay = 0;//向左预测延时判断可开火消抖
uint16_t mobpre_yaw_right_delay = 0;//向右预测延时判断可开火消抖
uint16_t mobpre_yaw_stop_delay = 0;//预测关闭延时判断可开火消抖
/****************云台键盘模式下各小函数辅助变量********************/
//调头模式角度目标
float TURNMode_Yaw_Back_Total;//按下C,yaw需要改变的角度值
float TURNMode_Yaw_Turn_Total;//按下QE,yaw需要改变的角度值,正负代表左右转


void Gimbal_value(void)
{
	Cloud_Angle_Measure[YAW][MECH]=GM6020[0].rotor_angle;
	Cloud_Palstance_Measure[YAW][MECH]=GM6020[0].rotor_speed;
	Cloud_Angle_Measure[PITCH][MECH]=GM6020[1].rotor_angle;
	Cloud_Palstance_Measure[PITCH][MECH]=GM6020[1].rotor_speed;
	
	Cloud_Angle_Measure[YAW][GYRO]=yaw_angle;
	Cloud_Palstance_Measure[YAW][GYRO]=yaw_w;	
}



/**
  * @brief  云台参数初始化
  * @param  void
  * @retval void
  * @attention 只在系统启动时调用一次
  */
void GIMBAL_InitArgument(void)
{
    modeGimbal = MECH;//默认以机械模式启动
    
    /* 灵敏度,响应快慢 */	
	kRc_Mech_Yaw   = -0.01;
	kRc_Mech_Pitch = 0.01;	
	kRc_Gyro_Yaw   = -0.15;//调节陀螺仪模式下转头速度快慢，影响陀螺仪模式下车扭头速度

	
	kKey_Mech_Yaw   = 0.5;
	kKey_Mech_Pitch = 0.45;	
	kKey_Gyro_Yaw   = -0.38;//注意正负,否则会反向

	
	/* 斜坡,变化快慢 */
	Slope_Begin_Pitch =  1;//上电时移动快慢
  	Slope_Begin_Yaw   =  1;//上电时移动快慢
	
	Slope_Mouse_Pitch = 15;//20;//鼠标响应,抬头低头速度
	Slope_Mouse_Yaw   = 15;//鼠标响应,扭头速度
	
	Slope_Turn_Yaw = 20;//25;//QE扭头快慢
	Slope_Back_Yaw = 20;//30;//C调头快慢
	
	Slope_Auto_Yaw   = 4;//1;//50;
	Slope_Auto_Pitch = 3;//1;//50;
	
	Slope_Fric_Pitch = 8;
    
    //普通
	Gimbal_PID_list[PITCH][MECH][GIMBAL_NORMAL][OUTER][kp]=90;//90;
	Gimbal_PID_list[YAW][MECH][GIMBAL_NORMAL][OUTER][kp]=90;
			
				//inner
	Gimbal_PID_list[PITCH][MECH][GIMBAL_NORMAL][INNER][kp]=250;//144;
	Gimbal_PID_list[PITCH][MECH][GIMBAL_NORMAL][INNER][ki]=1;//1;
	Gimbal_PID_list[PITCH][MECH][GIMBAL_NORMAL][INNER][kd]=0;
			
	Gimbal_PID_list[YAW][MECH][GIMBAL_NORMAL][INNER][kp]=180;
	Gimbal_PID_list[YAW][MECH][GIMBAL_NORMAL][INNER][ki]=1;
	Gimbal_PID_list[YAW][MECH][GIMBAL_NORMAL][INNER][kd]=0;			
			/* kPID,陀螺仪模式 */
				//outer		
	Gimbal_PID_list[YAW][GYRO][GIMBAL_NORMAL][OUTER][kp]=100;
			
				//inner		
	Gimbal_PID_list[YAW][GYRO][GIMBAL_NORMAL][INNER][kp]=80;
	Gimbal_PID_list[YAW][GYRO][GIMBAL_NORMAL][INNER][ki]=0;
	Gimbal_PID_list[YAW][GYRO][GIMBAL_NORMAL][INNER][kd]=0;
    
	Gimbal_PID_list[PITCH][MECH][GIMBAL_AUTO][OUTER][kp]=0;
	Gimbal_PID_list[YAW][MECH][GIMBAL_AUTO][OUTER][kp]=0;
			
				//inner
	Gimbal_PID_list[PITCH][MECH][GIMBAL_AUTO][INNER][kp]=0;
	Gimbal_PID_list[PITCH][MECH][GIMBAL_AUTO][INNER][ki]=0;
	Gimbal_PID_list[PITCH][MECH][GIMBAL_AUTO][INNER][kd]=0;
			
	Gimbal_PID_list[YAW][MECH][GIMBAL_AUTO][INNER][kp]=0;
	Gimbal_PID_list[YAW][MECH][GIMBAL_AUTO][INNER][ki]=0;
	Gimbal_PID_list[YAW][MECH][GIMBAL_AUTO][INNER][kd]=0;
			
			/* kPID,陀螺仪模式 */
				//outer
	Gimbal_PID_list[YAW][GYRO][GIMBAL_AUTO][OUTER][kp]=0;
			
				//inner		
	Gimbal_PID_list[YAW][GYRO][GIMBAL_AUTO][INNER][kp]=0;
	Gimbal_PID_list[YAW][GYRO][GIMBAL_AUTO][INNER][ki]=0;
	Gimbal_PID_list[YAW][GYRO][GIMBAL_AUTO][INNER][kd]=0;	
    

    
    //卡尔曼滤波器初始化
	/*PID角度误差卡尔曼,一阶*/
	KalmanCreate(&Gimbal_Pitch_Mech_Error_Kalman, 1, 40);

	
	KalmanCreate(&Gimbal_Yaw_Mech_Error_Kalman, 1, 40);
	KalmanCreate(&Gimbal_Yaw_Gyro_Error_Kalman, 1, 40);
	
	KalmanCreate(&Vision_Distance_Kalman, 1, 2000);
    
    
    
    
}




/**
  * @brief  云台失控保护
  * @param  void
  * @retval void
  * @attention 所有输出置0
  */
void GIMBAL_StopMotor(void)
{
    
}

/**
  * @brief  云台初始化
  * @param  void
  * @retval void
  * @attention 
  */
int GIMBAL_InitCtrl(void)
{    
    static bool bTick_Record  = FALSE;
    static bool bAngle_Record  = FALSE;
    Gimbal_value();	//赋值
    if(bTick_Record == FALSE)
    {   
        if (system_tick > 5)//保证不断电情况下下次可用        
		  bTick_Record = TRUE;
    }    
    else
    {
        //记录上电时云台机械角度
        if (bAngle_Record == FALSE)
        {
            bAngle_Record = TRUE;
			
            Cloud_Angle_Target[PITCH][MECH] = GM6020[PITCH].rotor_angle;
            Cloud_Angle_Target[YAW][MECH] = GM6020[YAW].rotor_angle;
        }
	
                    //记录陀螺仪初始角度
        else
        {

            //平缓地让云台移动到中间,防止刚上电狂甩
            Cloud_Angle_Target[PITCH][MECH] = RAMP_float( Mech_Mid_Pitch, Cloud_Angle_Target[PITCH][MECH], Slope_Begin_Pitch);
            if(GIMBAL_IfPitchLevel())   
            {
                Cloud_Angle_Target[YAW][MECH]   = RAMP_float( Mech_Mid_Yaw, Cloud_Angle_Target[YAW][MECH], Slope_Begin_Yaw);
                if(ABS(GIMBAL_GetOffsetAngle())<=50)
                    return 1;        
            }
            
        }
    }
    return 0;
}


/**
  * @brief  遥控控制云台模式
  * @param  void
  * @retval void
  * @attention 在此改变角度环目标值
  */
void GIMBAL_Rc_Ctrl(void)
{
    	if(modeGimbal == GYRO)
	{
		//限制云台与底盘分离角度
		Gimbal_Chass_Separ_Limit();
	}
	
	if (IF_RC_SW2_DOWN)//s2拨到下,陀螺仪模式
	{
		modeGimbal = GYRO;
	}
	else if (IF_RC_SW2_MID)//S2中
	{
		modeGimbal = MECH;//S2中,机械模式
	}
	
	/*-----遥控控制弹仓开启------*/
	if (Magazine_opened_flag)
	{
		Cloud_Angle_Target[PITCH][MECH] = Mech_Mid_Pitch;

		if (modeGimbal == MECH)//机械模式下yaw归中
		{
			Cloud_Angle_Target[YAW][MECH] = Mech_Mid_Yaw;
		}
		else if (modeGimbal == GYRO)//陀螺仪模式下摇杆控制
		{
			Cloud_Angle_Target[YAW][GYRO] += -rc_t.rc.ch1*kRc_Gyro_Yaw;
		}

	}
	else    
	{
		//正常控制pitch
		if (modeGimbal == MECH)
		{
			Cloud_Angle_Target[PITCH][MECH] += rc_t.rc.ch2*kRc_Mech_Pitch; 
			Cloud_Angle_Target[YAW][MECH]   = rc_t.rc.ch1*kRc_Mech_Yaw; //机械模式,yaw固定在中
			
			Cloud_Angle_Target[YAW][GYRO] = Cloud_Angle_Measure[YAW][GYRO];
		}
		else if (modeGimbal == GYRO)
		{
			Cloud_Angle_Target[PITCH][MECH] += -rc_t.rc.ch2*kRc_Mech_Pitch;//pitch仍用机械处理方式  
			Cloud_Angle_Target[YAW][GYRO]   += -rc_t.rc.ch1*kRc_Gyro_Yaw; 
		}
	}	
}


/**
  * @brief  键盘控制云台模式
  * @param  void
  * @retval void
  * @attention 
  */              
void GIMBAL_Key_Ctrl(void)
{
    if(modeGimbal == GYRO)
	{
		//限制云台与底盘分离角度
		Gimbal_Chass_Separ_Limit();
	}
	
	switch(actGimbal)
	{
		/*--------------云台模式选择----------------*/
		case GIMBAL_NORMAL:
			GIMBAL_NORMAL_Mode_Ctrl();//在此选择控制模式
		break;
		
		/*--------------V  180°调头----------------*/
		case GIMBAL_AROUND:
			modeGimbal = GYRO;//进入陀螺仪模式
		
			if (TURNMode_Yaw_Back_Total == 0)
			{
				actGimbal = GIMBAL_NORMAL;
			}
			else
			{
				Cloud_Angle_Target[YAW][GYRO] = RampInc_float( &TURNMode_Yaw_Back_Total, Cloud_Angle_Target[YAW][GYRO], Slope_Back_Yaw );
			}
		break;
		
		/*------------弹仓开启,禁止抬头-----------------*/
		case GIMBAL_LEVEL:
			GIMBAL_LEVEL_Mode_Ctrl();
		break;
		
		/*--------------Q E  90°调头----------------*/
		case GIMBAL_TURN:				
			modeGimbal = GYRO;//进入陀螺仪模式

		    if (TURNMode_Yaw_Turn_Total == 0)
			{
				actGimbal = GIMBAL_NORMAL;
			}
			else
			{
				Cloud_Angle_Target[YAW][GYRO] = RampInc_float( &TURNMode_Yaw_Turn_Total, Cloud_Angle_Target[YAW][GYRO], Slope_Back_Yaw );
			}
		break;
			
		/*--------------右键自瞄----------------*/	
		case GIMBAL_AUTO:
			modeGimbal = GYRO;//进入陀螺仪模式
		
		if(!IF_MOUSE_PRESSED_RIGH)//松开右键退出自瞄
		{
				actGimbal = GIMBAL_NORMAL;
				
//				//自瞄目标偏差清零,避免切换时云台跳动
//				VisionRecvData.identify_target = FALSE;
//				Auto_KF_Delay = 0;//清零给下次延迟预测用
//				Mobility_Prediction_Yaw = FALSE;//标记预测没开启
//				Mobi_Pre_Yaw_Fire = FALSE;//默认标记预测没到位，禁止开火
//				
//				mobpre_yaw_left_delay  = 0;//重置左预测的开火延迟
//				mobpre_yaw_right_delay = 0;//重置右预测的开火延迟	
//				mobpre_yaw_stop_delay = 0;//停止预测开火延时重置
		}
		else
		{
			GIMBAL_AUTO_Mode_Ctrl();//自瞄控制函数
		}
		break;
    }
}



/*******************云台键盘模式各类模式小函数*******************/

/**
  * @brief  云台键盘模式选择,按键响应
  * @param  void
  * @retval void
  * @attention 云台键盘控制状态下的所有模式切换都在这
  * 无模式切换时一直处于此模式
  */
void GIMBAL_NORMAL_Mode_Ctrl(void)
{
	//按键延时响应,防止手贱狂按
	static uint32_t  Key_Ctrl_CurrentTime = 0;
	static uint32_t PressV_Time  = 0;//调头,500ms延时响应,1秒最多按2下
	static uint32_t PressQ_Time  = 0;//90°,250ms延时响应,1秒最多按4下
    static uint32_t PressE_Time  = 0;//90°,250ms延时响应,1秒最多按4下
//	static uint32_t PressCF_Time  = 0;//打大符,400ms延时响应
//	static uint32_t PressCG_Time  = 0;//手动打符,400ms延时响应
//	static uint32_t PressCV_Time  = 0;//打小符,400ms延时响应
	static uint32_t Mouse_Yaw_Stop  = 0;//鼠标不动，结束响应
	static uint32_t Mouse_Pitch_Stop  = 0;//鼠标不动，结束响应
	
	Key_Ctrl_CurrentTime = system_tick;//获取实时时间,用来做按键延时判断	
	
	
//	if ( CHASSIS_IfActiveMode() == TRUE || Magazine_IfOpen() ==	 TRUE)//获取底盘模式,true为机械模式
//	{
//		modeGimbal = MECH;
//	} 
//	else					//注释掉loop中的底盘会令陀螺仪模式失效
//	{
//		modeGimbal = GYRO;
//	}

	
	
	if ( !IF_KEY_PRESSED_CTRL && IF_KEY_PRESSED_V
					&& Key_Ctrl_CurrentTime > PressV_Time)
	{   //Ctrl不处于按下状态时按V调头
		actGimbal  =  GIMBAL_AROUND;//切换成调头模式

		PressV_Time = Key_Ctrl_CurrentTime + 50;//500ms延时防手贱狂按

		if(IF_KEY_PRESSED_A)//AV左调头
		{
			TURNMode_Yaw_Back_Total = 3579;
		}
		else if(IF_KEY_PRESSED_D)//DV右调头
		{
			TURNMode_Yaw_Back_Total = -3579;
		}
		else//默认右调头
		{
			TURNMode_Yaw_Back_Total = -3579;//因为角度放大了20倍,所以是180°*20约等于3579
		}
	}
	/*---------------------------------*/	
	else if ( !IF_KEY_PRESSED_CTRL
				&& ( (IF_KEY_PRESSED_Q && Key_Ctrl_CurrentTime > PressQ_Time)
					|| (IF_KEY_PRESSED_E && Key_Ctrl_CurrentTime > PressE_Time) ) )
	{   //Ctrl不处于按下状态时按Q(左),E(右)90°调头
		actGimbal = GIMBAL_TURN;//切换成快速扭头模式
		
		//注意方向
		if ( IF_KEY_PRESSED_Q)
		{
			PressQ_Time = Key_Ctrl_CurrentTime + 25;//250ms延时防手贱狂按
			
			TURNMode_Yaw_Turn_Total = 1789;//Q左转约8192/4度
		}
		else if (IF_KEY_PRESSED_E)
		{
			PressE_Time = Key_Ctrl_CurrentTime + 25;//250ms延时防手贱狂按
			
			TURNMode_Yaw_Turn_Total = -1789;//E右转约8192/4度
		}
			
	}	
	/*---------------------------------*/
	else if (Magazine_opened_flag)
	{
		actGimbal = GIMBAL_LEVEL;

	}
	/*---------------------------------*/
	else if (IF_MOUSE_PRESSED_RIGH)//若SW1不在中,则右键自瞄
	{
		actGimbal = GIMBAL_AUTO;

	}
//	/*----------------小符-----------------*/
//	else if(IF_KEY_PRESSED_V && IF_KEY_PRESSED_CTRL && Key_Ctrl_CurrentTime > PressCV_Time)//Ctrl+F打符,400ms响应一次
//	{
//		PressCV_Time = Key_Ctrl_CurrentTime + TIME_STAMP_400MS;
//		actGimbal = GIMBAL_SM_BUFF;
//	}
//	/*----------------大符-----------------*/
//	else if(IF_KEY_PRESSED_F && IF_KEY_PRESSED_CTRL && Key_Ctrl_CurrentTime > PressCF_Time)//Ctrl+F打符,400ms响应一次
//	{
//		PressCF_Time = Key_Ctrl_CurrentTime + TIME_STAMP_400MS;
//		actGimbal = GIMBAL_BUFF;
//	}
//	/*----------------吊射-----------------*/
//	else if(IF_KEY_PRESSED_C && !IF_MOUSE_PRESSED_RIGH && !IF_RC_SW1_MID)
//	{
//		actGimbal = GIMBAL_BASE;
//	}
	/*---------------------------------*/
//	else if(IF_KEY_PRESSED_G && IF_KEY_PRESSED_CTRL && Key_Ctrl_CurrentTime > PressCG_Time)//Ctrl+G手动打符,400ms响应一次
//	{
//		PressCG_Time = Key_Ctrl_CurrentTime + TIME_STAMP_400MS;
//		actGimbal = GIMBAL_MANUAL;
//	}
	/*---------------------------------*/
	else       //最后做云台角度计算,这是普通模式下的角度计算,优先级最低,所以放最后面
	{
		if (modeGimbal == MECH)//机械模式
		{
			Cloud_Angle_Target[PITCH][MECH] += MOUSE_Y_MOVE_SPEED * kKey_Mech_Pitch;
			Cloud_Angle_Target[YAW][MECH]   = Mech_Mid_Yaw;	//yaw保持不动	//yaw保持不动,永远在中间
			
			Cloud_Angle_Target[YAW][GYRO] = Cloud_Angle_Measure[YAW][GYRO];
			
		}
		else if (modeGimbal == GYRO)
		{
			Mouse_Gyro_Yaw   += MOUSE_X_MOVE_SPEED * kKey_Gyro_Yaw;//记录目标变化角度
			Mouse_Gyro_Pitch += MOUSE_Y_MOVE_SPEED * kKey_Gyro_Pitch;//pitch仍旧使用机械模式

			if(MOUSE_X_MOVE_SPEED == 0)
			{
				Mouse_Yaw_Stop ++ ;
				if(Mouse_Yaw_Stop > 25)//鼠标长时间停留，停止移动
				{
					Mouse_Gyro_Yaw = 0;
				}
			}
			else
			{
				Mouse_Yaw_Stop = 0;
			}
			
			if(MOUSE_Y_MOVE_SPEED == 0)
			{
				Mouse_Pitch_Stop ++ ;
				if(Mouse_Pitch_Stop > 25)//鼠标长时间停留，停止移动
				{
					Mouse_Gyro_Pitch = 0;
				}
			}
			else
			{
				Mouse_Pitch_Stop = 0;
			}
			Cloud_Angle_Target[YAW][GYRO]   = RampInc_float( &Mouse_Gyro_Yaw, Cloud_Angle_Target[YAW][GYRO], Slope_Mouse_Yaw );
			Cloud_Angle_Target[PITCH][MECH] = RampInc_float( &Mouse_Gyro_Pitch, Cloud_Angle_Target[PITCH][MECH], Slope_Mouse_Pitch );
		}
	}
}


/**
  * @brief  补弹模式
  * @param  void
  * @retval void
  * @attention 此模式下禁止控制pitch
  */
void GIMBAL_LEVEL_Mode_Ctrl(void)
{
	modeGimbal = MECH;//补弹时进入机械模式
	
	//补弹完毕,退出补弹模式
	if(!Magazine_opened_flag)
	{
		actGimbal = GIMBAL_NORMAL;
	}
	else//补弹未完成,角度固定在中间
	{
		Cloud_Angle_Target[YAW][MECH]   = Mech_Mid_Yaw;
		Cloud_Angle_Target[PITCH][MECH] = Mech_Mid_Pitch;
	}
}

/**
  * @brief  自瞄控制函数
  * @param  void
  * @retval void
  * @attention 中间坐标(0,0),左正右负,上负下正
  *             yaw为陀螺仪模式,pitch为机械模式(其实pitch全程都在用机械模式)
  *            只有当新数据来了才在当前实时角度上累加误差当成目标角度
  *            新数据一来,目标角度就实时更新
  */
uint32_t Vision_Time[2];// NOW/LAST

int vision_time_delta;//视觉延迟ms
//根据距离调节预测比例和限幅
float yaw_speed_k = 0;
float kf_yaw_angcon = 0;

float pitch_speed_k = 0;
float kf_pitch_angcon = 0;

int kf_delay_open = 0;
float kf_speed_yl = 0;//
int vision_time_delta;
void GIMBAL_AUTO_Mode_Ctrl(void)
{        
    float debug_kf_angle_temp;//预测角度斜坡暂存量
    float debug_kf_angle_ramp = 20;//预测角度斜坡变化量    
    float debug_kf_y_angle;//yaw预测暂存
    float debug_kf_p_angle;//pitch预测暂存
    static uint32_t Mouse_Yaw_Stop  = 0;//鼠标不动，结束响应
	static uint32_t Mouse_Pitch_Stop  = 0;//鼠标不动，结束响应

    static float yaw_angle_raw, pitch_angle_raw;//卡尔曼滤波角度测量值
    static float yaw_angle_ref;//记录目标角度
	static float pitch_angle_ref;//记录目标角度

    /*************************采样***********************/
    Auto_Error_Yaw[NOW]=VisionRecvData.yaw_angle;
    Auto_Error_Pitch[NOW]=VisionRecvData.pitch_angle;
    Auto_Distance=VisionRecvData.distance;
    if(Auto_Distance<0)
        Auto_Distance=0;
    Auto_Distance = KalmanFilter(&Vision_Distance_Kalman, Auto_Distance);

    /***********************数据更新*********************/
    if(Vision_Data_Flag)
    {
        //更新目标角度//记录当前时刻的目标位置,为卡尔曼做准备
		yaw_angle_ref   = Cloud_Angle_Measure[YAW][GYRO]   + Auto_Error_Yaw[NOW]   ;
		pitch_angle_ref = Cloud_Angle_Measure[PITCH][MECH] + Auto_Error_Pitch[NOW];
		
        Vision_Data_Flag=0;//清零
        Vision_Time[NOW] = system_time;//获取新数据到来时的时间
    }
       
    if(Vision_Time[NOW] != Vision_Time[LAST])//更新新数据到来的时间
	{
		vision_time_delta = Vision_Time[NOW] - Vision_Time[LAST];//计算视觉延迟
		yaw_angle_raw  = yaw_angle_ref;//更新二阶卡尔曼滤波测量值
		pitch_angle_raw = pitch_angle_ref;
		Vision_Time[LAST] = Vision_Time[NOW];
	}
    
    /*************************滤波***********************/
    	//目标速度解算
	if(VisionRecvData.identify_target == TRUE)//识别到了目标
	{
		Vision_Angle_Speed_Yaw = Target_Speed_Calc(&Vision_Yaw_speed_Struct, Vision_Time[NOW], yaw_angle_raw);
		Vision_Angle_Speed_Pitch = Target_Speed_Calc(&Vision_Pitch_speed_Struct, Vision_Time[NOW], pitch_angle_raw);
		//对角度和速度进行二阶卡尔曼滤波融合,0位置,1速度
		yaw_kf_result = kalman_filter_calc(&yaw_kalman_filter, yaw_angle_raw, Vision_Angle_Speed_Yaw);
		pitch_kf_result = kalman_filter_calc(&pitch_kalman_filter, pitch_angle_raw, Vision_Angle_Speed_Pitch);

	}
    //据说可以消除目标切换，自瞄开启关闭的抖动，有待验证
//	else
//	{
////		//对角度和速度进行二阶卡尔曼滤波融合,0位置,1速度
//		Vision_Angle_Speed_Yaw = Target_Speed_Calc(&Vision_Yaw_speed_Struct, system_time, Cloud_Angle_Measure[YAW][GYRO]);
//		Vision_Angle_Speed_Pitch = Target_Speed_Calc(&Vision_Pitch_speed_Struct, system_time, Cloud_Angle_Measure[PITCH][MECH]);
////		//对角度和速度进行二阶卡尔曼滤波融合,0位置,1速度
//		yaw_kf_result = kalman_filter_calc(&yaw_kalman_filter, Cloud_Angle_Measure[YAW][GYRO], 0);
//		pitch_kf_result = kalman_filter_calc(&pitch_kalman_filter, Cloud_Angle_Measure[PITCH][MECH], 0);
////		debug_kf_angle_temp = 0;
//	}
    
    /**********************yaw预测**********************/
    
    kf_delay_open = debug_kf_delay;
    if(VisionRecvData.identify_target == TRUE)
    {
        
        #if AUTO_PRE == 1
            //电控预测
        Auto_KF_Delay++;//滤波延时开启
        if(VisionRecvData.auto_too_close == TRUE)
        {
			yaw_speed_k = debug_y_sp_k;///4.f;//3.f;//预测系数减半
			kf_yaw_angcon = debug_kf_y_angcon;//3.f;//2.f;
			kf_speed_yl = debug_kf_speed_y_low;            
        }
        else
        {
            if( GIMBAL_AUTO_PITCH_Sentry() == TRUE )
			{
				yaw_speed_k = debug_y_sen_sp_k;			
				kf_yaw_angcon = debug_kf_y_angcon;
				kf_speed_yl = debug_kf_speed_y_low_sen;
				
            }
            else
            {
                yaw_speed_k = debug_y_sp_k;
				kf_yaw_angcon = debug_kf_y_angcon;
				kf_speed_yl = debug_kf_speed_y_low;
            }
        }
        /*预测开启条件*/
        if(ABS(Auto_Error_Yaw[NOW]) < debug_auto_err_y//debug看 
        && Auto_KF_Delay > kf_delay_open 
        && fabs(yaw_kf_result[KF_SPEED]) >= kf_speed_yl 
        && fabs(yaw_kf_result[KF_SPEED]) < debug_kf_speed_y_high )
		{
            if_yaw_auto_pre=1;
            if(yaw_kf_result[KF_SPEED]>=0)
			{
				debug_kf_angle_temp = yaw_speed_k * (yaw_kf_result[KF_SPEED] - kf_speed_yl) ;//debug_kf_dist;
			}
            else if(yaw_kf_result[KF_SPEED]<0)
			{
				debug_kf_angle_temp = yaw_speed_k * (yaw_kf_result[KF_SPEED] + kf_speed_yl) ;//debug_kf_dist;			
			}
            
            debug_kf_angle_temp = constrain_float(debug_kf_angle_temp, -debug_kf_y_angcon, debug_kf_y_angcon);//预测暂存量限幅
			debug_kf_y_angle = RAMP_float(debug_kf_angle_temp, debug_kf_y_angle, debug_kf_angle_ramp);//预测量缓慢变化
			
			debug_kf_y_angle = constrain_float(debug_kf_y_angle, -debug_kf_y_angcon, debug_kf_y_angcon);
			Cloud_Angle_Target[YAW][GYRO] = yaw_kf_result[KF_ANGLE] + debug_kf_y_angle;//debug_y_sk * (yaw_kf_result[KF_SPEED] - debug_kf_speed);//Vision_Gyro_MovProj_Yaw(yaw_kf_result[1]);//yaw_kf_result[0];
			
           
            /*↓↓↓↓↓↓↓↓↓↓↓↓↓↓预测到位判断↓↓↓↓↓↓↓↓↓↓↓↓↓*/
            if( (yaw_kf_result[KF_SPEED]>0) //目标向左移且误差值显示说目标在右边，则说明预测到位置，可打弹
            && (Auto_Error_Yaw[NOW] < 0.3f) )
            {
                mobpre_yaw_right_delay = 0;//向右预测开火延时重置

				mobpre_yaw_left_delay++;
                if(mobpre_yaw_left_delay > 0/*75*/)//稳定150ms
				{
					Mobi_Pre_Yaw_Fire = TRUE;//预测到位，可开火
				}
				else
				{
					Mobi_Pre_Yaw_Fire = FALSE;//预测没到位，不可开火
				}
            }
            else if( (yaw_kf_result[KF_SPEED]<0) //目标向右移且误差值显示说目标在左边，则说明预测到位置，可打弹
						&& (Auto_Error_Yaw[NOW] > -0.3f) )
			{
				mobpre_yaw_left_delay = 0;//向左预测开火延时重置
				
				mobpre_yaw_right_delay++;
				if(mobpre_yaw_right_delay > 0/*75*/)//稳定150ms
				{
					Mobi_Pre_Yaw_Fire = TRUE;//预测到位，可开火
				}
				else
				{
					Mobi_Pre_Yaw_Fire = FALSE;//预测没到位，不可开火
				}

            }
            else
			{
				Mobi_Pre_Yaw_Fire = FALSE;//标记预测没到位，禁止开火				
				mobpre_yaw_left_delay = 0;//向左预测开火延时重置
				mobpre_yaw_right_delay = 0;//向右预测开火延时重置
			}
            Mobi_Prediction_Yaw = TRUE;//标记预测已开启
			mobpre_yaw_stop_delay = 0;//重置静止时的开火延迟
        }
        /*预测条件没达到，关闭预测*/
		else
		{
            if_yaw_auto_pre=0;
            Cloud_Angle_Target[YAW][GYRO] = yaw_kf_result[KF_ANGLE];
			Mobi_Prediction_Yaw = FALSE;//标记预测没开启
			mobpre_yaw_left_delay  = 0;//重置左预测的开火延迟
			mobpre_yaw_right_delay = 0;//重置右预测的开火延迟	
			if( fabs(Auto_Error_Yaw[NOW]) < 20 )//接近目标
			{
				mobpre_yaw_stop_delay++;
				if(mobpre_yaw_stop_delay > 25)//停止稳定50ms
				{
					Mobi_Pre_Yaw_Fire = TRUE;//此时根据视觉开火标志位做射击判断，记得一定要置TRUE
				}
			}
			else
			{
				Mobi_Pre_Yaw_Fire = FALSE;//标记没回复到位，禁止开火
			}	
        }
    /*---------------pitch预测------------------*/  
        if(Auto_KF_Delay > debug_kf_delay 
        && fabs(Auto_Error_Pitch[NOW]) < debug_auto_err_p
        && fabs(pitch_kf_result[KF_SPEED]) > debug_kf_speed_p_low
        && (GIMBAL_AUTO_PITCH_Sen_SK() == FALSE || GIMBAL_AUTO_PITCH_Sen() == FALSE)
        && VisionRecvData.distance < 1000)		  
        {
            if(VisionRecvData.auto_too_close == TRUE)//目标距离太近，减小预测
			{
				pitch_speed_k = debug_p_sp_k/2.f;//预测系数减半
				kf_pitch_angcon = debug_kf_p_angcon/1.5f;
			}
            else//正常预测量
			{
				pitch_speed_k = debug_p_sp_k;
				kf_pitch_angcon = debug_kf_p_angcon;
			}
            if(pitch_kf_result[KF_SPEED]>=0)
			{
				debug_kf_p_angle = pitch_speed_k * (pitch_kf_result[KF_SPEED] - debug_kf_speed_p_low);
			}
			else
			{
				debug_kf_p_angle = pitch_speed_k * (pitch_kf_result[KF_SPEED] + debug_kf_speed_p_low);			
			}
			//pitch预测量限幅
			debug_kf_p_angle = constrain_float(debug_kf_p_angle, -kf_pitch_angcon, kf_pitch_angcon);
			
			Cloud_Angle_Target[PITCH][MECH] = pitch_kf_result[KF_ANGLE] + debug_kf_p_angle;

        }
        /*预测条件没达到，关闭预测*/
		else
		{
			Cloud_Angle_Target[PITCH][MECH] =pitch_kf_result[KF_ANGLE];
		}
        #else
            //预测全靠视觉
            Cloud_Angle_Target[YAW][GYRO] = yaw_kf_result[0];
            Cloud_Angle_Target[PITCH][MECH] = pitch_kf_result[0];
        #endif
    }
    else    //未识别到目标,可随意控制云台
    {
      yaw_kf_result = kalman_filter_calc(&yaw_kalman_filter, Cloud_Angle_Measure[YAW][GYRO], 0);
		pitch_kf_result = kalman_filter_calc(&pitch_kalman_filter, Cloud_Angle_Measure[PITCH][MECH], 0);
	
		if (modeGimbal == MECH)//机械模式
		{
			Cloud_Angle_Target[PITCH][MECH] += MOUSE_Y_MOVE_SPEED * kKey_Mech_Pitch;
			Cloud_Angle_Target[YAW][MECH]    = Mech_Mid_Yaw;	//yaw保持不动,永远在中间			
		}
		else if (modeGimbal == GYRO)
		{
			Mouse_Gyro_Yaw   += MOUSE_X_MOVE_SPEED * kKey_Gyro_Yaw;//记录目标变化角度
			Mouse_Gyro_Pitch += MOUSE_Y_MOVE_SPEED * kKey_Gyro_Pitch;//pitch仍旧使用机械模式
			if(MOUSE_X_MOVE_SPEED == 0)
			{
				Mouse_Yaw_Stop ++ ;
				if(Mouse_Yaw_Stop > 25)//鼠标长时间停留，停止移动
				{
					Mouse_Gyro_Yaw = 0;
				}
			}
			else
			{
				Mouse_Yaw_Stop = 0;
			}
			
			if(MOUSE_Y_MOVE_SPEED == 0)
			{
				Mouse_Pitch_Stop ++ ;
				if(Mouse_Pitch_Stop > 25)//鼠标长时间停留，停止移动
				{
					Mouse_Gyro_Pitch = 0;
				}
			}
			else
			{
				Mouse_Pitch_Stop = 0;
			}
			Cloud_Angle_Target[YAW][GYRO]   = RampInc_float( &Mouse_Gyro_Yaw, Cloud_Angle_Target[YAW][GYRO], Slope_Mouse_Yaw );
			Cloud_Angle_Target[PITCH][MECH] = RampInc_float( &Mouse_Gyro_Pitch, Cloud_Angle_Target[PITCH][MECH], Slope_Mouse_Pitch );
		}        
        
    }
    
}

/*------------------------PID-------------------------*/

/**
  * @brief  不同模式下PID值的切换
  * @param  void
  * @retval void
  * @attention 
  */
void GIMBAL_PID_Mode(void)
{
    //pitch
    Pitch_mech_pid.pid_position.p=Gimbal_PID_list[PITCH][MECH][actGimbal][OUTER][kp];
    Pitch_mech_pid.pid_position.i=Gimbal_PID_list[PITCH][MECH][actGimbal][OUTER][ki];
    Pitch_mech_pid.pid_position.d=Gimbal_PID_list[PITCH][MECH][actGimbal][OUTER][kd];
    Pitch_mech_pid.pid_speed.p=Gimbal_PID_list[PITCH][MECH][actGimbal][INNER][kp];
    Pitch_mech_pid.pid_speed.i=Gimbal_PID_list[PITCH][MECH][actGimbal][INNER][ki];
    Pitch_mech_pid.pid_speed.d=Gimbal_PID_list[PITCH][MECH][actGimbal][INNER][kd];
    //yaw
    if(modeGimbal==MECH)
    {
        Yaw_mech_pid.pid_position.p=Gimbal_PID_list[YAW][MECH][actGimbal][OUTER][kp];
        Yaw_mech_pid.pid_position.i=Gimbal_PID_list[YAW][MECH][actGimbal][OUTER][ki];
        Yaw_mech_pid.pid_position.d=Gimbal_PID_list[YAW][MECH][actGimbal][OUTER][kd];
        Yaw_mech_pid.pid_speed.p=Gimbal_PID_list[YAW][MECH][actGimbal][INNER][kp];
        Yaw_mech_pid.pid_speed.i=Gimbal_PID_list[YAW][MECH][actGimbal][INNER][ki];
        Yaw_mech_pid.pid_speed.d=Gimbal_PID_list[YAW][MECH][actGimbal][INNER][kd];
    }
    else if(modeGimbal==GYRO)
    {
        Yaw_gyro_pid.pid_position.p=Gimbal_PID_list[YAW][GYRO][actGimbal][OUTER][kp];
        Yaw_gyro_pid.pid_position.i=Gimbal_PID_list[YAW][GYRO][actGimbal][OUTER][ki];
        Yaw_gyro_pid.pid_position.d=Gimbal_PID_list[YAW][GYRO][actGimbal][OUTER][kd];
        Yaw_gyro_pid.pid_speed.p=Gimbal_PID_list[YAW][GYRO][actGimbal][INNER][kp];
        Yaw_gyro_pid.pid_speed.i=Gimbal_PID_list[YAW][GYRO][actGimbal][INNER][ki];
        Yaw_gyro_pid.pid_speed.d=Gimbal_PID_list[YAW][GYRO][actGimbal][INNER][kd];
    }

}
/**
  * @brief  PID计算
  * @param  void
  * @retval void
  * @attention 
  */
void GIMBAL_PID(void)
{
    static uint8_t yaw_last_mode;
    float pitch_position_error;
    float yaw_position_error[2];
    static float pitch_speed_error[2];    
    static float yaw_speed_error[2][2];
    
    static float  pTermPitch;     
    static float  pTermYaw[2];//   outer/inner    outer/inner//mech/gyro
    static float  iTermPitch;   
    static float  iTermYaw[2];
    static float  dTermPitch;      
    static float  dTermYaw[2];
    
    //pitch
	Cloud_Angle_Target[PITCH][MECH] = constrain_float( Cloud_Angle_Target[PITCH][MECH], Mech_Min_Pitch, Mech_Max_Pitch );
    pitch_position_error=Cloud_Angle_Target[YAW][MECH]-Cloud_Angle_Measure[YAW][MECH];
    	//误差进行卡尔曼滤波,消除低频低幅度抖动
	pitch_position_error = KalmanFilter(&Gimbal_Pitch_Mech_Error_Kalman, pitch_position_error);
    //外环
	pitch_speed_error[NOW]=Gimbal_PID_list[PITCH][MECH][actGimbal][OUTER][kp]*pitch_position_error*0.003f;
    
    pTermPitch=Gimbal_PID_list[PITCH][MECH][actGimbal][INNER][kp]*pitch_speed_error[NOW];
    iTermPitch+=Gimbal_PID_list[PITCH][MECH][actGimbal][INNER][ki]*pitch_speed_error[NOW];
    iTermPitch=constrain_float(iTermPitch,-PID_Iterm_Max,PID_Iterm_Max);
    dTermPitch=Gimbal_PID_list[PITCH][MECH][actGimbal][INNER][kd]*(pitch_speed_error[NOW]-pitch_speed_error[LAST]);
    //输出 
    pitch_out=pTermPitch+iTermPitch+dTermPitch;
    pitch_out=constrain_float(pitch_out,-PID_Out_Max,PID_Out_Max);
    pitch_speed_error[LAST]=pitch_speed_error[NOW];
    
    //yaw
    switch(modeGimbal)//机械模式
	{
        case MECH:
        if(yaw_last_mode==GYRO)
        {           
            yaw_position_error[GYRO]=0;
            yaw_speed_error[GYRO][NOW]=0;
            yaw_speed_error[GYRO][LAST]=0;
            pTermYaw[GYRO]=0;
            iTermYaw[GYRO]=0;
            dTermYaw[GYRO]=0;
        }
        yaw_position_error[MECH]=Cloud_Angle_Target[YAW][MECH]-Cloud_Angle_Measure[YAW][MECH];
        yaw_position_error[MECH] = KalmanFilter(&Gimbal_Yaw_Mech_Error_Kalman, yaw_position_error[MECH]);
        
        yaw_speed_error[MECH][NOW]=Gimbal_PID_list[YAW][MECH][actGimbal][OUTER][kp]*yaw_position_error[MECH]*0.003f;
        
        pTermYaw[MECH]=Gimbal_PID_list[YAW][MECH][actGimbal][INNER][kp]*yaw_speed_error[MECH][NOW];
        iTermYaw[MECH]+=Gimbal_PID_list[YAW][MECH][actGimbal][INNER][ki]*yaw_speed_error[MECH][NOW];
        iTermYaw[MECH]=constrain_float(iTermYaw[MECH],-PID_Iterm_Max,PID_Iterm_Max);
        dTermYaw[MECH]=Gimbal_PID_list[YAW][MECH][actGimbal][INNER][kd]*(yaw_speed_error[MECH][NOW]-yaw_speed_error[MECH][LAST]);
        
        yaw_out=pTermYaw[MECH]+iTermYaw[MECH]+dTermYaw[MECH];
        yaw_out=constrain_float(yaw_out,-PID_Out_Max,PID_Out_Max);
        
        yaw_speed_error[MECH][LAST]=yaw_speed_error[MECH][NOW];             
        yaw_last_mode=MECH;
        break;
        
        case GYRO:
        if(yaw_last_mode==MECH)
        {
                
            yaw_position_error[MECH]=0;
            yaw_speed_error[MECH][NOW]=0;
            yaw_speed_error[MECH][LAST]=0;
            pTermYaw[MECH]=0;
            iTermYaw[MECH]=0;
            dTermYaw[MECH]=0;
        }
        yaw_speed_error[GYRO][NOW]=Gimbal_PID_list[YAW][GYRO][actGimbal][OUTER][kp]*yaw_position_error[GYRO]*0.003f;
        
        pTermYaw[GYRO]=Gimbal_PID_list[YAW][GYRO][actGimbal][INNER][kp]*yaw_speed_error[GYRO][NOW];
        iTermYaw[GYRO]+=Gimbal_PID_list[YAW][GYRO][actGimbal][INNER][ki]*yaw_speed_error[GYRO][NOW];
        iTermYaw[GYRO]=constrain_float(iTermYaw[GYRO],-PID_Iterm_Max,PID_Iterm_Max);
        dTermYaw[GYRO]=Gimbal_PID_list[YAW][GYRO][actGimbal][INNER][kd]*(yaw_speed_error[GYRO][NOW]-yaw_speed_error[GYRO][LAST]);
        
        yaw_out=pTermYaw[GYRO]+iTermYaw[GYRO]+dTermYaw[GYRO];
        yaw_out=constrain_float(yaw_out,-PID_Out_Max,PID_Out_Max);
        
        yaw_speed_error[GYRO][LAST]=yaw_speed_error[GYRO][NOW];
        yaw_last_mode=GYRO;        
        break;
        
	}	
	
}
/**
  * @brief  模糊PID
  * @param  void
  * @retval void
  * @attention 
  */
void fuzzy_PID(void)
{

}

/**
  * @brief  PID控制输出
  * @param  void
  * @retval void
  * @attention 
  */

void task_PID_out(void)
{
    GIMBAL_PID_Mode();//根据操作模式变换kpid,每次都要变		
    GIMBAL_PID();//PID计算
    Gimbal_set_voltage(yaw_out,pitch_out);//CAN输出
    yaw_out=0;
    pitch_out=0;
}

void sent_gimbal_to_down(int16_t v)
{
	CAN_TxHeaderTypeDef tx_header;
	uint8_t             tx_data[8];
	
	tx_header.StdId = GIMBAL_DATA;
	tx_header.IDE   = CAN_ID_STD;
	tx_header.RTR   = CAN_RTR_DATA;
	tx_header.DLC   = 2;

	tx_data[0] = (v>>8)&0xff;
	tx_data[1] =    (v)&0xff;
//	tx_data[2] = (v>>8)&0xff;
//	tx_data[3] =    (v)&0xff;
	
	HAL_CAN_AddTxMessage(&hcan2, &tx_header, tx_data,(uint32_t*)CAN_TX_MAILBOX0);

}
/****************************辅助函数***********************************************/
/**
  * @brief  限制云台与底盘分离角度
  * @param  void
  * @retval void
  * @attention 
  */
void Gimbal_Chass_Separ_Limit(void)
{
	if ( (GIMBAL_GetOffsetAngle() <= -CLOUD_SEPAR_ANGLE				//右过大
			&& (rc_t.mouse.x>0||rc_t.rc.ch1>0))	
				|| ( GIMBAL_GetOffsetAngle() >= CLOUD_SEPAR_ANGLE	//左过大
					&& ( rc_t.mouse.x<0 || rc_t.rc.ch1<0) ) )
	{
//		RC_Ctl.mouse.x = 0;
//		//调试的时候记得注释，否则会出现ch0有半边无数据的情况
//		RC_Ctl.rc.ch0  = RC_CH_VALUE_OFFSET;
		
		kRc_Gyro_Yaw   -= krc_gyro_yaw/300;//0.005;
		kKey_Gyro_Yaw   -= -krc_gyro_yaw/300;//-0.15;
		
		if(ABS(kRc_Gyro_Yaw) < ABS(krc_gyro_yaw/290)
			|| ABS(kKey_Gyro_Yaw) < ABS(krc_gyro_yaw/290))
		{
			kRc_Gyro_Yaw = 0;
			kKey_Gyro_Yaw = 0;
		}
	}
	else
	{
		kRc_Gyro_Yaw   = krc_gyro_yaw;//0.015;
		kKey_Gyro_Yaw   = -kkey_gyro_yaw;//-0.38;
	}
	
	#if YAW_POSITION == YAW_UP
		if ( (GIMBAL_GetOffsetAngle() <= (Mech_Left_Yaw - Mech_Mid_Yaw + 50)	&& RC_Ctl.mouse.x>0)//右过大	
					|| ( GIMBAL_GetOffsetAngle() >= (Mech_Right_Yaw - Mech_Mid_Yaw - 50) && RC_Ctl.mouse.x<0 )	//左过大
		   )
		{
			kKey_Gyro_Yaw = 0;
		}
		else
		{
			kKey_Gyro_Yaw   = -kkey_gyro_yaw;
		}
	#else
		if ( (GIMBAL_GetOffsetAngle() <= (Mech_Right_Yaw - Mech_Mid_Yaw +500) && rc_t.mouse.x>0)//右过大	
					|| ( GIMBAL_GetOffsetAngle() >= (Mech_Left_Yaw - Mech_Mid_Yaw - 500) && rc_t.mouse.x<0 )//左过大	
		   )
		{
			kKey_Gyro_Yaw = 0;
		}
		else
		{
			kKey_Gyro_Yaw   = -kkey_gyro_yaw;
		}
	#endif
}



/**
  * @brief  等待Pitch轴水平
  * @param  void
  * @retval 1回到水平位置,0未回到水平位置
  * @attention 角度小于50则认为回到水平
  */
uint8_t GIMBAL_IfPitchLevel(void)
{	  
	if ( ABS( Cloud_Angle_Measure[PITCH][MECH] - Mech_Mid_Pitch ) <= 50 )
	{
        return 1;
	}
		
	return 0;
}

/**
  * @brief  计算YAW偏离中心角度,底盘跟随模式用
  * @param  void
  * @retval sAngleError,偏离角度值,CAN反馈的机械角度
  */
int16_t GIMBAL_GetOffsetAngle(void)
{
	int16_t sAngleError;

	sAngleError = (Cloud_Angle_Measure[YAW][MECH] - Mech_Mid_Yaw)*YAW_POSITION;


	//过零处理,统一成劣弧
	if (sAngleError > 4096)
	{
		return (sAngleError - 8191) ;
	}
	else if (sAngleError < -4096)
	{
		return (sAngleError + 8191);
	}
	else
	{
		return  sAngleError;
	}
} 


/**
  * @brief  计算目标速度
  * @param  void
  * @retval 
  */
float Target_Speed_Calc(speed_calc_data_t *S, uint32_t time, float position)
{
	S->delay_cnt++;

	if (time != S->last_time)
	{
		S->speed = (position - S->last_position) / (time - S->last_time) * 2;//计算速度
           

		S->last_time = time;
		S->last_position = position;
		S->last_speed = S->speed;
		S->delay_cnt = 0;
	}

	if(S->delay_cnt > 300/*100*/) // delay 200ms speed = 0
	{
		S->processed_speed = 0;//时间过长则认为速度不变
	}

	return S->processed_speed;//计算出的速度
}

/**
  * @brief  是否在自瞄哨兵
  * @param  void
  * @retval TRUE   FALSE
  * @attention 自瞄抬头角度太大则认为在打哨兵
  */
bool GIMBAL_AUTO_PITCH_Sen(void)
{
	if(Mech_Max_Pitch-Cloud_Angle_Measure[PITCH][MECH] <= down_sb_pitch/*300*/ 
			|| IF_KEY_PRESSED_G)//抬头接近限位
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}

/**
  * @brief  是否在中等距离自瞄哨兵,加大预测
  * @param  void
  * @retval TRUE   FALSE
  * @attention 自瞄抬头角度太大则认为在打哨兵
  */
float pitch_sb_error = 0;
bool GIMBAL_AUTO_PITCH_Sen_SK(void)
{
	pitch_sb_error = Mech_Max_Pitch-Cloud_Angle_Measure[PITCH][MECH];
	if((pitch_sb_error<= down_sb_pitch/*450*//*550*/)
    && (pitch_sb_error > up_sb_pitch) )//抬头接近限位
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}


/*--------------调用其他任务中的函数，提前写这解决报错，后续会删除------------*/

bool GIMBAL_AUTO_PITCH_Sentry(void)
{return 0;}
    
