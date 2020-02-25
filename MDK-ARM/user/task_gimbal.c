#include "task_gimbal.h"


/*************��̨ϵͳ************/
eGimbalCtrlMode  modeGimbal;
eGimbalAction actGimbal;
#if		INFANTRY_DEBUG_ID == 3

/*---------------------��̨��ر���------------------------*/ 
	#define Mech_Min_Pitch     126   //down              
	#define Mech_Mid_Pitch     704                     
	#define Mech_Max_Pitch     1496    //up           
	#define Mech_Right_Yaw     5974     //right max         
	#define Mech_Mid_Yaw       6474                     
	#define Mech_Left_Yaw      6974     //left max        

	//��̨̧ͷ�Ƕ�,����Ħ���ֿ���
	#define CLOUD_FRIC_PIT_UP  (Mech_Mid_Pitch + 200)   


	//��̨���̷���Ƕ�,����yaw��λ
	#define CLOUD_SEPAR_ANGLE  800//С�Ļ��Ťͷģʽ��ͻ//����

	
	//����Ƕ�
	//float base_mech_pitch = 3960;                       //����

    
float down_sb_pitch = 530;//����
float up_sb_pitch   = 0;//����
 
#endif
/*************������************/

//��еģʽ�±���ϵ��,����ҡ����Ӧ�ٶ�
float kRc_Mech_Pitch, kRc_Mech_Yaw;

//������ģʽ�±���ϵ��,����ҡ����Ӧ�ٶ�
float kRc_Gyro_Pitch, kRc_Gyro_Yaw;
float krc_gyro_yaw = 0.018;

//��еģʽ�±���ϵ��,���Ƽ�����Ӧ�ٶ�
float kKey_Mech_Pitch, kKey_Mech_Yaw;

//������ģʽ�±���ϵ��,���Ƽ�����Ӧ�ٶ�
float kKey_Gyro_Pitch, kKey_Gyro_Yaw;
float kkey_gyro_yaw = 0.38;

float angleMpuPitch,	angleMpuYaw,	angleMpuRoll;//�����ǽǶ�ֵ
short palstanceMpuPitch,	palstanceMpuYaw,	palstanceMpuRoll;//�����ǽ��ٶ�ֵ


//�����Ƕ�
float Cloud_Angle_Target[2][2];//  pitch/yaw    mech/gyro

//�����Ƕ�
float Cloud_Angle_Measure[2][2];//  pitch/yaw    mech/gyro

//�������ٶ�
float Cloud_Palstance_Measure[2][2];//  pitch/yaw    mech/gyro

/*******************PID����**********************/
Cascade_pid_t Pitch_mech_pid;
Cascade_pid_t Yaw_mech_pid;
Cascade_pid_t Yaw_gyro_pid;
float pitch_out;
float yaw_out;
float Gimbal_PID_list[2][2][3][2][3];        //pitch/yaw  mech/gyro  ģʽ   OUTER/INNER   KP/KI/KD
/**************�޷�****************/
//������̨���������������


/**************б��***************/
float Slope_Mouse_Pitch, Slope_Mouse_Yaw;//Ħ���ֿ���ʱ̧ͷ����
	
//�ϵ�б�±���
float Slope_Begin_Pitch, Slope_Begin_Yaw;//���ϵ�ʱ�ƶ�����

//����������ģʽ�����ͳ��yawƫ����,��ֵ���Լ�������С,��ֹ˦ͷ����
float Mouse_Gyro_Yaw, Mouse_Gyro_Pitch;

//����������ģʽ��QECŤͷ����
float Slope_Turn_Yaw;
float Slope_Back_Yaw;

//����Ħ����̧ͷб��
float Slope_Fric_Pitch;

/***************����******************/
uint8_t if_yaw_auto_pre;
//���
float Auto_Error_Yaw[2];//    now/last
float Auto_Error_Pitch[2];
float Auto_Distance;//���뵥Ŀ

//����б��
float Slope_Auto_Yaw;
float Slope_Auto_Pitch;

//����ͻȻ����,�������˲�������ʱ
uint16_t Auto_KF_Delay = 0;

float   debug_y_sp_k;// = 38;//35;//30;//�ƶ�Ԥ��ϵ��,Խ��Ԥ��Խ��
float   debug_y_sen_sp_k;//�ڱ�Ԥ��ϵ��
float   debug_y_sen_brige_sp_k;//��ͷ�ڱ�
float   debug_p_sp_k;//�ƶ�Ԥ��ϵ��,Խ��Ԥ��Խ��
float   debug_auto_err_y;// = 10;//15;//10;//15;//yaw�Ƕȹ���ر�Ԥ��
float   debug_auto_err_p;//pitch�Ƕȹ���ر�Ԥ��
int     debug_kf_delay;// = 150;//100;//200;//120;//150;//Ԥ����ʱ����
float   debug_kf_speed_y_low;//yaw�ٶȹ��͹ر�Ԥ��
float   debug_kf_speed_y_low_sen;//̧ͷ���ڱ�ʱ��С��Ϳɿ�Ԥ����
float   debug_kf_speed_y_high;//yaw�ٶȹ��߹ر�Ԥ��
float   debug_kf_speed_p_low;//pitch�ٶȹ��͹ر�Ԥ��
float   debug_kf_y_angcon;// = 130;//125;//115;//135;//yawԤ�����޷�
float   debug_kf_p_angcon;//pitchԤ�����޷�

float   Vision_Angle_Speed_Yaw, Vision_Angle_Speed_Pitch;//�������˲��ٶȲ���ֵ
float   *yaw_kf_result, *pitch_kf_result;//���׿������˲����,0�Ƕ� 1�ٶ�
/*************�������˲�**************/
/*һ�׿�����*/
//��̨�Ƕ�������
extKalman_t Gimbal_Pitch_Mech_Error_Kalman;//����һ��kalmanָ��

extKalman_t Gimbal_Yaw_Mech_Error_Kalman;//����һ��kalmanָ��
extKalman_t Gimbal_Yaw_Gyro_Error_Kalman;//����һ��kalmanָ��

extKalman_t Vision_Distance_Kalman;

speed_calc_data_t Vision_Yaw_speed_Struct;
speed_calc_data_t Vision_Pitch_speed_Struct;

kalman_filter_init_t yaw_kalman_filter_para = {
  .P_data = {2, 0, 0, 2},
  .A_data = {1, 0.002/*0.001*/, 0, 1},//����ʱ����
  .H_data = {1, 0, 0, 1},
  .Q_data = {1, 0, 0, 1},
  .R_data = {200, 0, 0, 400}//500 1000
};//��ʼ��yaw�Ĳ���kalman����

kalman_filter_init_t pitch_kalman_filter_para = {
  .P_data = {2, 0, 0, 2},
  .A_data = {1, 0.002/*0.001*/, 0, 1},//����ʱ����
  .H_data = {1, 0, 0, 1},
  .Q_data = {1, 0, 0, 1},
  .R_data = {200, 0, 0, 400}
};//��ʼ��pitch�Ĳ���kalman����

kalman_filter_t yaw_kalman_filter;
kalman_filter_t pitch_kalman_filter;

/*�Զ����õ�һЩ��־λ*/
bool Mobi_Prediction_Yaw = FALSE;//Ԥ���Ƿ�����־λ
bool Mobi_Pre_Yaw_Fire = FALSE;//Ĭ��Ԥ��û��λ����ֹ��ǹ

uint16_t mobpre_yaw_left_delay = 0;//����Ԥ����ʱ�жϿɿ�������
uint16_t mobpre_yaw_right_delay = 0;//����Ԥ����ʱ�жϿɿ�������
uint16_t mobpre_yaw_stop_delay = 0;//Ԥ��ر���ʱ�жϿɿ�������
/****************��̨����ģʽ�¸�С������������********************/
//��ͷģʽ�Ƕ�Ŀ��
float TURNMode_Yaw_Back_Total;//����C,yaw��Ҫ�ı�ĽǶ�ֵ
float TURNMode_Yaw_Turn_Total;//����QE,yaw��Ҫ�ı�ĽǶ�ֵ,������������ת


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
  * @brief  ��̨������ʼ��
  * @param  void
  * @retval void
  * @attention ֻ��ϵͳ����ʱ����һ��
  */
void GIMBAL_InitArgument(void)
{
    modeGimbal = MECH;//Ĭ���Ի�еģʽ����
    
    /* ������,��Ӧ���� */	
	kRc_Mech_Yaw   = -0.01;
	kRc_Mech_Pitch = 0.01;	
	kRc_Gyro_Yaw   = -0.15;//����������ģʽ��תͷ�ٶȿ�����Ӱ��������ģʽ�³�Ťͷ�ٶ�

	
	kKey_Mech_Yaw   = 0.5;
	kKey_Mech_Pitch = 0.45;	
	kKey_Gyro_Yaw   = -0.38;//ע������,����ᷴ��

	
	/* б��,�仯���� */
	Slope_Begin_Pitch =  1;//�ϵ�ʱ�ƶ�����
  	Slope_Begin_Yaw   =  1;//�ϵ�ʱ�ƶ�����
	
	Slope_Mouse_Pitch = 15;//20;//�����Ӧ,̧ͷ��ͷ�ٶ�
	Slope_Mouse_Yaw   = 15;//�����Ӧ,Ťͷ�ٶ�
	
	Slope_Turn_Yaw = 20;//25;//QEŤͷ����
	Slope_Back_Yaw = 20;//30;//C��ͷ����
	
	Slope_Auto_Yaw   = 4;//1;//50;
	Slope_Auto_Pitch = 3;//1;//50;
	
	Slope_Fric_Pitch = 8;
    
    //��ͨ
	Gimbal_PID_list[PITCH][MECH][GIMBAL_NORMAL][OUTER][kp]=90;//90;
	Gimbal_PID_list[YAW][MECH][GIMBAL_NORMAL][OUTER][kp]=90;
			
				//inner
	Gimbal_PID_list[PITCH][MECH][GIMBAL_NORMAL][INNER][kp]=250;//144;
	Gimbal_PID_list[PITCH][MECH][GIMBAL_NORMAL][INNER][ki]=1;//1;
	Gimbal_PID_list[PITCH][MECH][GIMBAL_NORMAL][INNER][kd]=0;
			
	Gimbal_PID_list[YAW][MECH][GIMBAL_NORMAL][INNER][kp]=180;
	Gimbal_PID_list[YAW][MECH][GIMBAL_NORMAL][INNER][ki]=1;
	Gimbal_PID_list[YAW][MECH][GIMBAL_NORMAL][INNER][kd]=0;			
			/* kPID,������ģʽ */
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
			
			/* kPID,������ģʽ */
				//outer
	Gimbal_PID_list[YAW][GYRO][GIMBAL_AUTO][OUTER][kp]=0;
			
				//inner		
	Gimbal_PID_list[YAW][GYRO][GIMBAL_AUTO][INNER][kp]=0;
	Gimbal_PID_list[YAW][GYRO][GIMBAL_AUTO][INNER][ki]=0;
	Gimbal_PID_list[YAW][GYRO][GIMBAL_AUTO][INNER][kd]=0;	
    

    
    //�������˲�����ʼ��
	/*PID�Ƕ�������,һ��*/
	KalmanCreate(&Gimbal_Pitch_Mech_Error_Kalman, 1, 40);

	
	KalmanCreate(&Gimbal_Yaw_Mech_Error_Kalman, 1, 40);
	KalmanCreate(&Gimbal_Yaw_Gyro_Error_Kalman, 1, 40);
	
	KalmanCreate(&Vision_Distance_Kalman, 1, 2000);
    
    
    
    
}




/**
  * @brief  ��̨ʧ�ر���
  * @param  void
  * @retval void
  * @attention ���������0
  */
void GIMBAL_StopMotor(void)
{
    
}

/**
  * @brief  ��̨��ʼ��
  * @param  void
  * @retval void
  * @attention 
  */
int GIMBAL_InitCtrl(void)
{    
    static bool bTick_Record  = FALSE;
    static bool bAngle_Record  = FALSE;
    Gimbal_value();	//��ֵ
    if(bTick_Record == FALSE)
    {   
        if (system_tick > 5)//��֤���ϵ�������´ο���        
		  bTick_Record = TRUE;
    }    
    else
    {
        //��¼�ϵ�ʱ��̨��е�Ƕ�
        if (bAngle_Record == FALSE)
        {
            bAngle_Record = TRUE;
			
            Cloud_Angle_Target[PITCH][MECH] = GM6020[PITCH].rotor_angle;
            Cloud_Angle_Target[YAW][MECH] = GM6020[YAW].rotor_angle;
        }
	
                    //��¼�����ǳ�ʼ�Ƕ�
        else
        {

            //ƽ��������̨�ƶ����м�,��ֹ���ϵ��˦
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
  * @brief  ң�ؿ�����̨ģʽ
  * @param  void
  * @retval void
  * @attention �ڴ˸ı�ǶȻ�Ŀ��ֵ
  */
void GIMBAL_Rc_Ctrl(void)
{
    	if(modeGimbal == GYRO)
	{
		//������̨����̷���Ƕ�
		Gimbal_Chass_Separ_Limit();
	}
	
	if (IF_RC_SW2_DOWN)//s2������,������ģʽ
	{
		modeGimbal = GYRO;
	}
	else if (IF_RC_SW2_MID)//S2��
	{
		modeGimbal = MECH;//S2��,��еģʽ
	}
	
	/*-----ң�ؿ��Ƶ��ֿ���------*/
	if (Magazine_opened_flag)
	{
		Cloud_Angle_Target[PITCH][MECH] = Mech_Mid_Pitch;

		if (modeGimbal == MECH)//��еģʽ��yaw����
		{
			Cloud_Angle_Target[YAW][MECH] = Mech_Mid_Yaw;
		}
		else if (modeGimbal == GYRO)//������ģʽ��ҡ�˿���
		{
			Cloud_Angle_Target[YAW][GYRO] += -rc_t.rc.ch1*kRc_Gyro_Yaw;
		}

	}
	else    
	{
		//��������pitch
		if (modeGimbal == MECH)
		{
			Cloud_Angle_Target[PITCH][MECH] += rc_t.rc.ch2*kRc_Mech_Pitch; 
			Cloud_Angle_Target[YAW][MECH]   = rc_t.rc.ch1*kRc_Mech_Yaw; //��еģʽ,yaw�̶�����
			
			Cloud_Angle_Target[YAW][GYRO] = Cloud_Angle_Measure[YAW][GYRO];
		}
		else if (modeGimbal == GYRO)
		{
			Cloud_Angle_Target[PITCH][MECH] += -rc_t.rc.ch2*kRc_Mech_Pitch;//pitch���û�е����ʽ  
			Cloud_Angle_Target[YAW][GYRO]   += -rc_t.rc.ch1*kRc_Gyro_Yaw; 
		}
	}	
}


/**
  * @brief  ���̿�����̨ģʽ
  * @param  void
  * @retval void
  * @attention 
  */              
void GIMBAL_Key_Ctrl(void)
{
    if(modeGimbal == GYRO)
	{
		//������̨����̷���Ƕ�
		Gimbal_Chass_Separ_Limit();
	}
	
	switch(actGimbal)
	{
		/*--------------��̨ģʽѡ��----------------*/
		case GIMBAL_NORMAL:
			GIMBAL_NORMAL_Mode_Ctrl();//�ڴ�ѡ�����ģʽ
		break;
		
		/*--------------V  180���ͷ----------------*/
		case GIMBAL_AROUND:
			modeGimbal = GYRO;//����������ģʽ
		
			if (TURNMode_Yaw_Back_Total == 0)
			{
				actGimbal = GIMBAL_NORMAL;
			}
			else
			{
				Cloud_Angle_Target[YAW][GYRO] = RampInc_float( &TURNMode_Yaw_Back_Total, Cloud_Angle_Target[YAW][GYRO], Slope_Back_Yaw );
			}
		break;
		
		/*------------���ֿ���,��ֹ̧ͷ-----------------*/
		case GIMBAL_LEVEL:
			GIMBAL_LEVEL_Mode_Ctrl();
		break;
		
		/*--------------Q E  90���ͷ----------------*/
		case GIMBAL_TURN:				
			modeGimbal = GYRO;//����������ģʽ

		    if (TURNMode_Yaw_Turn_Total == 0)
			{
				actGimbal = GIMBAL_NORMAL;
			}
			else
			{
				Cloud_Angle_Target[YAW][GYRO] = RampInc_float( &TURNMode_Yaw_Turn_Total, Cloud_Angle_Target[YAW][GYRO], Slope_Back_Yaw );
			}
		break;
			
		/*--------------�Ҽ�����----------------*/	
		case GIMBAL_AUTO:
			modeGimbal = GYRO;//����������ģʽ
		
		if(!IF_MOUSE_PRESSED_RIGH)//�ɿ��Ҽ��˳�����
		{
				actGimbal = GIMBAL_NORMAL;
				
//				//����Ŀ��ƫ������,�����л�ʱ��̨����
//				VisionRecvData.identify_target = FALSE;
//				Auto_KF_Delay = 0;//������´��ӳ�Ԥ����
//				Mobility_Prediction_Yaw = FALSE;//���Ԥ��û����
//				Mobi_Pre_Yaw_Fire = FALSE;//Ĭ�ϱ��Ԥ��û��λ����ֹ����
//				
//				mobpre_yaw_left_delay  = 0;//������Ԥ��Ŀ����ӳ�
//				mobpre_yaw_right_delay = 0;//������Ԥ��Ŀ����ӳ�	
//				mobpre_yaw_stop_delay = 0;//ֹͣԤ�⿪����ʱ����
		}
		else
		{
			GIMBAL_AUTO_Mode_Ctrl();//������ƺ���
		}
		break;
    }
}



/*******************��̨����ģʽ����ģʽС����*******************/

/**
  * @brief  ��̨����ģʽѡ��,������Ӧ
  * @param  void
  * @retval void
  * @attention ��̨���̿���״̬�µ�����ģʽ�л�������
  * ��ģʽ�л�ʱһֱ���ڴ�ģʽ
  */
void GIMBAL_NORMAL_Mode_Ctrl(void)
{
	//������ʱ��Ӧ,��ֹ�ּ���
	static uint32_t  Key_Ctrl_CurrentTime = 0;
	static uint32_t PressV_Time  = 0;//��ͷ,500ms��ʱ��Ӧ,1����ఴ2��
	static uint32_t PressQ_Time  = 0;//90��,250ms��ʱ��Ӧ,1����ఴ4��
    static uint32_t PressE_Time  = 0;//90��,250ms��ʱ��Ӧ,1����ఴ4��
//	static uint32_t PressCF_Time  = 0;//����,400ms��ʱ��Ӧ
//	static uint32_t PressCG_Time  = 0;//�ֶ����,400ms��ʱ��Ӧ
//	static uint32_t PressCV_Time  = 0;//��С��,400ms��ʱ��Ӧ
	static uint32_t Mouse_Yaw_Stop  = 0;//��겻����������Ӧ
	static uint32_t Mouse_Pitch_Stop  = 0;//��겻����������Ӧ
	
	Key_Ctrl_CurrentTime = system_tick;//��ȡʵʱʱ��,������������ʱ�ж�	
	
	
//	if ( CHASSIS_IfActiveMode() == TRUE || Magazine_IfOpen() ==	 TRUE)//��ȡ����ģʽ,trueΪ��еģʽ
//	{
//		modeGimbal = MECH;
//	} 
//	else					//ע�͵�loop�еĵ��̻���������ģʽʧЧ
//	{
//		modeGimbal = GYRO;
//	}

	
	
	if ( !IF_KEY_PRESSED_CTRL && IF_KEY_PRESSED_V
					&& Key_Ctrl_CurrentTime > PressV_Time)
	{   //Ctrl�����ڰ���״̬ʱ��V��ͷ
		actGimbal  =  GIMBAL_AROUND;//�л��ɵ�ͷģʽ

		PressV_Time = Key_Ctrl_CurrentTime + 50;//500ms��ʱ���ּ���

		if(IF_KEY_PRESSED_A)//AV���ͷ
		{
			TURNMode_Yaw_Back_Total = 3579;
		}
		else if(IF_KEY_PRESSED_D)//DV�ҵ�ͷ
		{
			TURNMode_Yaw_Back_Total = -3579;
		}
		else//Ĭ���ҵ�ͷ
		{
			TURNMode_Yaw_Back_Total = -3579;//��Ϊ�ǶȷŴ���20��,������180��*20Լ����3579
		}
	}
	/*---------------------------------*/	
	else if ( !IF_KEY_PRESSED_CTRL
				&& ( (IF_KEY_PRESSED_Q && Key_Ctrl_CurrentTime > PressQ_Time)
					|| (IF_KEY_PRESSED_E && Key_Ctrl_CurrentTime > PressE_Time) ) )
	{   //Ctrl�����ڰ���״̬ʱ��Q(��),E(��)90���ͷ
		actGimbal = GIMBAL_TURN;//�л��ɿ���Ťͷģʽ
		
		//ע�ⷽ��
		if ( IF_KEY_PRESSED_Q)
		{
			PressQ_Time = Key_Ctrl_CurrentTime + 25;//250ms��ʱ���ּ���
			
			TURNMode_Yaw_Turn_Total = 1789;//Q��תԼ8192/4��
		}
		else if (IF_KEY_PRESSED_E)
		{
			PressE_Time = Key_Ctrl_CurrentTime + 25;//250ms��ʱ���ּ���
			
			TURNMode_Yaw_Turn_Total = -1789;//E��תԼ8192/4��
		}
			
	}	
	/*---------------------------------*/
	else if (Magazine_opened_flag)
	{
		actGimbal = GIMBAL_LEVEL;

	}
	/*---------------------------------*/
	else if (IF_MOUSE_PRESSED_RIGH)//��SW1������,���Ҽ�����
	{
		actGimbal = GIMBAL_AUTO;

	}
//	/*----------------С��-----------------*/
//	else if(IF_KEY_PRESSED_V && IF_KEY_PRESSED_CTRL && Key_Ctrl_CurrentTime > PressCV_Time)//Ctrl+F���,400ms��Ӧһ��
//	{
//		PressCV_Time = Key_Ctrl_CurrentTime + TIME_STAMP_400MS;
//		actGimbal = GIMBAL_SM_BUFF;
//	}
//	/*----------------���-----------------*/
//	else if(IF_KEY_PRESSED_F && IF_KEY_PRESSED_CTRL && Key_Ctrl_CurrentTime > PressCF_Time)//Ctrl+F���,400ms��Ӧһ��
//	{
//		PressCF_Time = Key_Ctrl_CurrentTime + TIME_STAMP_400MS;
//		actGimbal = GIMBAL_BUFF;
//	}
//	/*----------------����-----------------*/
//	else if(IF_KEY_PRESSED_C && !IF_MOUSE_PRESSED_RIGH && !IF_RC_SW1_MID)
//	{
//		actGimbal = GIMBAL_BASE;
//	}
	/*---------------------------------*/
//	else if(IF_KEY_PRESSED_G && IF_KEY_PRESSED_CTRL && Key_Ctrl_CurrentTime > PressCG_Time)//Ctrl+G�ֶ����,400ms��Ӧһ��
//	{
//		PressCG_Time = Key_Ctrl_CurrentTime + TIME_STAMP_400MS;
//		actGimbal = GIMBAL_MANUAL;
//	}
	/*---------------------------------*/
	else       //�������̨�Ƕȼ���,������ͨģʽ�µĽǶȼ���,���ȼ����,���Է������
	{
		if (modeGimbal == MECH)//��еģʽ
		{
			Cloud_Angle_Target[PITCH][MECH] += MOUSE_Y_MOVE_SPEED * kKey_Mech_Pitch;
			Cloud_Angle_Target[YAW][MECH]   = Mech_Mid_Yaw;	//yaw���ֲ���	//yaw���ֲ���,��Զ���м�
			
			Cloud_Angle_Target[YAW][GYRO] = Cloud_Angle_Measure[YAW][GYRO];
			
		}
		else if (modeGimbal == GYRO)
		{
			Mouse_Gyro_Yaw   += MOUSE_X_MOVE_SPEED * kKey_Gyro_Yaw;//��¼Ŀ��仯�Ƕ�
			Mouse_Gyro_Pitch += MOUSE_Y_MOVE_SPEED * kKey_Gyro_Pitch;//pitch�Ծ�ʹ�û�еģʽ

			if(MOUSE_X_MOVE_SPEED == 0)
			{
				Mouse_Yaw_Stop ++ ;
				if(Mouse_Yaw_Stop > 25)//��곤ʱ��ͣ����ֹͣ�ƶ�
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
				if(Mouse_Pitch_Stop > 25)//��곤ʱ��ͣ����ֹͣ�ƶ�
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
  * @brief  ����ģʽ
  * @param  void
  * @retval void
  * @attention ��ģʽ�½�ֹ����pitch
  */
void GIMBAL_LEVEL_Mode_Ctrl(void)
{
	modeGimbal = MECH;//����ʱ�����еģʽ
	
	//�������,�˳�����ģʽ
	if(!Magazine_opened_flag)
	{
		actGimbal = GIMBAL_NORMAL;
	}
	else//����δ���,�Ƕȹ̶����м�
	{
		Cloud_Angle_Target[YAW][MECH]   = Mech_Mid_Yaw;
		Cloud_Angle_Target[PITCH][MECH] = Mech_Mid_Pitch;
	}
}

/**
  * @brief  ������ƺ���
  * @param  void
  * @retval void
  * @attention �м�����(0,0),�����Ҹ�,�ϸ�����
  *             yawΪ������ģʽ,pitchΪ��еģʽ(��ʵpitchȫ�̶����û�еģʽ)
  *            ֻ�е����������˲��ڵ�ǰʵʱ�Ƕ����ۼ�����Ŀ��Ƕ�
  *            ������һ��,Ŀ��ǶȾ�ʵʱ����
  */
uint32_t Vision_Time[2];// NOW/LAST

int vision_time_delta;//�Ӿ��ӳ�ms
//���ݾ������Ԥ��������޷�
float yaw_speed_k = 0;
float kf_yaw_angcon = 0;

float pitch_speed_k = 0;
float kf_pitch_angcon = 0;

int kf_delay_open = 0;
float kf_speed_yl = 0;//
int vision_time_delta;
void GIMBAL_AUTO_Mode_Ctrl(void)
{        
    float debug_kf_angle_temp;//Ԥ��Ƕ�б���ݴ���
    float debug_kf_angle_ramp = 20;//Ԥ��Ƕ�б�±仯��    
    float debug_kf_y_angle;//yawԤ���ݴ�
    float debug_kf_p_angle;//pitchԤ���ݴ�
    static uint32_t Mouse_Yaw_Stop  = 0;//��겻����������Ӧ
	static uint32_t Mouse_Pitch_Stop  = 0;//��겻����������Ӧ

    static float yaw_angle_raw, pitch_angle_raw;//�������˲��ǶȲ���ֵ
    static float yaw_angle_ref;//��¼Ŀ��Ƕ�
	static float pitch_angle_ref;//��¼Ŀ��Ƕ�

    /*************************����***********************/
    Auto_Error_Yaw[NOW]=VisionRecvData.yaw_angle;
    Auto_Error_Pitch[NOW]=VisionRecvData.pitch_angle;
    Auto_Distance=VisionRecvData.distance;
    if(Auto_Distance<0)
        Auto_Distance=0;
    Auto_Distance = KalmanFilter(&Vision_Distance_Kalman, Auto_Distance);

    /***********************���ݸ���*********************/
    if(Vision_Data_Flag)
    {
        //����Ŀ��Ƕ�//��¼��ǰʱ�̵�Ŀ��λ��,Ϊ��������׼��
		yaw_angle_ref   = Cloud_Angle_Measure[YAW][GYRO]   + Auto_Error_Yaw[NOW]   ;
		pitch_angle_ref = Cloud_Angle_Measure[PITCH][MECH] + Auto_Error_Pitch[NOW];
		
        Vision_Data_Flag=0;//����
        Vision_Time[NOW] = system_time;//��ȡ�����ݵ���ʱ��ʱ��
    }
       
    if(Vision_Time[NOW] != Vision_Time[LAST])//���������ݵ�����ʱ��
	{
		vision_time_delta = Vision_Time[NOW] - Vision_Time[LAST];//�����Ӿ��ӳ�
		yaw_angle_raw  = yaw_angle_ref;//���¶��׿������˲�����ֵ
		pitch_angle_raw = pitch_angle_ref;
		Vision_Time[LAST] = Vision_Time[NOW];
	}
    
    /*************************�˲�***********************/
    	//Ŀ���ٶȽ���
	if(VisionRecvData.identify_target == TRUE)//ʶ����Ŀ��
	{
		Vision_Angle_Speed_Yaw = Target_Speed_Calc(&Vision_Yaw_speed_Struct, Vision_Time[NOW], yaw_angle_raw);
		Vision_Angle_Speed_Pitch = Target_Speed_Calc(&Vision_Pitch_speed_Struct, Vision_Time[NOW], pitch_angle_raw);
		//�ԽǶȺ��ٶȽ��ж��׿������˲��ں�,0λ��,1�ٶ�
		yaw_kf_result = kalman_filter_calc(&yaw_kalman_filter, yaw_angle_raw, Vision_Angle_Speed_Yaw);
		pitch_kf_result = kalman_filter_calc(&pitch_kalman_filter, pitch_angle_raw, Vision_Angle_Speed_Pitch);

	}
    //��˵��������Ŀ���л������鿪���رյĶ������д���֤
//	else
//	{
////		//�ԽǶȺ��ٶȽ��ж��׿������˲��ں�,0λ��,1�ٶ�
//		Vision_Angle_Speed_Yaw = Target_Speed_Calc(&Vision_Yaw_speed_Struct, system_time, Cloud_Angle_Measure[YAW][GYRO]);
//		Vision_Angle_Speed_Pitch = Target_Speed_Calc(&Vision_Pitch_speed_Struct, system_time, Cloud_Angle_Measure[PITCH][MECH]);
////		//�ԽǶȺ��ٶȽ��ж��׿������˲��ں�,0λ��,1�ٶ�
//		yaw_kf_result = kalman_filter_calc(&yaw_kalman_filter, Cloud_Angle_Measure[YAW][GYRO], 0);
//		pitch_kf_result = kalman_filter_calc(&pitch_kalman_filter, Cloud_Angle_Measure[PITCH][MECH], 0);
////		debug_kf_angle_temp = 0;
//	}
    
    /**********************yawԤ��**********************/
    
    kf_delay_open = debug_kf_delay;
    if(VisionRecvData.identify_target == TRUE)
    {
        
        #if AUTO_PRE == 1
            //���Ԥ��
        Auto_KF_Delay++;//�˲���ʱ����
        if(VisionRecvData.auto_too_close == TRUE)
        {
			yaw_speed_k = debug_y_sp_k;///4.f;//3.f;//Ԥ��ϵ������
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
        /*Ԥ�⿪������*/
        if(ABS(Auto_Error_Yaw[NOW]) < debug_auto_err_y//debug�� 
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
            
            debug_kf_angle_temp = constrain_float(debug_kf_angle_temp, -debug_kf_y_angcon, debug_kf_y_angcon);//Ԥ���ݴ����޷�
			debug_kf_y_angle = RAMP_float(debug_kf_angle_temp, debug_kf_y_angle, debug_kf_angle_ramp);//Ԥ���������仯
			
			debug_kf_y_angle = constrain_float(debug_kf_y_angle, -debug_kf_y_angcon, debug_kf_y_angcon);
			Cloud_Angle_Target[YAW][GYRO] = yaw_kf_result[KF_ANGLE] + debug_kf_y_angle;//debug_y_sk * (yaw_kf_result[KF_SPEED] - debug_kf_speed);//Vision_Gyro_MovProj_Yaw(yaw_kf_result[1]);//yaw_kf_result[0];
			
           
            /*����������������������������Ԥ�⵽λ�жϡ�������������������������*/
            if( (yaw_kf_result[KF_SPEED]>0) //Ŀ�������������ֵ��ʾ˵Ŀ�����ұߣ���˵��Ԥ�⵽λ�ã��ɴ�
            && (Auto_Error_Yaw[NOW] < 0.3f) )
            {
                mobpre_yaw_right_delay = 0;//����Ԥ�⿪����ʱ����

				mobpre_yaw_left_delay++;
                if(mobpre_yaw_left_delay > 0/*75*/)//�ȶ�150ms
				{
					Mobi_Pre_Yaw_Fire = TRUE;//Ԥ�⵽λ���ɿ���
				}
				else
				{
					Mobi_Pre_Yaw_Fire = FALSE;//Ԥ��û��λ�����ɿ���
				}
            }
            else if( (yaw_kf_result[KF_SPEED]<0) //Ŀ�������������ֵ��ʾ˵Ŀ������ߣ���˵��Ԥ�⵽λ�ã��ɴ�
						&& (Auto_Error_Yaw[NOW] > -0.3f) )
			{
				mobpre_yaw_left_delay = 0;//����Ԥ�⿪����ʱ����
				
				mobpre_yaw_right_delay++;
				if(mobpre_yaw_right_delay > 0/*75*/)//�ȶ�150ms
				{
					Mobi_Pre_Yaw_Fire = TRUE;//Ԥ�⵽λ���ɿ���
				}
				else
				{
					Mobi_Pre_Yaw_Fire = FALSE;//Ԥ��û��λ�����ɿ���
				}

            }
            else
			{
				Mobi_Pre_Yaw_Fire = FALSE;//���Ԥ��û��λ����ֹ����				
				mobpre_yaw_left_delay = 0;//����Ԥ�⿪����ʱ����
				mobpre_yaw_right_delay = 0;//����Ԥ�⿪����ʱ����
			}
            Mobi_Prediction_Yaw = TRUE;//���Ԥ���ѿ���
			mobpre_yaw_stop_delay = 0;//���þ�ֹʱ�Ŀ����ӳ�
        }
        /*Ԥ������û�ﵽ���ر�Ԥ��*/
		else
		{
            if_yaw_auto_pre=0;
            Cloud_Angle_Target[YAW][GYRO] = yaw_kf_result[KF_ANGLE];
			Mobi_Prediction_Yaw = FALSE;//���Ԥ��û����
			mobpre_yaw_left_delay  = 0;//������Ԥ��Ŀ����ӳ�
			mobpre_yaw_right_delay = 0;//������Ԥ��Ŀ����ӳ�	
			if( fabs(Auto_Error_Yaw[NOW]) < 20 )//�ӽ�Ŀ��
			{
				mobpre_yaw_stop_delay++;
				if(mobpre_yaw_stop_delay > 25)//ֹͣ�ȶ�50ms
				{
					Mobi_Pre_Yaw_Fire = TRUE;//��ʱ�����Ӿ������־λ������жϣ��ǵ�һ��Ҫ��TRUE
				}
			}
			else
			{
				Mobi_Pre_Yaw_Fire = FALSE;//���û�ظ���λ����ֹ����
			}	
        }
    /*---------------pitchԤ��------------------*/  
        if(Auto_KF_Delay > debug_kf_delay 
        && fabs(Auto_Error_Pitch[NOW]) < debug_auto_err_p
        && fabs(pitch_kf_result[KF_SPEED]) > debug_kf_speed_p_low
        && (GIMBAL_AUTO_PITCH_Sen_SK() == FALSE || GIMBAL_AUTO_PITCH_Sen() == FALSE)
        && VisionRecvData.distance < 1000)		  
        {
            if(VisionRecvData.auto_too_close == TRUE)//Ŀ�����̫������СԤ��
			{
				pitch_speed_k = debug_p_sp_k/2.f;//Ԥ��ϵ������
				kf_pitch_angcon = debug_kf_p_angcon/1.5f;
			}
            else//����Ԥ����
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
			//pitchԤ�����޷�
			debug_kf_p_angle = constrain_float(debug_kf_p_angle, -kf_pitch_angcon, kf_pitch_angcon);
			
			Cloud_Angle_Target[PITCH][MECH] = pitch_kf_result[KF_ANGLE] + debug_kf_p_angle;

        }
        /*Ԥ������û�ﵽ���ر�Ԥ��*/
		else
		{
			Cloud_Angle_Target[PITCH][MECH] =pitch_kf_result[KF_ANGLE];
		}
        #else
            //Ԥ��ȫ���Ӿ�
            Cloud_Angle_Target[YAW][GYRO] = yaw_kf_result[0];
            Cloud_Angle_Target[PITCH][MECH] = pitch_kf_result[0];
        #endif
    }
    else    //δʶ��Ŀ��,�����������̨
    {
      yaw_kf_result = kalman_filter_calc(&yaw_kalman_filter, Cloud_Angle_Measure[YAW][GYRO], 0);
		pitch_kf_result = kalman_filter_calc(&pitch_kalman_filter, Cloud_Angle_Measure[PITCH][MECH], 0);
	
		if (modeGimbal == MECH)//��еģʽ
		{
			Cloud_Angle_Target[PITCH][MECH] += MOUSE_Y_MOVE_SPEED * kKey_Mech_Pitch;
			Cloud_Angle_Target[YAW][MECH]    = Mech_Mid_Yaw;	//yaw���ֲ���,��Զ���м�			
		}
		else if (modeGimbal == GYRO)
		{
			Mouse_Gyro_Yaw   += MOUSE_X_MOVE_SPEED * kKey_Gyro_Yaw;//��¼Ŀ��仯�Ƕ�
			Mouse_Gyro_Pitch += MOUSE_Y_MOVE_SPEED * kKey_Gyro_Pitch;//pitch�Ծ�ʹ�û�еģʽ
			if(MOUSE_X_MOVE_SPEED == 0)
			{
				Mouse_Yaw_Stop ++ ;
				if(Mouse_Yaw_Stop > 25)//��곤ʱ��ͣ����ֹͣ�ƶ�
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
				if(Mouse_Pitch_Stop > 25)//��곤ʱ��ͣ����ֹͣ�ƶ�
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
  * @brief  ��ͬģʽ��PIDֵ���л�
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
  * @brief  PID����
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
    	//�����п������˲�,������Ƶ�ͷ��ȶ���
	pitch_position_error = KalmanFilter(&Gimbal_Pitch_Mech_Error_Kalman, pitch_position_error);
    //�⻷
	pitch_speed_error[NOW]=Gimbal_PID_list[PITCH][MECH][actGimbal][OUTER][kp]*pitch_position_error*0.003f;
    
    pTermPitch=Gimbal_PID_list[PITCH][MECH][actGimbal][INNER][kp]*pitch_speed_error[NOW];
    iTermPitch+=Gimbal_PID_list[PITCH][MECH][actGimbal][INNER][ki]*pitch_speed_error[NOW];
    iTermPitch=constrain_float(iTermPitch,-PID_Iterm_Max,PID_Iterm_Max);
    dTermPitch=Gimbal_PID_list[PITCH][MECH][actGimbal][INNER][kd]*(pitch_speed_error[NOW]-pitch_speed_error[LAST]);
    //��� 
    pitch_out=pTermPitch+iTermPitch+dTermPitch;
    pitch_out=constrain_float(pitch_out,-PID_Out_Max,PID_Out_Max);
    pitch_speed_error[LAST]=pitch_speed_error[NOW];
    
    //yaw
    switch(modeGimbal)//��еģʽ
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
  * @brief  ģ��PID
  * @param  void
  * @retval void
  * @attention 
  */
void fuzzy_PID(void)
{

}

/**
  * @brief  PID�������
  * @param  void
  * @retval void
  * @attention 
  */

void task_PID_out(void)
{
    GIMBAL_PID_Mode();//���ݲ���ģʽ�任kpid,ÿ�ζ�Ҫ��		
    GIMBAL_PID();//PID����
    Gimbal_set_voltage(yaw_out,pitch_out);//CAN���
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
/****************************��������***********************************************/
/**
  * @brief  ������̨����̷���Ƕ�
  * @param  void
  * @retval void
  * @attention 
  */
void Gimbal_Chass_Separ_Limit(void)
{
	if ( (GIMBAL_GetOffsetAngle() <= -CLOUD_SEPAR_ANGLE				//�ҹ���
			&& (rc_t.mouse.x>0||rc_t.rc.ch1>0))	
				|| ( GIMBAL_GetOffsetAngle() >= CLOUD_SEPAR_ANGLE	//�����
					&& ( rc_t.mouse.x<0 || rc_t.rc.ch1<0) ) )
	{
//		RC_Ctl.mouse.x = 0;
//		//���Ե�ʱ��ǵ�ע�ͣ���������ch0�а�������ݵ����
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
		if ( (GIMBAL_GetOffsetAngle() <= (Mech_Left_Yaw - Mech_Mid_Yaw + 50)	&& RC_Ctl.mouse.x>0)//�ҹ���	
					|| ( GIMBAL_GetOffsetAngle() >= (Mech_Right_Yaw - Mech_Mid_Yaw - 50) && RC_Ctl.mouse.x<0 )	//�����
		   )
		{
			kKey_Gyro_Yaw = 0;
		}
		else
		{
			kKey_Gyro_Yaw   = -kkey_gyro_yaw;
		}
	#else
		if ( (GIMBAL_GetOffsetAngle() <= (Mech_Right_Yaw - Mech_Mid_Yaw +500) && rc_t.mouse.x>0)//�ҹ���	
					|| ( GIMBAL_GetOffsetAngle() >= (Mech_Left_Yaw - Mech_Mid_Yaw - 500) && rc_t.mouse.x<0 )//�����	
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
  * @brief  �ȴ�Pitch��ˮƽ
  * @param  void
  * @retval 1�ص�ˮƽλ��,0δ�ص�ˮƽλ��
  * @attention �Ƕ�С��50����Ϊ�ص�ˮƽ
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
  * @brief  ����YAWƫ�����ĽǶ�,���̸���ģʽ��
  * @param  void
  * @retval sAngleError,ƫ��Ƕ�ֵ,CAN�����Ļ�е�Ƕ�
  */
int16_t GIMBAL_GetOffsetAngle(void)
{
	int16_t sAngleError;

	sAngleError = (Cloud_Angle_Measure[YAW][MECH] - Mech_Mid_Yaw)*YAW_POSITION;


	//���㴦��,ͳһ���ӻ�
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
  * @brief  ����Ŀ���ٶ�
  * @param  void
  * @retval 
  */
float Target_Speed_Calc(speed_calc_data_t *S, uint32_t time, float position)
{
	S->delay_cnt++;

	if (time != S->last_time)
	{
		S->speed = (position - S->last_position) / (time - S->last_time) * 2;//�����ٶ�
           

		S->last_time = time;
		S->last_position = position;
		S->last_speed = S->speed;
		S->delay_cnt = 0;
	}

	if(S->delay_cnt > 300/*100*/) // delay 200ms speed = 0
	{
		S->processed_speed = 0;//ʱ���������Ϊ�ٶȲ���
	}

	return S->processed_speed;//��������ٶ�
}

/**
  * @brief  �Ƿ��������ڱ�
  * @param  void
  * @retval TRUE   FALSE
  * @attention ����̧ͷ�Ƕ�̫������Ϊ�ڴ��ڱ�
  */
bool GIMBAL_AUTO_PITCH_Sen(void)
{
	if(Mech_Max_Pitch-Cloud_Angle_Measure[PITCH][MECH] <= down_sb_pitch/*300*/ 
			|| IF_KEY_PRESSED_G)//̧ͷ�ӽ���λ
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}

/**
  * @brief  �Ƿ����еȾ��������ڱ�,�Ӵ�Ԥ��
  * @param  void
  * @retval TRUE   FALSE
  * @attention ����̧ͷ�Ƕ�̫������Ϊ�ڴ��ڱ�
  */
float pitch_sb_error = 0;
bool GIMBAL_AUTO_PITCH_Sen_SK(void)
{
	pitch_sb_error = Mech_Max_Pitch-Cloud_Angle_Measure[PITCH][MECH];
	if((pitch_sb_error<= down_sb_pitch/*450*//*550*/)
    && (pitch_sb_error > up_sb_pitch) )//̧ͷ�ӽ���λ
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}


/*--------------�������������еĺ�������ǰд��������������ɾ��------------*/

bool GIMBAL_AUTO_PITCH_Sentry(void)
{return 0;}
    
