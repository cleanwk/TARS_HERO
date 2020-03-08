#include "revolver.h"


/******����,�����߼�����̨����*********/
#define     ROTATION ��         1           //˳��ʱ�� 

#define 	REVOL_SPEED_GRID    11			//���̸���
#define    	AN_BULLET           26807		//�����ӵ����λ������ֵ 8191/11*36
#define     REVOL_SPEED_FRE     196.36        //����ٶȺ���Ƶ�Ĺ�ϵREVOL_SPEED_RATIO/REVOL_SPEED_GRID


//���̵��ģʽ,�����봮��
eRevolverCtrlMode Revolver_mode;
//����ģʽ
eShootAction actShoot;

//����Ŀ��Ƕ��ۼƺ�,����λ��PID����
float  Revolver_Angle_Target_Sum;
float  Revolver_Target_Sum_temp;//���ģʽ�µ�Ŀ��ֵ�ݴ棬��ʱĿ��Ƕ���б�´���
float  Revolver_Ramp = AN_BULLET/40;//40msתһ��,һ�����ܳ���50ms

//�ۼƺ�
float Revolver_Angle_Measure_Sum;//���̲����Ƕ��ۼƺ�,����λ��PID
int16_t Revolver_Angle_Measure_Prev;//�ϴ�ʣ�µ��ۼӺͽǶ�,����Ȧ�������ж�


/*******************���̲���**********************/
//���̲����ٶ�
int16_t Revolver_Speed_Measure;
//���̲����Ƕ�
int16_t Revolver_Angle_Measure;//�����в����Ƕ�û�м��ϣ�û�ҵ����и�ֵ�ģ�Ӧ�ü���3508��ʵʩ�Ƕȸ�ֵ
//�����ٶ����
float Revolver_Speed_Error;
//���̽Ƕ����
float Revolver_Angle_Error[2];//  inner/outer
//����Ŀ��Ƕ�
float  Revolver_Angle_Target;
//����Ŀ��ת��
float  Revolver_Speed_Target;
/****************��Ƶ����******************/
#define SHOOT_LEFT_TIME_MAX  20	//��������л����

//�����ٶȻ���Ƶ
int16_t Revolver_Freq;
//λ�û�������,ʵʱ�ɱ�,��ֵԽСλ�û�������Խ��
uint32_t Shoot_Interval = 0;

//����������Ӧ��ģʽ�л�ʱ��ʱ���óɵ�ǰʱ��
uint32_t  Revol_Posit_RespondTime = 0;

/*����*/
uint8_t Revol_Switch_Left = 0;

/*****************PID����*****************/
float pTermRevolSpeed, iTermRevolSpeed;	
float pTermRevolAngle[2], iTermRevolAngle[2];//  inner/outer
float Revolver_Speed_kpid[3];//	kp/ki/kd
float Revolver_Angle_kpid[2][3];//  inner/outer    kp/ki/kd

/**********�޷�*************/
//��������޷�
float Revolver_Output_Max;
float iTermRevolSpeedMax;//�ٶȻ�΢���޷�
float iTermRevolPosiMax;//λ�û�΢���޷�

//���̵�������
float Revolver_Final_Output;

/********���**********/
//�����ӵ���,��һ�¼�һ��,��һ�ż�һ��
int16_t Key_ShootNum;//����������
int16_t ShootNum_Allow = 0;//��ʣ���ŵ����Դ�
uint16_t Residue_Heat;//ʣ���������,�������ƿ���
uint16_t Shoot_HeatLimit;//��ǰ�ȼ������������
uint16_t Shoot_HeatIncSpeed;//��ǰ�����ӵ���������ֵ

/**
  * @brief  ���̲�����ʼ��
  * @param  void
  * @retval void
  * @attention 
  */
void REVOLVER_InitArgument(void)
{
	/* Ŀ��ֵ */
	Revolver_Final_Output = 0;
	Revolver_Speed_Target = 0;
	
	/* PID���� */
	  //�ٶȻ�
	Revolver_Speed_kpid[kp] = 15;
	Revolver_Speed_kpid[ki] = 0;
	Revolver_Speed_kpid[kd] = 0;
	  //λ�û�
	Revolver_Angle_kpid[OUTER][kp] = 0.4;
	Revolver_Angle_kpid[OUTER][ki] = 0;
	Revolver_Angle_kpid[OUTER][kd] = 0;
	Revolver_Angle_kpid[INNER][kp] = 6;
	Revolver_Angle_kpid[INNER][ki] = 0;
	Revolver_Angle_kpid[INNER][kd] = 0;
	
	/* �޷� */
	iTermRevolSpeedMax  = 250;
	iTermRevolPosiMax   = 2500;
	Revolver_Output_Max = 9999;
	
	/* ��� */
	Key_ShootNum    = 0;
	Shoot_HeatLimit = 240;//������ʼ��
	Revolver_Freq   = 0;//��Ƶ��ʼ��
	
	/* λ�û�Ŀ��Ƕ� */
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
     //��������
		if(Revolver_Heat_Limit() == FALSE) //һ��Ҫ�����ٶȡ�λ�ÿ���֮ǰ
            //���ʱ��������
		{
			REVOLVER_Rest();//��������,�������������Ҳ��������
		}
		
		if(Revolver_mode == REVOL_SPEED_MODE)
		{
			REVOL_SpeedLoop();
		}
		else if(Revolver_mode == REVOL_POSI_MODE)
		{
			REVOL_PositionLoop();
		}
		
		if(IF_FRIC_READY())//Ħ���ֿ���
		{
			REVOLVER_CANbusCtrlMotor(Revolver_Final_Output);
		}
		else
		{
			Revolver_Speed_Target = 0;//Ħ���ֹر�,���̲�����
			Revolver_Angle_Rest();//Ħ���ֹرգ��������ڼ�Ĵ�ָ��
			REVOL_SpeedLoop();
			REVOLVER_CANbusCtrlMotor(Revolver_Final_Output);
		}       
}
#define REVOL_STEP0    0		//ʧ�ܱ�־
#define REVOL_STEP1    1		//SW1��λ��־
#define REVOL_STEP2    2		//���ֿ��ر�־
uint8_t	Revolver_Switch = 0;//����ң��ģʽ���ر�־λת��
bool REVOLVER_Rc_Switch(void)
{
//	if (IF_RC_SW2_MID || IF_RC_SW2_DOWN)//��е��������ģʽ
	if (IF_RC_SW2_MID)//��еģʽ
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
  * @brief  ��������
  * @param  void
  * @retval void
  * @attention ǹ�ڳ���������
  */
void REVOLVER_Rest(void)
{
	Key_ShootNum = 0;//λ�û���������
	Revolver_Speed_Target = 0;//�ٶȻ�ֹͣת��
	
	//�ٶȻ�λ������
	Revolver_Angle_Target_Sum   = Revolver_Angle_Measure;//λ�û�Ŀ��Ƕ�����
	Revolver_Angle_Measure_Sum  = Revolver_Angle_Measure;//λ�û�ת���Ƕ�����
	Revolver_Angle_Measure_Prev = Revolver_Angle_Measure;//�ϴ�λ������
	Revolver_Target_Sum_temp 	= Revolver_Angle_Measure;
	
	//PID��������
	iTermRevolSpeed = 0;
	iTermRevolAngle[INNER] = 0;
}

/**
  * @brief  ���̽Ƕ�����
  * @param  void
  * @retval void
  * @attention ģʽ�л�ʱ��,��ֹ�´��л�ȥ��ͻȻ��һ��
  */
void Revolver_Angle_Rest(void)
{
	Key_ShootNum = 0;
	Revolver_Angle_Target_Sum   = Revolver_Angle_Measure;//λ�û�Ŀ��Ƕ�����
	Revolver_Angle_Measure_Sum  = Revolver_Angle_Measure;//λ�û�ת���Ƕ�����
	Revolver_Angle_Measure_Prev = Revolver_Angle_Measure;//�ϴ�λ������
	Revolver_Target_Sum_temp 	= Revolver_Angle_Target_Sum;
}
/**
  * @brief  ���̵�ң��ģʽ
  * @param  void
  * @retval void
  * @attention ң�����ٶȻ�
  */
void REVOLVER_Rc_Ctrl(void)
{	
	/*******************�����**********************/
	
	if(IF_RC_SW2_MID)//��еģʽ�µ�����������Ե���
	{
		Revolver_mode = REVOL_POSI_MODE;//λ�û�
		if(REVOLVER_Rc_Switch() == TRUE)
		{
			Key_ShootNum++;//��һ��
		}
		
		if(revol_remot_change == TRUE)//�մӼ���ģʽ�л���������շ�������
		{
			revol_remot_change = FALSE;
			Revolver_Angle_Rest();//��ֹͻȻ�Ӽ����е�ң�ؿ�ת
		}
		
		if(Key_ShootNum != 0)
		{
			Key_ShootNum--;
			Revolver_Target_Sum_temp += AN_BULLET;
		}
		
		if(Revolver_Angle_Target_Sum != Revolver_Target_Sum_temp)//����ת��ȥ
		{
			Revolver_Angle_Target_Sum = RAMP_float(Revolver_Target_Sum_temp, Revolver_Angle_Target_Sum, Revolver_Ramp);
		}
		
//		REVOL_PositStuck();//�����жϼ���ת
	}
	/**************������*******************/
	else if(IF_RC_SW2_DOWN)//������ģʽ
	{
		Revolver_mode = REVOL_SPEED_MODE;//�ٶȻ�
		
		if(IF_RC_SW1_DOWN)//sw1�´�
		{
			Revolver_Freq = 5;//��Ƶѡ��
			//�ٶȻ�ת������
			Revolver_Speed_Target = REVOL_SPEED_FRE*Revolver_Freq;
		}
		else	//ң��ģʽ�رղ���
		{
			Revolver_Speed_Target = 0;
			Revolver_Freq = 0;
		}

//		REVOL_SpeedStuck();//�����жϼ���ת
	}
	/********************************************/
}

/*******����ģʽ************/

/**
  * @brief  ���̵ļ���ģʽ
  * @param  void
  * @retval void
  * @attention ������λ�û�����
  */
void REVOLVER_Key_Ctrl(void)
{
	Revolver_mode = REVOL_POSI_MODE;//��ֹ������,Ĭ��λ�û�
	
	SHOOT_NORMAL_Ctrl();//ȷ�����ģʽ
	
	/*- ȷ�������������ģʽ -*/
	switch(actShoot)//����ʲô���Ȳ�ע�ͣ�����������һ��Ӣ�۾��巢������
	{
		case SHOOT_NORMAL:
			//���ģʽѡ��,Ĭ�ϲ���
			SHOOT_NORMAL_Ctrl();
		break;
		
		case SHOOT_SINGLE:
			//��һ���������,��������
			SHOOT_SINGLE_Ctrl();
		break;
		
		case SHOOT_TRIPLE:
			//��������
			SHOOT_TRIPLE_Ctrl();
		break;
		
		case SHOOT_HIGHTF_LOWS:
			//B����Ƶ
			SHOOT_HIGHTF_LOWS_Ctrl();
		break;
		
		case SHOOT_MIDF_HIGHTS:
			//Z�Ƽ�
			SHOOT_MIDF_HIGHTS_Ctrl();
		break;
		
		case SHOOT_BUFF:
//			//����Զ���
		break;
		
		case SHOOT_AUTO:
			//�Ҽ�����ʱ�Զ���
//			SHOOT_AUTO_Ctrl();
		break;		
	}
	
	/*- ��ʼ����,������ -*/
	if(Revolver_mode == REVOL_SPEED_MODE && IF_FRIC_READY() )
	{
		REVOLVER_KeySpeedCtrl();
	}
	else if(Revolver_mode == REVOL_POSI_MODE && IF_FRIC_READY())
	{
		REVOLVER_KeyPosiCtrl();
	}
}

/************************���̼���ģʽ����ģʽС����****************************/
/**
  * @brief  ����ģʽ�·���ģʽѡ��
  * @param  void
  * @retval void
  * @attention  ��ͨģʽ�������,�Ҳ�����
  */
void SHOOT_NORMAL_Ctrl(void)
{
	static uint32_t shoot_left_time = 0;//�����������ʱ��,ʱ������л�������
	
	/*------ ���̧�����ܴ���һ�� -------*/
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
		shoot_left_time = 0;//������¼�ʱ
	}
	/*------------------------------------*/
	
	if(IF_MOUSE_PRESSED_LEFT &&	shoot_left_time <= SHOOT_LEFT_TIME_MAX	//�������
			&& !IF_KEY_PRESSED_B && !IF_KEY_PRESSED_Z /*&& !GIMBAL_IfAutoHit()*/)
	{
		Revolver_mode = REVOL_POSI_MODE;//λ�û���
		shoot_left_time++;//�жϳ���,�л�
		actShoot = SHOOT_SINGLE;	
	}
	else if(IF_MOUSE_PRESSED_LEFT && shoot_left_time > SHOOT_LEFT_TIME_MAX	//��������200ms
				&& !IF_KEY_PRESSED_B && !IF_KEY_PRESSED_Z /*&& !GIMBAL_IfAutoHit()*/)
	{
//		shoot_left_time++;
		Revolver_mode = REVOL_POSI_MODE;
		actShoot = SHOOT_TRIPLE;//����ģʽ
	}
	else if(IF_MOUSE_PRESSED_LEFT	//Ӣ���������٣����ݽṹ������ģʽҲ�ø��ģ�
				/*&& !IF_MOUSE_PRESSED_LEFT && !IF_KEY_PRESSED_Z && !GIMBAL_IfAutoHit()*/)
	{
		Revolver_mode = REVOL_POSI_MODE;
		actShoot = SHOOT_HIGHTF_LOWS;
		shoot_left_time = 0;
	}
	else if(IF_KEY_PRESSED_B	//Ӣ�۸�����
			/*	&& !IF_MOUSE_PRESSED_LEFT && !IF_KEY_PRESSED_B*/)
	{
		Revolver_mode = REVOL_POSI_MODE;
		actShoot = SHOOT_MIDF_HIGHTS;
		shoot_left_time = 0;
	}
	else if(actGimbal == GIMBAL_AUTO /*&& VisionRecvData.centre_lock==TRUE*/  //�Ӿ�˵���Դ���
				 && !IF_KEY_PRESSED_Z && !IF_KEY_PRESSED_B)
	{
		Revolver_mode = REVOL_POSI_MODE;
		actShoot = SHOOT_AUTO;
		shoot_left_time = 0;
	}
//	else if(GIMBAL_IfBuffHit() == TRUE && GIMBAL_IfManulHit() == FALSE)//���ģʽ�ҷ��ֶ����ģʽ
//	{
//		Revolver_mode  = REVOL_POSI_MODE;
//		actShoot = SHOOT_BUFF;
//		shoot_left_time = 0;
//	}
	else
	{
		actShoot = SHOOT_NORMAL;
		Shoot_Interval  = 0;//����������
		Revol_Posit_RespondTime = system_time;//������Ӧ
		shoot_left_time = 0;
		Key_ShootNum = 0;
	}
//	
//	if(GIMBAL_IfBuffHit() == FALSE)//�˳��˴��ģʽ
//	{
//		First_Into_Buff = TRUE;	
//		Buff_Shoot_Begin = FALSE;
//		buff_fire = FALSE;
//	}
}

/**
  * @brief  ��������
  * @param  void
  * @retval void
  * @attention  
  */
void SHOOT_SINGLE_Ctrl(void)
{
	long  CurrentTime = 0;
	static uint32_t  RespondTime = 0;//��Ӧ�����ʱ
	
	CurrentTime = system_time;
	
	Shoot_Interval = 125;//1000/8;//���һ��8��
	
	if(RespondTime < CurrentTime
			&& Revol_Switch_Left == 2//�뵯�ֿ���ͬ��
				&& Key_ShootNum == 0)
	{
		RespondTime = CurrentTime + Shoot_Interval;
		Key_ShootNum++;
	}
}

/**
  * @brief  ��������
  * @param  void
  * @retval void
  * @attention  
  */
void SHOOT_TRIPLE_Ctrl(void)
{
	Revolver_mode = REVOL_SPEED_MODE;

	if(JUDGE_usGetShootCold() <= 40)
	{
		Revolver_Freq = 8;//��Ƶѡ��
	}
	else if(JUDGE_usGetShootCold() <= 60 && JUDGE_usGetShootCold() > 40)
	{
		Revolver_Freq = 8;//10;//��Ƶѡ��
	}
	else if(JUDGE_usGetShootCold() <= 80 && JUDGE_usGetShootCold() > 60)
	{
		Revolver_Freq = 8;//12;//��Ƶѡ��
	}
	else if(JUDGE_usGetShootCold() >= 160)//ռ��ﱤ
	{
		Revolver_Freq = 14;//12;//��Ƶѡ��
	}
	else
	{
		Revolver_Freq = 8;//��Ƶѡ��
	}
	
	//�ٶȻ�ת������
	Revolver_Speed_Target = REVOL_SPEED_FRE*Revolver_Freq;
}

/**
  * @brief  ����Ƶ�����ٿ���
  * @param  void
  * @retval void
  * @attention  
  */
void SHOOT_HIGHTF_LOWS_Ctrl(void)
{
    long CurrentTime;
	static uint32_t  RespondTime = 0;//��Ӧ�����ʱ	
	CurrentTime=system_time;
	Shoot_Interval =5; //1000/20;//ȷ����Ƶ
	
	if(RespondTime < CurrentTime
			&& Key_ShootNum == 0)
	{
		RespondTime = CurrentTime + Shoot_Interval;
		Key_ShootNum++;
	}
}

/**
  * @brief  ����Ƶ�����ٿ���
  * @param  void
  * @retval void
  * @attention  
  */
void SHOOT_MIDF_HIGHTS_Ctrl(void)
{
	long  CurrentTime = 0;
	static uint32_t  RespondTime = 0;//��Ӧ�����ʱ	
	
	CurrentTime = system_tick;//��ǰϵͳʱ��
	
	Shoot_Interval = 10;//TIME_STAMP_1000MS/10;//ȷ����Ƶ
	
	if(RespondTime < CurrentTime
			&& Key_ShootNum == 0)
	{
		RespondTime = CurrentTime + Shoot_Interval;
		Key_ShootNum++;
	}
}

/**
  * @brief  �����������
  * @param  void
  * @retval void
  * @attention  
*/
void SHOOT_AUTO_Ctrl(void)
{
	long CurrentTime;
	static uint32_t RespondTime_Stop    = 0;//��Ӧ�����ʱ����ֹ
	static uint32_t RespondTime_MobiPre = 0;//��Ӧ�����ʱ���ƶ�Ԥ��
	CurrentTime = system_tick;

/***********************************************************************/
	if( if_yaw_auto_pre == TRUE)	//������Ԥ��			
	{
		Shoot_Interval = 6;//1000/15;//TIME_STAMP_50MS;//ȷ����Ƶ
		if(if_yaw_auto_pre==TRUE				//�Լ���Ԥ�⵽��λ��
				&& RespondTime_MobiPre < CurrentTime
					&& Key_ShootNum == 0 
						&& IF_MOUSE_PRESSED_LEFT)//�������
		{
			RespondTime_MobiPre = CurrentTime + Shoot_Interval;
			Key_ShootNum ++;
		}
		else//������Ԥ�⵫Ԥ�ⲻ��λ����ֹ��
		{
			Key_ShootNum = 0;
		}
	}
	else if(if_yaw_auto_pre == FALSE)	//û��Ԥ��
	{
		Shoot_Interval = 100;//TIME_STAMP_1000MS/5;//ȷ����Ƶ
		if(if_yaw_auto_pre==TRUE		//�Լ���Ԥ�⵽��λ��
				&& RespondTime_Stop < CurrentTime
					&& Key_ShootNum == 0
						&& IF_MOUSE_PRESSED_LEFT)//�������				
		{
			RespondTime_Stop = CurrentTime + Shoot_Interval;//TIME_STAMP_500MS;//ÿ��1s������һ��		
			Key_ShootNum += 3;
//			Key_ShootNum ++;
		}
	}
}

/**
  * @brief  ����ģʽ�����ٶȻ�����
  * @param  void
  * @retval void
  * @attention �Ƽ�ģʽ������Ƶ,�����ǵü���һ����־λ��������Ħ���ֵ�������
  */
void REVOLVER_KeySpeedCtrl(void)
{
//	REVOL_SpeedStuck();//�����жϼ���ת
}

/**
  * @brief  ����ģʽ����λ�û�����
  * @param  void
  * @retval void
  * @attention 
  */
void REVOLVER_KeyPosiCtrl(void)
{
	long  CurrentTime ;
//	static uint32_t  RespondTime = 0;//��Ӧ�����ʱ
	
	CurrentTime = system_time;
	
	if(Key_ShootNum != 0 && Revol_Posit_RespondTime < CurrentTime)
	{
		Revol_Posit_RespondTime = CurrentTime + Shoot_Interval;
		Key_ShootNum--;//����������
		Revolver_Target_Sum_temp += AN_BULLET;//����λ�ü�
		
//		posishoot_time = xTaskGetTickCount();//����ָ���´�ʱ��ϵͳʱ��,���ڷ�����ʱ����
	}		
	
	if(Revolver_Angle_Target_Sum != Revolver_Target_Sum_temp)//����ת��ȥ
	{
		Revolver_Angle_Target_Sum = RAMP_float(Revolver_Target_Sum_temp, Revolver_Angle_Target_Sum, Revolver_Ramp);
	}
	
//	REVOL_PositStuck();//�����жϼ���ת,��ǰ�������һ��
}
/**
  * @brief  ͳ��ת���Ƕ��ܺ�
  * @param  void
  * @retval void
  * @attention �л���ģʽ֮��ǵ����� 
  */
void REVOL_UpdateMotorAngleSum(void)
{		 
	//�ٽ�ֵ�жϷ�
	if (ABS(Revolver_Angle_Measure - Revolver_Angle_Measure_Prev) > 4095)//ת����Ȧ
	{		
		//���β����Ƕ�С���ϴβ����Ƕ��ҹ��˰�Ȧ,��˵�����ι������
		if (Revolver_Angle_Measure < Revolver_Angle_Measure_Prev)//����Ȧ�ҹ����
		{
			//�Ѿ�ת����һȦ,���ۼ�ת�� 8191(һȦ) - �ϴ� + ����
			Revolver_Angle_Measure_Sum += 8191 - Revolver_Angle_Measure_Prev + Revolver_Angle_Measure;
		}
		else
		{
			//������һȦ
			Revolver_Angle_Measure_Sum -= 8191 - Revolver_Angle_Measure + Revolver_Angle_Measure_Prev;
		}
	}
	else      
	{
		//δ���ٽ�ֵ,�ۼ���ת���ĽǶȲ�
		Revolver_Angle_Measure_Sum += Revolver_Angle_Measure - Revolver_Angle_Measure_Prev;
	}

	//��¼��ʱ����Ƕ�,��һ�μ���ת���ǶȲ���,�����ж��Ƿ�ת��1Ȧ
	Revolver_Angle_Measure_Prev = Revolver_Angle_Measure;
}


/***********************PID����**********************/

/**
  * @brief  �ٶȻ�PID����
  * @param  void
  * @retval void
  * @attention  ң��ֻ���ٶȻ�
  */
void REVOL_SpeedLoop(void)
{  
	Revolver_Speed_Error = Revolver_Speed_Target - Revolver_Speed_Measure;

	//���͵���PID�㷨
	pTermRevolSpeed   = Revolver_Speed_Error * Revolver_Speed_kpid[kp];
	iTermRevolSpeed  += Revolver_Speed_Error * Revolver_Speed_kpid[ki];
	iTermRevolSpeed   = constrain_float( iTermRevolSpeed, -iTermRevolSpeedMax, iTermRevolSpeedMax );

	Revolver_Final_Output = constrain_float( pTermRevolSpeed + iTermRevolSpeed, -Revolver_Output_Max, +Revolver_Output_Max );
}

/**
  * @brief  λ�û�PID����
  * @param  void
  * @retval void
  * @attention  ����ģʽ
  */
void REVOL_PositionLoop(void)
{
	//��ȡת�����ܽǶ�ֵ
	REVOL_UpdateMotorAngleSum();
	
	//�⻷����
	Revolver_Angle_Error[OUTER] = Revolver_Angle_Target_Sum - Revolver_Angle_Measure_Sum;
	pTermRevolAngle[OUTER] = Revolver_Angle_Error[OUTER] * Revolver_Angle_kpid[OUTER][kp];

	//�ڻ�����
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
  * @brief  ǹ����������
  * @param  void
  * @retval �����Ƿ���
  * @attention  ����Ҫ����һ�²���,����ʣ��ɷ����������ջ�
  *             �����˫ǹ����˺���������
  */
bool Revolver_Heat_Limit(void)
{return 1;}

uint16_t JUDGE_usGetShootCold(void)
{
    return 100;
}
