#include "shoot.h"

/**************ʹ�ñ���******************/
#define    FRIC_STEP0    0
#define    FRIC_STEP1    1
#define    FRIC_STEP2    2//ң��ģʽ�µĿ�����־λ
pid_t M3508E_PID;
pid_t M3508F_PID;//����Ħ����pid����
//�ٶ�ѡ��
#define FRI_OFF  	0
#define FRI_LOW  	1		//������٣�Ӣ��Ĭ�����Ϊ��������
//#define FRI_MID  	2		//B�����٣�
#define FRI_HIGH 	2		//B�� Ӣ�۸�����
#define FRI_MAD  	3		//�������٣�Ӣ���ݶ��ߵͣ���������
#define rubber_speed  500  //���ֵ��ٶ�
//#define FRI_MAX     5		//��������
fric_shoot_t fric_speed;
//ң��ģʽ�µ�һЩ��־λ
uint8_t Friction_Switch = 0;
uint8_t fric_changing_flag;
float fric_out1,fric_out2;//Ӣ������Ħ�����������ֵ
int shoot_ramp=5;
int shoot_mode;
int Fric_Speed_Level;
#if		INFANTRY_DEBUG_ID == 3
int shoot_target[4][2]=       //Ĭ��8000������ʱ11000�������⣩������10000�����⣩
{
    {0,0},
    {8000,8000},
    {11000,11000},
    {10000,10000}    
};    
#endif

/*************************************/
/**
  * @brief  Ħ���ֲ�����ʼ��
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
    PID_struct_init(&M3508E_PID,POSITION_PID,1.0,0.1,0,2000,2000);//����Ħ����pid��������
		PID_struct_init(&M3508F_PID,POSITION_PID,1.0,0.1,0,2000,2000);
}

/*
  * @brief  Ħ����ʧ�ر���
  * @param  void
  * @retval void
  * @attention ������СPWM���
  */
void shoot_StopMotor(void)
{

}
/************************Ħ�����ܿ���*****************************/

/**
  * @brief  Ħ���ֿ��ƺ���
  * @param  void
  * @retval void
  * @attention Ħ����ֹͣת��,����ر�,Ħ���ִӹرյ�����,��̨̧ͷ����
  */
float fric_rc_debug = 300;
void Shoot_Ctrl(void)
{
    if (SYSTEM_GetRemoteMode( ) == RC)//ң��ģʽ
		{
			
			
			if (FRIC_RcSwitch())//�ж�״̬�л�,�����ֿ����߼���ͬ
			{	//�л�Ϊ��
				if (Fric_Speed_Level !=FRI_LOW)
				{
					Fric_Speed_Level =FRI_LOW;
				}
				else//�л�Ϊ��
				{
					Fric_Speed_Level = FRI_LOW;//ң��ģʽ�µ��ٶ�ѡ�񣬵����٣������¼���ⵯ,����pitchҪ̧ͷ
				}
			}			            
		}
		else				//����ģʽ,�ɵ�����
		{
			FRIC_KeyLevel_Ctrl();
		}
        fric_speed.target_fric_spd[0] = shoot_target[Fric_Speed_Level][0];
        fric_speed.target_fric_spd[1] = shoot_target[Fric_Speed_Level][1];//Ħ����Ŀ��ֵ����0,��־��ң��ģʽ�¿���,����pitchҪ̧ͷ
        shoot_fric_ctrl();        
}
/**
  * @brief  ���ֿ��ƺ���
  * @param  void
  * @retval void
  * @attention ����ģʽ�������������������͵�
  */
void Rubber_Ctrl(void)
{
	if(SYSTEM_GetRemoteMode( ) == KEY)
	{
		Rubber_KeyLevel_Ctrl();
	}
	if(SYSTEM_GetRemoteMode( ) == RC)
	{
	 }        //ң�������Ʋ���
	
		
}
/**
  * @brief  ���ֿ���,����ר��
  * @param  
  * @retval void
  * @attention ����ģʽ���������ƽ����͵�
  */

void Rubber_KeyLevel_Ctrl(void)
{
	if(IF_MOUSE_PRESSED_LEFT)
	{
		shoot_rubber_ctrl();
	}

}


/**
  * @brief  Ħ����ң�ؿ���
  * @param  void
  * @retval �Ƿ�ת����ǰ״̬
  * @attention �����ֿ����߼���ͬ
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
  * @brief  Ħ���ֵȼ�����,����ר��,���ݵȼ��Զ���������
  * @param  ��ǰ�ȼ�
  * @retval void
  * @attention ����ģʽ�²���Ħ����
  */

void FRIC_KeyLevel_Ctrl(void)
{

    if (IF_MOUSE_PRESSED_LEFT)//�����飬�������
	{
		Fric_Speed_Level = FRI_LOW;//Ӣ���õ�����
		
	}
//	if (GIMBAL_IfBuffHit() == TRUE)
//	{
//		Fric_Speed_Level = FRI_MAD;//���ģʽ,�������		
//	}
	else if (IF_KEY_PRESSED_B)//�Ƽ����٣����٣�
	{
		Fric_Speed_Level = FRI_HIGH;				
		
	}
	//else if (IF_KEY_PRESSED_B)
	//{
		//Fric_Speed_Level = FRI_MID;//����Ƶ������ģʽ,�Ƽ��ⲫ��

	//}		
    else if(GIMBAL_AUTO_PITCH_Sentry() == TRUE)//�������
    {
			Fric_Speed_Level = FRI_MAD;

    }	
	else//��ֹ��������Ĭ�ϸ�����
	{
		Fric_Speed_Level = FRI_HIGH;	
		
	}

}

/*
  * @brief  ����Ħ����ת��
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
	 fric_set_output(fric_speed.fric_spd[0], fric_speed.fric_spd[1]);//	
		
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
Friction_set_voltage( 0,  fric_out1, fric_out2,0);//ԭ�����������ݺ�Ħ����һ�𷢳������ڽṹ���ˡ���ô���ٶ�
}
void abs_limit(float *a, float ABS_MAX)
{
    if(*a > ABS_MAX)
        *a = ABS_MAX;
    if(*a < -ABS_MAX)
        *a = -ABS_MAX;
}
//��ʼ��,Ӣ��Ħ������
void PID_struct_init(
    pid_t* pid,
    uint32_t mode,
    float 	kp, 
    float 	ki, 
    float 	kd,
    uint32_t maxout,
    uint32_t intergral_limit
   // uint32_t intergral_seperate,//���ַ����㷨��ֵ
)
{
    pid->i_limit = intergral_limit;
    pid->MaxOutput = maxout;
   // pid->pid_mode = mode;
   // pid->i_seperate=intergral_seperate;
    pid->p = kp;
    pid->i = ki;
    pid->d = kd;
	
}      

/**
    *@bref. calculate delta PID and position PID
    *@param[in] set�� target
    *@param[in] real	measure
    ע�͵�������ʽ�㷨�Լ�һЩ������,Ӣ��Ħ������
    */
float pid_calc(pid_t* pid, float get, float set){

    pid->get[NOW] = get;
    pid->set[NOW] = set;
    pid->err[NOW] = set - get;	//set - measure
//    if (pid->max_err != 0 && ABS(pid->err[NOW]) >  pid->max_err  )
//		return 0;
//	if (pid->deadband != 0 && ABS(pid->err[NOW]) < pid->deadband)
//		return 0;
    
//   if(pid->pid_mode == POSITION_PID) //λ��ʽp
//    {
        pid->pout = pid->p * pid->err[NOW];
//        if(ABS(pid->err[NOW])<pid->i_seperate)//��������㷨
        pid->iout += pid->i * pid->err[NOW];
        pid->dout = pid->d * (pid->err[NOW] - pid->err[LAST] );
        abs_limit(&(pid->iout), pid->i_limit);
        pid->pos_out = pid->pout + pid->iout + pid->dout;
        abs_limit(&(pid->pos_out), pid->MaxOutput);
//        pid->last_pos_out = pid->pos_out;	//update last time 
//    }
//    else if(pid->pid_mode == DELTA_PID)//����ʽP
//    {
//        pid->pout = pid->p * (pid->err[NOW] - pid->err[LAST]);
//        pid->iout = pid->i * pid->err[NOW];
//        pid->dout = pid->d * (pid->err[NOW] - 2*pid->err[LAST] + pid->err[LLAST]);
//        
//        abs_limit(&(pid->iout), pid->i_limit);
//        pid->delta_u = pid->pout + pid->iout + pid->dout;
//        pid->delta_out = pid->last_delta_out + pid->delta_u;
//        abs_limit(&(pid->delta_out), pid->MaxOutput);
//        pid->last_delta_out = pid->delta_out;	//update last time
//    }
//    
//    pid->err[LLAST] = pid->err[LAST];
//    pid->err[LAST] = pid->err[NOW];
//    pid->get[LLAST] = pid->get[LAST];
//    pid->get[LAST] = pid->get[NOW];
//    pid->set[LLAST] = pid->set[LAST];
//    pid->set[LAST] = pid->set[NOW];
//      return pid->pid_mode==POSITION_PID ? pid->pos_out : pid->delta_out;
        return pid->pos_out;
}
<<<<<<< Updated upstream


=======
>>>>>>> Stashed changes
//���ý���ת��
void shoot_rubber_ctrl(void)
{
	TIM8->CCR1=rubber_speed; //ͨ��1
}