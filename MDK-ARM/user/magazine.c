#include "magazine.h"
																	///ң����ƽʱ�����ã��ͼ��󲻳�ͻ����
#if    INFANTRY_DEBUG_ID==3
#define Magazine_Close_Angle   0//����
#define Magazine_Open_Angle    90//����
#endif

//���ֿ��ر�־λ
#define MAGA_STEP0    0		//ʧ�ܱ�־			///ͣ
#define MAGA_STEP1    1		//SW1��λ��־		///��
#define MAGA_STEP2    2		//���ֿ��ر�־	///��
uint8_t Magazine_Switch;

uint8_t Magazine_flag;		///1�ǿ���0�ǹ�
uint8_t Magazine_opened_flag;///��1��ʾ�򿪣������ʾ����
int16_t Magazine_Target;//PWMĿ��ֵ
int16_t Magazine_Real;//PWM��ʵֵ
int16_t Magazine_Ramp = 100;//����б��,���Ʊ仯�ٶ�		///����Ҫ��������

/**
  * @brief  ���ֶ��ֹͣת��
  * @param  void
  * @retval void
  * @attention ʧ�ر���,��0���ֲ���,��ʱ���ֶ��򿪵���
  */
void Magazine_StopCtrl(void)
{
    TIM1->CCR2 = 0;
}

void Magazine_init(void)
{
    //�Ƕȳ�ʼ��,ʹĿ��ֵ�����ֵ��Ϊ�ر�ֵ
		Magazine_Target = Magazine_Close_Angle;
		Magazine_Real = Magazine_Close_Angle;
		Magazine_flag = 0;
}
/**
  * @brief  �����ܿ�
  * @param  void
  * @retval void
  * @attention ʧ�ر���,��0���ֲ���,��ʱ���ֶ��򿪵���		///��TIM1->CCR2����
  */
void Magazine_Ctrl(void)
{
    static uint8_t opened_tick = 0;							///��ʼ����0����1���أ�
        if (SYSTEM_GetRemoteMode() == RC)				///����ģʽ����ң����ģʽ
        {  if (Magezine_Rc_Switch())//�ж��Ƿ�Ҫ���ָı䵱ǰ״̬
            {
                //�ı䵱ǰ״̬���ж�
                Magazine_flag=~Magazine_flag;		///�����Ҫ����Magazine_Servo()���flag
            }
        }	
		else if (SYSTEM_GetRemoteMode() == KEY)
		{
			Magazine_Key_Ctrl();
		}
        ///�жϵ������ڵ�״̬
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
		
    Magazine_Servo(Magazine_flag);   ///<*-*>���ö���ײ���ƺ��������ƶ���ı�״̬
}
    
/**
  * @brief  ң��ģʽ,�ж��Ƿ��´���״̬ת��ָ��,����һ��֮�����̱��FALSE
  * @param  void
  * @retval �Ƿ��´��˸ı�״̬��ָ��
  * @attention �߼��ϸ���,�ú�����
  */
bool Magezine_Rc_Switch(void)
{
	if (IF_RC_SW2_MID)//ң��ģʽ
	{
				if (IF_RC_SW1_UP)//������������1		///ң������s1�����ڸ�λ
				{
					if (Magazine_Switch == MAGA_STEP1)//������������2		///Ψ��Magazine_Switch���ڸ�λ״̬ʱ���ܽ��п��ϲ���
					{
						Magazine_Switch = MAGA_STEP2;			///����ʵ�Ǵ��ݸ����������Flag
					}
					else if (Magazine_Switch == MAGA_STEP2)//���ֹر�		///���ܲ�����ָ���չرգ����������һ�ε���״̬ת������
					{
						Magazine_Switch = MAGA_STEP0;//�ж���ϵ			///�������ν������һ�κ����̱�ΪFALSE��
					}
				}
				else		//��־SW1�Ƿ��и�λ�����,�ڸ�λ������²����ٴν���STEP2		
				{
					Magazine_Switch = MAGA_STEP1;//����SW1���´α任֮ǰһֱ������	///����SW1����UP��һֱ���ֵ�һ��
				}
	}
	else//s2�����м�,�������ֿ���
	{
		Magazine_Switch = MAGA_STEP0;//������Ħ���ֿ���Ҳ�������л��ɼ���ģʽ
	}
	
	if (Magazine_Switch == MAGA_STEP2)			///����Magazine_Switch����MAGA_STEP2״̬ʱ�ܷ���TRUE
	{
		return TRUE;//ֻ��SW1���±任��ʱ���ΪTRUE
	}
	else
	{
		return FALSE;
	}
}  

/**
  * @brief  ����ģʽ
  * @param  void
  * @retval void
  * @attention 														///û�н�Maga_Key_state[NOW]���ݸ�Maga_Key_state[LAST]�Ļ��ƣ�
  */
void Magazine_Key_Ctrl(void)
{
    static uint8_t Maga_tick=0;
    static uint8_t Maga_Key_state[2]={0,0};///���ڱ������Ҽ���״̬��1��0��
    static uint16_t Maga_time[2]={0,0};	///���ڼ�ʱ		///now��last֮�ֿ���������ʶ����
    
    if(IF_KEY_PRESSED_R)						
    {
        Maga_Key_state[NOW]=1;
    }
    else
    {
        Maga_Key_state[NOW]=0;
    }
		
		if(Maga_Key_state[NOW]!=Maga_Key_state[LAST])Maga_time[NOW]=system_time;///�������״̬����ʱ�ŻὫ����ʱ�����Ϊϵͳʱ��
    
    if(Maga_Key_state[NOW]==0&&Maga_Key_state[LAST]==1)///�ɿ��Ҽ�
        Maga_tick++;
    
    if(Maga_tick)
    {
        if(Maga_time[NOW]-Maga_time[LAST]>500)//500���û�ٴεõ��½��أ�ȡ�����μ���
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
		Maga_Key_state[LAST]=Maga_Key_state[NOW];///�ҽ���һ�ε����״̬���ݸ�����һ�Ρ�<*-*>
}


/**
  * @brief  ���ֶ������
  * @param  ����flag��
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
    if(temp>0)							///����ʵֵ������������100����ֱ��������ʵֵ����TIM1->CCR2��ֵ�仯��Ҫ̫��
    {
        if(temp>Magazine_Ramp)		///��Ŀ��ֵ����ʵֵ������100ʱ
        {
            Magazine_Real+=Magazine_Ramp;
        }
        else
        {
            Magazine_Real+=temp;		
        }
    }
    else if(temp<0)				///������������������ͬ�����̲�ͬ
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


