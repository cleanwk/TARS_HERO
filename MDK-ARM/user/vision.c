#include "vision.h"

eVisionAction    actVison;
extVisionRecvData_t      VisionRecvData;    //�Ӿ����սṹ��
extVisionSendData_t      VisionSendData;    //�Ӿ����ͽṹ��
uint8_t Attack_Color_Choose = ATTACK_NONE;//���ݱ��������޸�
uint8_t Vision_Data_Flag=0;

/**
  * @brief  �Ӿ��ܿ���
  * @param  void
  * @retval void
  * @attention  
  */
void Vision_Ctrl(void)
{
	
//		if (GIMBAL_IfBuffHit() == TRUE && GIMBAL_IfManulHit() == FALSE)//�Զ����ģʽ
//		{
//			actVison = VISION_BUFF;
//		}
//		else if (GIMBAL_IfManulHit() == TRUE)//�ֶ�ģʽ
//		{
//			actVison = VISION_MANU;
//		}
//		else
//		{
//			actVison = VISION_AUTO;
//		}

		switch(actVison)
		{
			/*- ��� -*/
			case VISION_BUFF:
//				Vision_Buff_Ctrl();
			break;
			
			/*- ���� -*/
			case VISION_AUTO:
				Vision_Auto_Attack_Ctrl();
			break;
			
			/*- �ֶ� -*/
			case VISION_MANU:
				Vision_Auto_Attack_Off();
			break;
		}
}

/**
  * @brief  �������
  * @param  void
  * @retval void
  * @attention  
  */
void Vision_Auto_Attack_Ctrl(void)
{
	/* ȷ���з���ɫ */
	
	//��С���Է�����ɫʶ��ָ��
	if(Attack_Color_Choose == ATTACK_BLUE)
	{
//		Vision_Send_Data( VISION_BLUE );
	}
	else if(Attack_Color_Choose == ATTACK_RED)
	{
//		Vision_Send_Data( VISION_RED );
	}
	else if(Attack_Color_Choose == ATTACK_NONE)
	{
		Vision_Auto_Attack_Off();
	}
}


/**
  * @brief  �ر�����
  * @param  void
  * @retval void
  * @attention  
  */
void Vision_Auto_Attack_Off(void)
{
//	Vision_Send_Data( VISION_OFF );
}


