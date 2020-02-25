#include "vision.h"

eVisionAction    actVison;
extVisionRecvData_t      VisionRecvData;    //视觉接收结构体
extVisionSendData_t      VisionSendData;    //视觉发送结构体
uint8_t Attack_Color_Choose = ATTACK_NONE;//根据比赛需求修改
uint8_t Vision_Data_Flag=0;

/**
  * @brief  视觉总控制
  * @param  void
  * @retval void
  * @attention  
  */
void Vision_Ctrl(void)
{
	
//		if (GIMBAL_IfBuffHit() == TRUE && GIMBAL_IfManulHit() == FALSE)//自动打符模式
//		{
//			actVison = VISION_BUFF;
//		}
//		else if (GIMBAL_IfManulHit() == TRUE)//手动模式
//		{
//			actVison = VISION_MANU;
//		}
//		else
//		{
//			actVison = VISION_AUTO;
//		}

		switch(actVison)
		{
			/*- 打符 -*/
			case VISION_BUFF:
//				Vision_Buff_Ctrl();
			break;
			
			/*- 自瞄 -*/
			case VISION_AUTO:
				Vision_Auto_Attack_Ctrl();
			break;
			
			/*- 手动 -*/
			case VISION_MANU:
				Vision_Auto_Attack_Off();
			break;
		}
}

/**
  * @brief  自瞄控制
  * @param  void
  * @retval void
  * @attention  
  */
void Vision_Auto_Attack_Ctrl(void)
{
	/* 确定敌方颜色 */
	
	//向小电脑发送颜色识别指令
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
  * @brief  关闭自瞄
  * @param  void
  * @retval void
  * @attention  
  */
void Vision_Auto_Attack_Off(void)
{
//	Vision_Send_Data( VISION_OFF );
}


