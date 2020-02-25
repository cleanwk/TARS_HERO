#include "bsp_can.h"

uint32_t TxMailbox1=0;
uint16_t flaghcan2=0;
uint16_t flagcan1=0;
moto_handle M3508[4];
moto_handle GM6020[2];
moto_handle GM_TEST[2];
moto_handle M2006;

uint32_t can1_error=0;
uint32_t can2_error=0;


void drv_can_init()
{
	
	
	CAN_FilterTypeDef can_filter_st;//筛选器结构体变量
	
	can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
	can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
	can_filter_st.FilterIdHigh = 0x0000;  //32位ID
	can_filter_st.FilterIdLow = 0x0000;
	can_filter_st.FilterMaskIdHigh = 0x0000;  //32位MASK
	can_filter_st.FilterMaskIdLow = 0x0000;
	can_filter_st.FilterBank = 0;//过滤器0
	can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;///过滤器0关联至FIFO0
	can_filter_st.FilterActivation = ENABLE;
	
	HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
	HAL_CAN_Start(&hcan1);
	
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);//挂起中断允许
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_TX_MAILBOX_EMPTY);
	
	can_filter_st.SlaveStartFilterBank = 14;
    can_filter_st.FilterBank = 14;
  
	HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
	HAL_CAN_Start(&hcan2);
	
	HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);//挂起中断允许
	HAL_CAN_ActivateNotification(&hcan2, CAN_IT_TX_MAILBOX_EMPTY);


}

/***************接收回调函数*******************/
/**
  * @brief  RX_handle 将接收的数据进行转换
  * @brief  CAN1  motor_info[]    CAN2 UWB数据
  * @param  CAN_HandleTypeDef *hcan
  * @retval None
  */

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	HAL_StatusTypeDef	HAL_RetVal;
	CAN_RxHeaderTypeDef header;
    uint8_t rx_data[8];
	uint8_t i;
	

	if (hcan == &hcan1)
    {
		HAL_RetVal=HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, rx_data);
	
        if ( HAL_OK==HAL_RetVal)
		{			
			                  // get motor index by can_id
            switch (header.StdId)
            {    
                case M3508_A:
                case M3508_B:
                case M3508_C:
                case M3508_D:
                    i=header.StdId-0x201;
                    M3508[i].rotor_angle           = ((rx_data[0] << 8) | rx_data[1]);
                    M3508[i].rotor_speed           = ((rx_data[2] << 8) | rx_data[3]);
                    M3508[i].torque_current        = ((rx_data[4] << 8) | rx_data[5]);
                    M3508[i].temp                  =   rx_data[6];
                    break;
                case GM6020_YAW:
                case GM6020_PITCH:
                    i=header.StdId-0x209;
                    GM6020[i].rotor_angle           = ((rx_data[0] << 8) | rx_data[1]);
                    GM6020[i].rotor_speed           = ((rx_data[2] << 8) | rx_data[3]);
                    GM6020[i].torque_current        = ((rx_data[4] << 8) | rx_data[5]);
                    GM6020[i].temp                  =   rx_data[6];
                    break;
                case M2006_ID:
                    M2006.rotor_angle           = ((rx_data[0] << 8) | rx_data[1]);
                    M2006.rotor_speed           = ((rx_data[2] << 8) | rx_data[3]);
                    M2006.torque_current        = ((rx_data[4] << 8) | rx_data[5]);
                    M2006.temp                  =   rx_data[6];
								case M3508_E:
                case M3508_F:
					           i=header.StdId-0x202;
				            M3508[i].rotor_angle           = ((rx_data[0] << 8) | rx_data[1]);
                    M3508[i].rotor_speed           = ((rx_data[2] << 8) | rx_data[3]);
                    M3508[i].torque_current        = ((rx_data[4] << 8) | rx_data[5]);
                    M3508[i].temp                  =   rx_data[6];
                    if(((int)M2006.torque_current)>2000)
                    {
                        
                    }
                    break;
            }
		}
        
        else 
        {
            can1_error++;
        }
    }
        
	else if(hcan == &hcan2)
	{
		
	    HAL_RetVal=HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, rx_data);
		if ( HAL_OK==HAL_RetVal)
		{
			flaghcan2++;
            switch (header.StdId)
            {
                case CAN_RM_Rx1:
                    rc_t.rc.ch1=((rx_data[0]<<8)|(rx_data[1]));
                    rc_t.rc.ch2=((rx_data[2]<<8)|(rx_data[3]));
                    rc_t.rc.ch3=((rx_data[4]<<8)|(rx_data[5]));
                    rc_t.rc.ch4=((rx_data[6]<<8)|(rx_data[7]));
                break;
                case CAN_RM_Rx2:
                    rc_t.mouse.x=((rx_data[0]<<8)|(rx_data[1]));
                    rc_t.mouse.y=((rx_data[2]<<8)|(rx_data[3]));
                    rc_t.mouse.z=((rx_data[4]<<8)|(rx_data[5]));
                    rc_t.kb.key_code=((rx_data[6]<<8)|(rx_data[7]));
                break;
                case CAN_RM_Rx3:
                    rc_t.rc.sw1=rx_data[0]&0x03;
                    rc_t.rc.sw2=(rx_data[0]>>3)&0x03;
                    rc_t.mouse.l=(rx_data[0]>>6)&0x01;
                    rc_t.mouse.r=(rx_data[0]>>7)&0x01;
                    rc_t.wheel = (rx_data[2] | rx_data[1] << 8);
                if(rc_t.rc.sw1==0&&rc_t.rc.sw2==0)
                {
                    
                }
            }
		}
		
        else 
        {
            can2_error++;
        }
	
  }
	
}

/********************发送函数********************/
/**
  * @brief  set_motor_voltage 发送数据
  * @param  id_range  数据发送报头
  * @param  v1 0-1位数据
  * @param  v2 2-3位数据
  * @param  v3 4-5位数据
  * @param  v4 6-7位数据
  * @retval None
  */


void Chassis_set_voltage(int16_t v1, int16_t v2, int16_t v3, int16_t v4)
{
	CAN_TxHeaderTypeDef tx_header;
	uint8_t             tx_data[8];
	
	tx_header.StdId = 0x200;
	tx_header.IDE   = CAN_ID_STD;
	tx_header.RTR   = CAN_RTR_DATA;
	tx_header.DLC   = 8;

	tx_data[0] = (v1>>8)&0xff;
	tx_data[1] =    (v1)&0xff;
	tx_data[2] = (v2>>8)&0xff;
	tx_data[3] =    (v2)&0xff;
	tx_data[4] = (v3>>8)&0xff;
	tx_data[5] =    (v3)&0xff;
	tx_data[6] = (v4>>8)&0xff;
	tx_data[7] =    (v4)&0xff;
	HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data,(uint32_t*)CAN_TX_MAILBOX0);

}

void Gimbal_set_voltage(int16_t v1, int16_t v2)
{
	CAN_TxHeaderTypeDef tx_header;
	uint8_t             tx_data[8];
	
	tx_header.StdId = 0x2ff;
	tx_header.IDE   = CAN_ID_STD;
	tx_header.RTR   = CAN_RTR_DATA;
	tx_header.DLC   = 4;

	tx_data[0] = (v1>>8)&0xff;
	tx_data[1] =    (v1)&0xff;
	tx_data[2] = (v2>>8)&0xff;
	tx_data[3] =    (v2)&0xff;
	
	HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data,(uint32_t*)CAN_TX_MAILBOX0);

}
void Friction_set_voltage(int16_t v1, int16_t v2, int16_t v3, int16_t v4)
{
	CAN_TxHeaderTypeDef tx_header;
	uint8_t             tx_data[8];
	
	tx_header.StdId = 0x1ff;
	tx_header.IDE   = CAN_ID_STD;
	tx_header.RTR   = CAN_RTR_DATA;
	tx_header.DLC   = 8;
  tx_data[0] = (v1>>8)&0xff;
	tx_data[1] =    (v1)&0xff;
	tx_data[2] = (v2>>8)&0xff;
	tx_data[3] =    (v2)&0xff;
	tx_data[4] = (v3>>8)&0xff;
	tx_data[5] =    (v3)&0xff;
	tx_data[6] = (v4>>8)&0xff;
	tx_data[5] =    (v4)&0xff;
	
	HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data,(uint32_t*)CAN_TX_MAILBOX0);

}



/*******************发送邮箱回调*******************/
void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan)
{ 
	
	if (hcan == &hcan1)
  {
        
  }
 
}

void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan)
{
	TxMailbox1++;
	if (hcan == &hcan1)
  {
	
  }

}

void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan)
{
	
	if (hcan == &hcan1)
  {
		
  }
  
}
