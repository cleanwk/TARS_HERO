/**
 ***************************************(C) COPYRIGHT 2018 DJI***************************************
 * @file       bsp_uart.c
 * @brief      this file contains rc data receive and processing function
 * @note       
 * @Version    V1.0.0
 * @Date       Jan-30-2018
 ***************************************(C) COPYRIGHT 2018 DJI***************************************
 */
                                                                                                              
#include "string.h"
#include "stdlib.h"
#include "bsp_uart.h"
#include "main.h"
uint8_t   dbus_buf[DBUS_BUFLEN];
rc_info_t rc_t;//存储遥控器数据的结构体
uint8_t TxBuffer[] = "DMA_Semaphore test\n";//发送

/**
接收数据函数初始化
  */
static int uart_receive_dma_no_it(UART_HandleTypeDef* huart, uint8_t* pData, uint32_t Size)
{
  uint32_t tmp1 = 0;

  tmp1 = huart->RxState;
	
	if (tmp1 == HAL_UART_STATE_READY)
	{
		if ((pData == NULL) || (Size == 0))
		{
			return HAL_ERROR;
		}

		huart->pRxBuffPtr = pData;
		huart->RxXferSize = Size;
		huart->ErrorCode  = HAL_UART_ERROR_NONE;

		/* Enable the DMA Stream */
		HAL_DMA_Start(huart->hdmarx, (uint32_t)&huart->Instance->DR, (uint32_t)pData, Size);

		/* 
		 * Enable the DMA transfer for the receiver request by setting the DMAR bit
		 * in the UART CR3 register 
		 */
		SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);

		return HAL_OK;
	}
	else
	{
		return HAL_BUSY;
	}
}

/**
  * @brief      returns the number of remaining data units in the current DMAy Streamx transfer.
  * @param[in]  dma_stream: where y can be 1 or 2 to select the DMA and x can be 0
  *             to 7 to select the DMA Stream.
  * @retval     The number of remaining data units in the current DMAy Streamx transfer.
  */
uint16_t dma_current_data_counter(DMA_Stream_TypeDef *dma_stream)
{
  /* Return the number of remaining data units for DMAy Streamx */
  return ((uint16_t)(dma_stream->NDTR));
}



/**
 RC数据处理函数
  */
void rc_callback_handler(rc_info_t *_rc, uint8_t *buff)
{
    _rc->rc.ch1 = (buff[0] | buff[1] << 8) & 0x07FF;
    _rc->rc.ch1 -= 1024;
    _rc->rc.ch2 = (buff[1] >> 3 | buff[2] << 5) & 0x07FF;
    _rc->rc.ch2 -= 1024;
    _rc->rc.ch3 = (buff[2] >> 6 | buff[3] << 2 | buff[4] << 10) & 0x07FF;
    _rc->rc.ch3 -= 1024;
    _rc->rc.ch4 = (buff[4] >> 1 | buff[5] << 7) & 0x07FF;
    _rc->rc.ch4 -= 1024;
    _rc->rc.sw1 = ((buff[5] >> 4) & 0x000C) >> 2;
    _rc->rc.sw2 = (buff[5] >> 4) & 0x0003;
  
    if ((abs(_rc->rc.ch1) > 660) || \
        (abs(_rc->rc.ch2) > 660) || \
        (abs(_rc->rc.ch3) > 660) || \
        (abs(_rc->rc.ch4) > 660))
    {
        memset(_rc, 0, sizeof(rc_info_t));
    }	

    _rc->mouse.x = buff[6] | (buff[7] << 8); // x axis
    _rc->mouse.y = buff[8] | (buff[9] << 8);
    _rc->mouse.z = buff[10] | (buff[11] << 8);

    _rc->mouse.l = buff[12];
    _rc->mouse.r = buff[13];

    _rc->kb.key_code = buff[14] | buff[15] << 8; // key borad code
    _rc->wheel = (buff[16] | buff[17] << 8) - 1024;
}

/**
  中断处理函数
  */
static void uart_rx_idle_callback(UART_HandleTypeDef* huart)
{
	/* clear idle it flag avoid idle interrupt all the time */
	__HAL_UART_CLEAR_IDLEFLAG(huart);

	/* handle received data in idle interrupt */
	if (huart == &DBUS_HUART)
	{
		/* clear DMA transfer complete flag */
		__HAL_DMA_DISABLE(huart->hdmarx);

		/* handle dbus data dbus_buf from DMA */
		if ((DBUS_MAX_LEN - dma_current_data_counter(huart->hdmarx->Instance)) == DBUS_BUFLEN)
		{
			rc_callback_handler(&rc_t, dbus_buf);	
		}
		
		/* restart dma transmission */
		__HAL_DMA_SET_COUNTER(huart->hdmarx, DBUS_MAX_LEN);
		__HAL_DMA_ENABLE(huart->hdmarx);
	}
}

/**
  UART中断时进入这个函数，判断是否已经接收完数据 
  */
void uart_receive_handler(UART_HandleTypeDef *huart)
{  
	if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) && 
			__HAL_UART_GET_IT_SOURCE(huart, UART_IT_IDLE))
	{
		uart_rx_idle_callback(huart);
	}
}

/**
  初始化  
  */
void dbus_uart_init(void)
{
	/* open uart idle it */
	__HAL_UART_CLEAR_IDLEFLAG(&DBUS_HUART);
	__HAL_UART_ENABLE_IT(&DBUS_HUART, UART_IT_IDLE);

	uart_receive_dma_no_it(&DBUS_HUART, dbus_buf, DBUS_MAX_LEN);
}





