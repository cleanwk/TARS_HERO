/**
 ***************************************(C) COPYRIGHT 2018 DJI***************************************
 * @file       bsp_uart.h
 * @brief      this file contains the common defines and functions prototypes for 
 *             the bsp_uart.c driver
 * @note         
 * @Version    V1.0.0
 * @Date       Jan-30-2018      
 ***************************************(C) COPYRIGHT 2018 DJI***************************************
 */

#ifndef __BSP_UART_H__
#define __BSP_UART_H__

#include "usart.h"

#define UART_RX_DMA_SIZE (1024)
#define DBUS_MAX_LEN     (50)
#define DBUS_BUFLEN      (18)
#define DBUS_HUART       huart1 
/* ----------------------- RC Switch Definition----------------------------- */

#define    RC_SW_UP              ((uint16_t)1)
#define    RC_SW_MID             ((uint16_t)3)
#define    RC_SW_DOWN            ((uint16_t)2)
/** 
  * @brief  remote control information
  */
typedef struct 
{
  /* rocker channel information */
    struct
    {
        int16_t ch1;
        int16_t ch2;
        int16_t ch3;
        int16_t ch4;
  /* left and right lever information */
        uint8_t sw1;
        uint8_t sw2;
    }rc;
  /* mouse movement and button information */
  struct
  {
    int16_t x;
    int16_t y;
    int16_t z;

    uint8_t l;
    uint8_t r;
  } mouse;
  /* keyboard key information */
  union {
    uint16_t key_code;
    struct
    {
      uint16_t W : 1;
      uint16_t S : 1;
      uint16_t A : 1;
      uint16_t D : 1;
      uint16_t SHIFT : 1;
      uint16_t CTRL : 1;
      uint16_t Q : 1;
      uint16_t E : 1;
      uint16_t R : 1;
      uint16_t F : 1;
      uint16_t G : 1;
      uint16_t Z : 1;
      uint16_t X : 1;
      uint16_t C : 1;
      uint16_t V : 1;
      uint16_t B : 1;
    } bit;
  } kb;
  int16_t wheel;
} rc_info_t;

/* 获取遥控器摇杆偏移值 
   RLR：右摇杆左右移动  LUD：左摇杆上下移动	*/
#define    RC_CH0_RLR_OFFSET    (rc_t.rc.ch0)
#define    RC_CH1_RUD_OFFSET  	(rc_t.rc.ch1)
#define    RC_CH2_LLR_OFFSET  	(rc_t.rc.ch2)
#define    RC_CH3_LUD_OFFSET  	(rc_t.rc.ch3)


/* 检测遥控器开关状态 */
#define    IF_RC_SW1_UP      (rc_t.rc.sw1 == RC_SW_UP)
#define    IF_RC_SW1_MID     (rc_t.rc.sw1 == RC_SW_MID)
#define    IF_RC_SW1_DOWN    (rc_t.rc.sw1 == RC_SW_DOWN)
#define    IF_RC_SW2_UP      (rc_t.rc.sw2 == RC_SW_UP)
#define    IF_RC_SW2_MID     (rc_t.rc.sw2 == RC_SW_MID)
#define    IF_RC_SW2_DOWN    (rc_t.rc.sw2 == RC_SW_DOWN)


/* 获取鼠标三轴的移动速度 */
#define    MOUSE_X_MOVE_SPEED    (rc_t.mouse.x)
#define    MOUSE_Y_MOVE_SPEED    (rc_t.mouse.y)
#define    MOUSE_Z_MOVE_SPEED    (rc_t.mouse.z)


/* 检测鼠标按键状态 
   按下为1，没按下为0*/
#define    IF_MOUSE_PRESSED_LEFT    (rc_t.mouse.l == 1)
#define    IF_MOUSE_PRESSED_RIGH    (rc_t.mouse.r == 1)

/* 检测键盘按键状态 
   若对应按键被按下，则逻辑表达式的值为1，否则为0 */
//#define    IF_KEY_PRESSED         (  rc_t.kb.bit  )
#define    IF_KEY_PRESSED_W       ( (rc_t.kb.bit.W)    != 0 )
#define    IF_KEY_PRESSED_S       ( (rc_t.kb.bit.S)    != 0 )
#define    IF_KEY_PRESSED_A       ( (rc_t.kb.bit.A)    != 0 )
#define    IF_KEY_PRESSED_D       ( (rc_t.kb.bit.D)    != 0 )
#define    IF_KEY_PRESSED_Q       ( (rc_t.kb.bit.Q)    != 0 )
#define    IF_KEY_PRESSED_E       ( (rc_t.kb.bit.E)    != 0 )
#define    IF_KEY_PRESSED_G       ( (rc_t.kb.bit.G)    != 0 )
#define    IF_KEY_PRESSED_X       ( (rc_t.kb.bit.X)    != 0 )
#define    IF_KEY_PRESSED_Z       ( (rc_t.kb.bit.Z)    != 0 )
#define    IF_KEY_PRESSED_C       ( (rc_t.kb.bit.C)    != 0 )
#define    IF_KEY_PRESSED_B       ( (rc_t.kb.bit.B)    != 0 )
#define    IF_KEY_PRESSED_V       ( (rc_t.kb.bit.V)    != 0 )
#define    IF_KEY_PRESSED_F       ( (rc_t.kb.bit.F)    != 0 )
#define    IF_KEY_PRESSED_R       ( (rc_t.kb.bit.R)    != 0 )
#define    IF_KEY_PRESSED_CTRL    ( (rc_t.kb.bit.CTRL) != 0 )
#define    IF_KEY_PRESSED_SHIFT   ( (rc_t.kb.bit.SHIFT) != 0 )


void uart_receive_handler(UART_HandleTypeDef *huart);
void dbus_uart_init(void);
void USART_SendData(USART_TypeDef* USARTx, uint16_t Data);
extern rc_info_t rc_t;
#endif

