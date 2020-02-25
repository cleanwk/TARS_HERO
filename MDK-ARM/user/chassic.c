#include "bsp_uart.h"


float  traget_speed[4]={0};
void test_horizontal(void)
{
	traget_speed[1]=rc_t.rc.ch2 ;   //���2
	traget_speed[0]=-rc_t.rc.ch2 ;   //���1
	traget_speed[2]=rc_t.rc.ch2 ;  //���3
	traget_speed[3]=-rc_t.rc.ch2 ;  //���4
	//traget_speed[x] = RC_Info.chx / 660 * MAX_CHASSIS_VX_SPEED;
}
void test_vertical(void)
{
	traget_speed[1]=rc_t.rc.ch1 ;   //���2
	traget_speed[0]=rc_t.rc.ch1 ;   //���1
	traget_speed[2]=-rc_t.rc.ch1 ;  //���3
	traget_speed[3]=-rc_t.rc.ch1 ;  //���4
}
void test_rotate(void)
{
	traget_speed[1]=rc_t.rc.ch3 ;   //���2
	traget_speed[0]=rc_t.rc.ch3 ;   //���1
	traget_speed[2]=rc_t.rc.ch3 ;  //���3
	traget_speed[3]=rc_t.rc.ch3 ;  //���4
}
void test_chassis_speed(void)
{
	traget_speed[1]=rc_t.rc.ch2 + rc_t.rc.ch1 + rc_t.rc.ch3;   //���2
	traget_speed[0]=-rc_t.rc.ch2 + rc_t.rc.ch1 + rc_t.rc.ch3;   //���1
	traget_speed[2]=rc_t.rc.ch2 - rc_t.rc.ch1 + rc_t.rc.ch3;  //���3
	traget_speed[3]=-rc_t.rc.ch2 - rc_t.rc.ch1 + rc_t.rc.ch3 ;  //���4

}	
