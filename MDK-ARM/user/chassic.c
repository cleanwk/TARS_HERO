#include "bsp_uart.h"


float  traget_speed[4]={0};
void test_horizontal(void)
{
	traget_speed[1]=rc_t.rc.ch2 ;   //电机2
	traget_speed[0]=-rc_t.rc.ch2 ;   //电机1
	traget_speed[2]=rc_t.rc.ch2 ;  //电机3
	traget_speed[3]=-rc_t.rc.ch2 ;  //电机4
	//traget_speed[x] = RC_Info.chx / 660 * MAX_CHASSIS_VX_SPEED;
}
void test_vertical(void)
{
	traget_speed[1]=rc_t.rc.ch1 ;   //电机2
	traget_speed[0]=rc_t.rc.ch1 ;   //电机1
	traget_speed[2]=-rc_t.rc.ch1 ;  //电机3
	traget_speed[3]=-rc_t.rc.ch1 ;  //电机4
}
void test_rotate(void)
{
	traget_speed[1]=rc_t.rc.ch3 ;   //电机2
	traget_speed[0]=rc_t.rc.ch3 ;   //电机1
	traget_speed[2]=rc_t.rc.ch3 ;  //电机3
	traget_speed[3]=rc_t.rc.ch3 ;  //电机4
}
void test_chassis_speed(void)
{
	traget_speed[1]=rc_t.rc.ch2 + rc_t.rc.ch1 + rc_t.rc.ch3;   //电机2
	traget_speed[0]=-rc_t.rc.ch2 + rc_t.rc.ch1 + rc_t.rc.ch3;   //电机1
	traget_speed[2]=rc_t.rc.ch2 - rc_t.rc.ch1 + rc_t.rc.ch3;  //电机3
	traget_speed[3]=-rc_t.rc.ch2 - rc_t.rc.ch1 + rc_t.rc.ch3 ;  //电机4

}	
