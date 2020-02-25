#ifndef _MAGAZINE_H
#define _MAGAZINE_H
//弹仓开关角度对应的PWM值
#include "system.h"
extern uint8_t Magazine_opened_flag;
void Magazine_Ctrl(void);
void Magazine_init(void);
void Magazine_Servo(uint8_t flag);
void Magazine_StopCtrl(void);
bool Magezine_Rc_Switch(void);
void Magazine_Key_Ctrl(void);

#endif
