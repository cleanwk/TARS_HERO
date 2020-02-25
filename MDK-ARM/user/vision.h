#ifndef _VISION_H
#define _VISION_H

#include "system.h"

#define    VISION_OFF         		(0x00)
#define    VISION_RED           	(0x01)
#define    VISION_BLUE          	(0x02)
#define    VISION_RBUFF_ANTI   	 	(0x03)//红逆大符
#define    VISION_BBUFF_ANTI   		(0x04)//蓝逆大符
#define    VISION_RBUFF_CLOCKWISE   (0x05)//红顺大符
#define    VISION_BBUFF_CLOCKWISE   (0x06)//蓝顺大符
#define    VISION_RBUFF_STAND   	(0x07)//红小符
#define    VISION_BBUFF_STAND   	(0x08)//蓝小符


#define ATTACK_NONE    0	//不识别
#define ATTACK_RED     1	//识别红方
#define ATTACK_BLUE    2	//识别蓝方

//STM32接收,直接将串口接收到的数据拷贝进结构体
typedef __packed struct
{

	/* 数据 */
	float     pitch_angle;
	float     yaw_angle;
	float     distance;			//距离
	uint8_t   centre_lock;		//是否瞄准到了中间  0没有  1瞄准到了
	uint8_t	  identify_target;	//视野内是否有目标/是否识别到了目标   0否  1是	
	uint8_t   identify_buff;	//打符时是否识别到了目标，1是，2识别到切换了装甲，0没识别到

	uint8_t	  auto_too_close;   //目标距离太近,视觉发1，否则发0
     
	
}extVisionRecvData_t;


//STM32发送,直接将打包好的数据一个字节一个字节地发送出去
typedef struct
{

	/* 数据 */
	float     pitch_angle;
	float     yaw_angle;
	float     distance;			//距离
	uint8_t   lock_sentry;		//是否在抬头识别哨兵
	uint8_t   base;				//吊射
	

}extVisionSendData_t;

typedef enum
{
	VISION_MANU = 0,	//手动模式
    VISION_BUFF = 1,	//它站着不动你屏息干嘛
    VISION_AUTO = 2,	//朋友开挂
}eVisionAction;

extern  eVisionAction           actVison;
extern  extVisionRecvData_t     VisionRecvData;    //视觉接收结构体
extern  extVisionSendData_t     VisionSendData;    //视觉发送结构体
extern  uint8_t Vision_Data_Flag;

void Vision_Ctrl(void);
void Vision_Auto_Attack_Ctrl(void);
void Vision_Auto_Attack_Off(void);

#endif
