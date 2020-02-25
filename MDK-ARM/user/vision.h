#ifndef _VISION_H
#define _VISION_H

#include "system.h"

#define    VISION_OFF         		(0x00)
#define    VISION_RED           	(0x01)
#define    VISION_BLUE          	(0x02)
#define    VISION_RBUFF_ANTI   	 	(0x03)//������
#define    VISION_BBUFF_ANTI   		(0x04)//������
#define    VISION_RBUFF_CLOCKWISE   (0x05)//��˳���
#define    VISION_BBUFF_CLOCKWISE   (0x06)//��˳���
#define    VISION_RBUFF_STAND   	(0x07)//��С��
#define    VISION_BBUFF_STAND   	(0x08)//��С��


#define ATTACK_NONE    0	//��ʶ��
#define ATTACK_RED     1	//ʶ��췽
#define ATTACK_BLUE    2	//ʶ������

//STM32����,ֱ�ӽ����ڽ��յ������ݿ������ṹ��
typedef __packed struct
{

	/* ���� */
	float     pitch_angle;
	float     yaw_angle;
	float     distance;			//����
	uint8_t   centre_lock;		//�Ƿ���׼�����м�  0û��  1��׼����
	uint8_t	  identify_target;	//��Ұ���Ƿ���Ŀ��/�Ƿ�ʶ����Ŀ��   0��  1��	
	uint8_t   identify_buff;	//���ʱ�Ƿ�ʶ����Ŀ�꣬1�ǣ�2ʶ���л���װ�ף�0ûʶ��

	uint8_t	  auto_too_close;   //Ŀ�����̫��,�Ӿ���1������0
     
	
}extVisionRecvData_t;


//STM32����,ֱ�ӽ�����õ�����һ���ֽ�һ���ֽڵط��ͳ�ȥ
typedef struct
{

	/* ���� */
	float     pitch_angle;
	float     yaw_angle;
	float     distance;			//����
	uint8_t   lock_sentry;		//�Ƿ���̧ͷʶ���ڱ�
	uint8_t   base;				//����
	

}extVisionSendData_t;

typedef enum
{
	VISION_MANU = 0,	//�ֶ�ģʽ
    VISION_BUFF = 1,	//��վ�Ų�������Ϣ����
    VISION_AUTO = 2,	//���ѿ���
}eVisionAction;

extern  eVisionAction           actVison;
extern  extVisionRecvData_t     VisionRecvData;    //�Ӿ����սṹ��
extern  extVisionSendData_t     VisionSendData;    //�Ӿ����ͽṹ��
extern  uint8_t Vision_Data_Flag;

void Vision_Ctrl(void);
void Vision_Auto_Attack_Ctrl(void);
void Vision_Auto_Attack_Off(void);

#endif
