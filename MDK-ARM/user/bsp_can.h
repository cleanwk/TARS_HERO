#ifndef _BSP_CAN_H_
#define _BSP_CAN_H_
#include "system.h"


typedef struct can_std_msg
{
  uint32_t std_id;
  uint8_t data[8];
}CAN_MsgTypeDef;

/**
  * @brief  CAN发送或是接收的ID
  */
typedef enum
{
    MOTOR_M3508 =0x200,
    M3508_A=0x201,  
    M3508_B=0x202,
    M3508_C=0x203,
    M3508_D=0x204,
	  M3508_E=0x206,
    M3508_F=0x207,
    MOTOR_GM6020=0x2FF,
    GM6020_YAW=0x209,
    GM6020_PITCH=0x20A,
    M2006_ID=0x205,
    CAN_RM_Rx1=0X01,
    CAN_RM_Rx2=0X02,
    CAN_RM_Rx3=0X03,
    GIMBAL_DATA=0x10
    
} CAN_Message_ID;


typedef struct 
{		
		uint16_t can_id;
		int16_t  set_voltage;
		uint16_t rotor_angle; //0-8191对应0-360 ，
		int16_t rotor_speed;  //转速
		int16_t torque_current;  //电流  -16384-16384对应+-20A
		uint8_t temp;   //温度  bit6
}moto_handle;






extern void drv_can_init(void);

void Chassis_set_voltage(int16_t v1, int16_t v2, int16_t v3, int16_t v4);
void Gimbal_set_voltage(int16_t v1, int16_t v2);
void Shoot_set_voltage(int16_t v);
void Friction_set_voltage(int16_t v1, int16_t v2, int16_t v3, int16_t v4);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan);


//变量声明
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

extern moto_handle M3508[4];
extern moto_handle GM6020[2];
extern moto_handle M2006;

#endif
