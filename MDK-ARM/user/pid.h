#ifndef __pid_H
#define __pid_H
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "system.h"

enum{

    
    POSITION_PID=0,
    DELTA_PID=1,
};

enum
{
    kp=0,
    ki=1,
    kd=2,
};

typedef struct __pid_t
{
    float p;
    float i;
    float d;
    
    float set[2];				//目标值,包含NOW， LAST， LLAST上上次
    float get[2];				//测量值
    float err[2];				//误差
	
    
    float pout;							//p输出
    float iout;							//i输出
    float dout;							//d输出
    
    float pos_out;						//本次位置式输出
//    float last_pos_out;				    //上次输出
//    float delta_u;						//本次增量值
//    float delta_out;					//本次增量式输出 = last_delta_out + delta_u
//    float last_delta_out;
    
//	  float max_err;
//	  float deadband;				//err < deadband return
//    uint32_t pid_mode;
    uint32_t MaxOutput;				//输出限幅
    uint32_t i_limit;		//积分限幅
//    uint32_t i_seperate;
    
    int update_flag;
    
}pid_t;
void pid_param_update(pid_t	*pid, float kp, float ki, float kd,float i_max,float out_max);
typedef struct _angle_pid_struct_
{
	pid_t pid_position; //外环	
	pid_t pid_speed;
	
	int update_flag;  //参数是否更新flag 调参用
}Cascade_pid_t; //串级PID

void PID_struct_init(
     pid_t* pid,
    uint32_t mode,
    float 	kp, 
    float 	ki, 
    float 	kd,
    uint32_t maxout,
    uint32_t intergral_limit);
void Cascade_pid_init(Cascade_pid_t *pid,
              float position_kp,float position_ki,float position_kd,
              float position_i_max,float position_out_max,
			  float speed_kp,float speed_ki,float speed_kd,
              float speed_i_max,float speed_out_max); 
    
float pid_calc(pid_t* pid, float get, float set);
                            
float Cascade_pid_calc(Cascade_pid_t *pid,float target_position,float measure_position,float measure_speed,_Bool _choose);

extern pid_t pid_spd[4];
extern float test_v_in;

#endif


