#include "pid.h"
#include "mytype.h"
#include <math.h>


float ano_test[10];
int   error_yaw_position;
int   error_pitch_position;
int   error_yaw_speed;
int   error_pitch_speed;
float test_v_in;
void abs_limit(float *a, float ABS_MAX){
    if(*a > ABS_MAX)
        *a = ABS_MAX;
    if(*a < -ABS_MAX)
        *a = -ABS_MAX;
}
//初始化
void PID_struct_init(
    pid_t* pid,
    uint32_t mode,
    float 	kp, 
    float 	ki, 
    float 	kd,
    uint32_t maxout,
    uint32_t intergral_limit
   // uint32_t intergral_seperate,//积分分离算法阈值
)
{
    pid->i_limit = intergral_limit;
    pid->MaxOutput = maxout;
   // pid->pid_mode = mode;
   // pid->i_seperate=intergral_seperate;
    pid->p = kp;
    pid->i = ki;
    pid->d = kd;
	
}      


/***************************串级******************************/

/**
  * @brief  Cascade_pid_init
  * @param  Cascade_pid_t struct
    @param  target_position value
    @param  measure_position value
    @param  target_speed value
    @param  measure_speed value
  * @retval calculation result
  */
void Cascade_pid_init(Cascade_pid_t *pid,
              float position_kp,float position_ki,float position_kd,
              float position_i_max,float position_out_max,
							float speed_kp,float speed_ki,float speed_kd,
              float speed_i_max,float speed_out_max)
{
  pid->pid_position.p        = position_kp;
  pid->pid_position.i        = position_ki;
  pid->pid_position.d        = position_kd;
  pid->pid_position.i_limit     = position_i_max;
  pid->pid_position.MaxOutput   = position_out_max;
	
	pid->pid_speed .p         = speed_kp;
	pid->pid_speed .i         = speed_ki;
	pid->pid_speed .d         = speed_kd;
	pid->pid_speed .i_limit       = speed_i_max;
	pid->pid_speed .MaxOutput     = speed_out_max;
}
void Cascade_pid_param_update(Cascade_pid_t *pid,
              float position_kp,float position_ki,float position_kd,
              float position_i_max,float position_out_max,
							float speed_kp,float speed_ki,float speed_kd,
              float speed_i_max,float speed_out_max)
{
  pid->pid_position.p        = position_kp;
  pid->pid_position.i        = position_ki;
  pid->pid_position.d        = position_kd;
  pid->pid_position.i_limit     = position_i_max;
  pid->pid_position.MaxOutput   = position_out_max;
	
	pid->pid_speed .p         = speed_kp;
	pid->pid_speed .i          = speed_ki;
	pid->pid_speed .d          = speed_kd;
	pid->pid_speed .i_limit = speed_i_max;
	pid->pid_speed .MaxOutput     = speed_out_max;
}


/*中途更改参数设定(调参用)------------------------------------------------------------*/
void pid_param_update(pid_t	*pid, float kp, float ki, float kd,float i_max,float out_max)
{
    pid->p = kp;
    pid->i = ki;
    pid->d = kd;
    pid->i_limit=i_max;
    pid->MaxOutput=out_max;
}



/**
    *@bref. calculate delta PID and position PID
    *@param[in] set： target
    *@param[in] real	measure
    注释掉了增量式算法以及一些多余项
    */
float pid_calc(pid_t* pid, float get, float set){

    pid->get[0] = get;
    pid->set[0]= set;
    pid->err[0] = set - get;	//set - measure
//    if (pid->max_err != 0 && ABS(pid->err[NOW]) >  pid->max_err  )
//		return 0;
//	if (pid->deadband != 0 && ABS(pid->err[NOW]) < pid->deadband)
//		return 0;
    
//   if(pid->pid_mode == POSITION_PID) //位置式p
//    {
        pid->pout = pid->p * pid->err[0];
//        if(ABS(pid->err[NOW])<pid->i_seperate)//分离积分算法
        pid->iout += pid->i * pid->err[0];
        pid->dout = pid->d * (pid->err[0] - pid->err[1] );
        abs_limit(&(pid->iout), pid->i_limit);
        pid->pos_out = pid->pout + pid->iout + pid->dout;
        abs_limit(&(pid->pos_out), pid->MaxOutput);
//        pid->last_pos_out = pid->pos_out;	//update last time 
//    }
//    else if(pid->pid_mode == DELTA_PID)//增量式P
//    {
//        pid->pout = pid->p * (pid->err[NOW] - pid->err[LAST]);
//        pid->iout = pid->i * pid->err[NOW];
//        pid->dout = pid->d * (pid->err[NOW] - 2*pid->err[LAST] + pid->err[LLAST]);
//        
//        abs_limit(&(pid->iout), pid->i_limit);
//        pid->delta_u = pid->pout + pid->iout + pid->dout;
//        pid->delta_out = pid->last_delta_out + pid->delta_u;
//        abs_limit(&(pid->delta_out), pid->MaxOutput);
//        pid->last_delta_out = pid->delta_out;	//update last time
//    }
//    
//    pid->err[LLAST] = pid->err[LAST];
//    pid->err[LAST] = pid->err[NOW];
//    pid->get[LLAST] = pid->get[LAST];
//    pid->get[LAST] = pid->get[NOW];
//    pid->set[LLAST] = pid->set[LAST];
//    pid->set[LAST] = pid->set[NOW];
//      return pid->pid_mode==POSITION_PID ? pid->pos_out : pid->delta_out;
        return pid->pos_out;
}


float Cascade_pid_calc(Cascade_pid_t *pid,float target_position,float measure_position,float measure_speed,_Bool _choose)
{
//	//外环角度环
	pid->pid_position.set[0] = target_position;
	pid->pid_position.get[0] = measure_position;
	pid->pid_position.err [1] = pid->pid_position.err[0];	
    pid->pid_position.err [0] = target_position - measure_position;
    if(pid->pid_position.err [0]>=8191/2)
        pid->pid_position.err [0]-=8191;
    
    if(_choose)//调参时发送yaw/pitch的内环数据
    error_yaw_position = pid->pid_position .err [0];
    else
    error_pitch_position = pid->pid_position .err [0];
    
    //卡尔曼滤波消抖
    
    pid->pid_position .pout  = pid->pid_position .p  * pid->pid_position.err[0];
    pid->pid_position .iout += pid->pid_position .i  * pid->pid_position .err[0];
    pid->pid_position .dout  = pid->pid_position .d  *( pid->pid_position .err[0] - pid->pid_position.err[1]);
    abs_limit(&(pid->pid_position.iout ), pid->pid_position.i_limit );
  
    pid->pid_position .pos_out  = pid->pid_position .pout  + pid->pid_position .iout  + pid->pid_position .dout ;
    abs_limit(&(pid->pid_position.pos_out ), pid->pid_position.MaxOutput );	
    //内环速度环
	pid->pid_speed .set[0] =   pid->pid_position.pos_out * 0.003f;//符号
	pid->pid_speed .get[0] =   measure_speed;        //pid->pid_speed .ref ;
	
	test_v_in= pid->pid_position.pos_out * 0.003f;
	
	
	pid->pid_speed .err [1] = pid->pid_speed .err[0];
	pid->pid_speed .err [0] = pid->pid_speed.set[0] - pid->pid_speed .get[0];
	
    if(_choose)//调参时发送yaw/pitch的内环数据
    error_yaw_speed = pid->pid_speed .err [0];
    else
    error_pitch_speed = pid->pid_speed .err [0];   
	
	pid->pid_speed .pout = pid->pid_speed .p * pid->pid_speed.err[0];
	pid->pid_speed .iout += pid->pid_speed .i * pid->pid_speed.err [0];
	pid->pid_speed .dout =pid->pid_speed .d * (pid->pid_speed.err[0]-pid->pid_speed.err[1]);
	abs_limit(&(pid->pid_position.iout ), pid->pid_speed.i_limit );
	
	pid->pid_speed .pos_out  =pid->pid_speed .pout + pid->pid_speed .iout +pid->pid_speed .dout ;
	abs_limit(&(pid->pid_speed.pos_out ), pid->pid_position.MaxOutput );	
	
	return pid->pid_speed .pos_out;
	
}


pid_t pid_spd[4];//底盘电机PID

Cascade_pid_t pid_gimbal[2];//云台电机PID



