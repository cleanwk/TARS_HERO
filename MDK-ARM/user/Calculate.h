#ifndef _CALCULATE_H
#define _CALCULATE_H
#include "stm32f4xx.h"
#include "arm_math.h"

#define ABS(x)		((x>0)? (x): (-x)) 

#define mat         arm_matrix_instance_f32 //float
#define mat_64      arm_matrix_instance_f64 //double
#define mat_init    arm_mat_init_f32
#define mat_add     arm_mat_add_f32
#define mat_sub     arm_mat_sub_f32
#define mat_mult    arm_mat_mult_f32
#define mat_trans   arm_mat_trans_f32//�������ת��
#define mat_inv     arm_mat_inverse_f32
#define mat_inv_f64 arm_mat_inverse_f64
//��ֵ�˲�
typedef struct 
{
    int tick;
    float value[10];
}mean_filter_t;


//һ��
typedef struct 
{
    float X_last; //��һʱ�̵����Ž��  X(k|k-1)
    float X_pre;  //��ǰʱ�̵�Ԥ����  X(k|k-1)
    float X_now;  //��ǰʱ�̵����Ž��  X(k|k)
    float P_pre;  //��ǰʱ��Ԥ������Э����  P(k|k-1)
    float P_now;  //��ǰʱ�����Ž����Э����  P(k|k)
    float P_last; //��һʱ�����Ž����Э����  P(k-1|k-1)
    float kg;     //kalman����

    float Q;
    float R;

}extKalman_t;

//����
typedef struct
{
  float raw_value;
  float filtered_value[2];
  mat xhat, xhatminus, z, A, H, AT, HT, Q, R, P, Pminus, K;
} kalman_filter_t;

typedef struct
{
  float raw_value;
  float filtered_value[2];
  float xhat_data[2], xhatminus_data[2], z_data[2],Pminus_data[4], K_data[4];
  float P_data[4];
  float AT_data[4], HT_data[4];
  float A_data[4];
  float H_data[4];
  float Q_data[4];
  float R_data[4];
} kalman_filter_init_t;



float RAMP_float( float final, float now, float ramp );
float RampInc_float( float *buffer, float now, float ramp );
float constrain_float(float amt, float low, float high);

float mean_fliter(mean_filter_t *x,float get);

void KalmanCreate(extKalman_t *p,float T_Q,float T_R);
float KalmanFilter(extKalman_t* p,float dat);
void   kalman_filter_init(kalman_filter_t *F, kalman_filter_init_t *I);
float *kalman_filter_calc(kalman_filter_t *F, float signal1, float signal2);


#endif
