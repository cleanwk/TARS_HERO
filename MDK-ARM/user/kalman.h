#ifndef _KALMAN_H
#define _KALMAN_H


typedef struct {
    float X_last; //��һʱ�̵����Ž��  X(k-|k-1)
    float X_pre;  //��ǰʱ�̵�Ԥ����  X(k|k-1)
    float X_now;  //��ǰʱ�̵����Ž��  X(k|k)
    float P_pre;  //��ǰʱ��Ԥ������Э����  P(k|k-1)
    float P_now;  //��ǰʱ�����Ž����Э����  P(k|k)
    float P_last; //��һʱ�����Ž����Э����  P(k-1|k-1)
    float kg;     //kalman����

    float Q;
    float R;

}extKalman_t;

void KalmanCreate(extKalman_t *p,float T_Q,float T_R);
float KalmanFilter(extKalman_t* p,float dat);

#endif
