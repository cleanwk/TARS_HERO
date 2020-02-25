#ifndef __BLURRY_PID_H
#define __BLURRY_PID_H
#define NL -6
#define NM -4
#define NS -2
#define ZO 0
#define PS 2
#define PM 4
#define PL 6
//负大[NL]、负中[NM]、负小[NS]、零[ZO]、正小[PS]、正中[PM]、正大[PL]

int ruleKp[7][7]={{PL,PL,PM,PM,PS,ZO,ZO},
                        {PL,PL,PM,PS,PS,ZO,NS},
                        {PM,PM,PM,PS,ZO,NS,NS},
                        {PM,PM,PS,ZO,NS,NM,NM},
                        {PS,PS,ZO,NS,NS,NM,NM},
                        {PS,ZO,NS,NM,NM,NM,NL},
                        {ZO,ZO,NM,NM,NM,NL,NL}};      
    int ruleKi[7][7]={{NL,NL,NM,NM,NS,ZO,ZO},
                        {NL,NL,NM,NS,NS,ZO,ZO},
                        {NL,NM,NS,NS,ZO,PS,PS},
                        {NM,NM,NS,ZO,PS,PM,PL},
                        {NM,NS,ZO,PS,PS,PM,PL},
                        {ZO,ZO,PS,PS,PM,PL,PL},
                        {ZO,ZO,PS,PM,PM,PL,PL}};      
    int ruleKd[7][7]={{PS,NS,NL,NL,NL,NM,PS},
                        {PS,NS,NL,NM,NM,NS,ZO},
                        {ZO,NS,NM,NM,NS,NS,ZO},
                        {ZO,NS,NS,NS,NS,NS,ZO},
                        {ZO,ZO,ZO,ZO,ZO,ZO,ZO},
                        {PL,NS,PS,PS,PS,PS,PL},
                        {PL,PM,PM,PM,PS,PS,PL}};   
	
typedef struct
{

  float setpoint;               /*设定目标转速值*/

  float kp;                     /*比例系数*/

  float ki;                     /*积分系数*/

  float kd;                     /*微分系数*/

  float lasterror;              /*前一拍偏差*/

  float preerror;               /*前两拍偏差*/

  float deadband;               /*死区*/

  float output;                 /*输出值*/

  float result;                 /*物理量输出值*/

  float maximum;                /*输出值的上限*/

  float minimum;                /*输出值的下限*/

 

  float maxdKp;                 /*Kp增量的最大限值*/

  float mindKp;                 /*Kp增量的最小限值*/

  float qKp;                    /*Kp增量的影响系数*/

  float maxdKi;                 /*Ki增量的最大限值*/

  float mindKi;                 /*Ki增量的最小限值*/

  float qKi;                    /*Ki增量的影响系数*/

  float maxdKd;                 /*Kd增量的最大限值*/

  float mindKd;                 /*Kd增量的最小限值*/

  float qKd;                    /*Kd增量的影响系数*/

}FUZZYPID;

typedef struct //增量型pid

{

  float setpoint;       //设定值

  float proportiongain;     //比例系数

  float integralgain;      //积分系数

  float derivativegain;    //微分系数

  float lasterror;     //前一拍偏差

  float preerror;     //前两拍偏差

  float deadband;     //死区

  float result; //输出值

}PID;

void LinearQuantization(FUZZYPID *vPID,float pv,float *qValue);
void CalcMembership(float *ms,float qv,int * index);
float LinearRealization(float maxkpar,float minkpar,float qValuek);
void FuzzyComputation (FUZZYPID *vPID,float pv,float *deltaK);

#endif
