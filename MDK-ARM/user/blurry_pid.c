#include "blurry_pid.h"

/*
 * @Author: wukai 
 * @Date: 2020-02-11 9:32:56 
 * @Last Modified by:   wukai 
 * @Last Modified time: 2020-02-13 18:18:14 
 */


      


/*ʵ������ֵ��ģ�������������������Լ����������Ⱥ���*/

/*����������������,����{-6��-5��-4��-3��-2��-1��0��1��2��3��4��5��6}*/

 /*
  *pv �ǵ�ǰֵ���ת��
  * qValue ���e��er������ֵ
  */
 void LinearQuantization(FUZZYPID *vPID,float pv,float *qValue)  

{

  float thisError;

  float deltaError;

 

 thisError=vPID->setpoint-pv;                  //����ƫ��ֵ

 deltaError=thisError-vPID->lasterror;         //����ƫ������

 

 qValue[0]=6.0*thisError/(vPID->maximum-vPID->minimum);

 qValue[1]=3.0*deltaError/(vPID->maximum-vPID->minimum);

}

//������������ʵ���Ͽɸ�����Ҫ���ò�ͬ�ĺ��������ٽ����ߺ�������̫�ֲ������Լ�����һԪ�����ȶ��ǿ��Եģ������������������ʵ��Ŀ��ʹ�����Ժ������㹻�ˡ����������Ⱥ���Ҳ��һ���ж���ѡ������������ü�������С�����������Ⱥ�����

/*�����ȼ��㺯��*/
/*qv = qValue
 *ms[0] 
 */

void CalcMembership(float *ms,float qv,int * index)

{

 if((qv>=-NL)&&(qv<-NM))  

  {

    index[0]=0;

    index[1]=0;

    ms[0]=-0.5*qv-2.0;  //y=-0.5x-2.0

    ms[1]=0.5*qv+3.0;   //y=0.5x+3.0

  }

  else if((qv>=-NM)&&(qv<-NS))
  
  {
  
    index[0]=1;

    index[1]=1;

    ms[0]=-0.5*qv-1.0;  //y=-0.5x-1.0

    ms[1]=0.5*qv+2.0;   //y=0.5x+2.0

  } 

  else if((qv>=-NS)&&(qv<ZO-0.5))

  {

    index[0]=2;

    index[1]=2;

    ms[0]=-0.5*qv;      //y=-0.5x

    ms[1]=0.5*qv+1.0;   //y=0.5x+1.0

  }

  else if((qv>=ZO-0.5)&&(qv<ZO+0.5))

  {

    index[0]=3;

    index[1]=3;
  if((qv>=ZO-0.5)&&(qv<ZO))
  {
    
      ms[0]=-0.5*qv;      //y=-0.5x

      ms[1]=0.5*qv+1.0;   //y=0.5x+1.0
  }
  else
  {
    ms[0]=-0.5*qv+1.0;  //y=-0.5x+1.0

    ms[1]=0.5*qv;       //y=0.5x
  }

  }


  else if((qv>=ZO+0.5)&&(qv<PS))

  {

    index[0]=4;

    index[1]=4;

    ms[0]=-0.5*qv+1.0;  //y=-0.5x+1.0

    ms[1]=0.5*qv;       //y=0.5x

  }

  else if((qv>=PS)&&(qv<PM))

  {

    index[0]=5;

    index[1]=5;

    ms[0]=-0.5*qv+2.0;  //y=-0.5x+2.0

    ms[1]=0.5*qv-1.0;   //y=0.5x-1.0

  }

  else if((qv>=PM)&&(qv<=PL))

  {

    index[0]=6;

    index[1]=6;

    ms[0]=-0.5*qv+3.0;  //y=-0.5x+3.0

    ms[1]=0.5*qv-2.0;   //y=0.5x-2.0

  }

}

//kp ki kd�����Ľ���
/*static void ruleinit(int *ruleKp,int *ruleKi,int *ruleKd)
{
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
}*/

//ƫ�������������ֵת����ec�ľ�������
float LinearRealization(float maxkpar,float minkpar,float qValuek)
{
  float e_change=0;//ûɶ��
  float ec_change=0;

  ec_change = qValuek*(maxkpar -minkpar)/3;

  return ec_change;
}
//ģ������ĺ�����ʵ��

/*��ģ��������,���ݾ�������������������Ⱥ�������*/
/*
 *  ->pv���ת��
 * *deltaK->��ű��μ����kp ki kd������
 */ 
 void FuzzyComputation (FUZZYPID *vPID,float pv,float *deltaK)

{

  float qValue[2]={0,0};        //ƫ�������������ֵ

  int indexE[2]={0,0};          //ƫ������������

  float msE[2]={0,0};           //ƫ��������

  int indexEC[2]={0,0};         //ƫ����������������

  float msEC[2]={0,0};          //ƫ������������

  float qValueK[3];
  


  LinearQuantization(vPID,pv,qValue);  //��ʼģ����

  CalcMembership(msE,qValue[0],indexE);   //�õ�������

  CalcMembership(msEC,qValue[1],indexEC);  //�õ�������


 qValueK[0]=msE[0]*(msEC[0]*ruleKp[indexE[0]][indexEC[0]]+msEC[1]*ruleKp[indexE[0]][indexEC[1]])  //���������������ֵ

           +msE[1]*(msEC[0]*ruleKp[indexE[1]][indexEC[0]]+msEC[1]*ruleKp[indexE[1]][indexEC[1]]);

 qValueK[1]=msE[0]*(msEC[0]*ruleKi[indexE[0]][indexEC[0]]+msEC[1]*ruleKi[indexE[0]][indexEC[1]])

           +msE[1]*(msEC[0]*ruleKi[indexE[1]][indexEC[0]]+msEC[1]*ruleKi[indexE[1]][indexEC[1]]);

 qValueK[2]=msE[0]*(msEC[0]*ruleKd[indexE[0]][indexEC[0]]+msEC[1]*ruleKd[indexE[0]][indexEC[1]])

            +msE[1]*(msEC[0]*ruleKd[indexE[1]][indexEC[0]]+msEC[1]*ruleKd[indexE[1]][indexEC[1]]);

 

 deltaK[0]=LinearRealization(vPID->maxdKp,vPID->mindKp,qValueK[0]);

 deltaK[1]=LinearRealization(vPID->maxdKi,vPID->mindKi,qValueK[1]);

  deltaK[2]=LinearRealization(vPID->maxdKd,vPID->mindKd,qValueK[2]);

}

//int main()
//{

//  
//  FUZZYPID *vPID = (FUZZYPID*)malloc(sizeof(vPID));
//  float deltak[3]; // kp ki kd �ı仯�������Ŷ���������������������

//  //ruleinit(ruleKp,ruleKi,ruleKd);
//  FuzzyComputation(vPID,5000.0,deltak);
//   
//}

/*
  * @brief  ����pidʵ��
  * @param  processValue ���ʵ��ת��
  * @retval void
  * @attention ���ģ���������õ�����������
  */
void PIDRegulation(PID *vPID, float processValue)

{

  float thisError;

  float increment;

  float pError,dError,iError;

 

 thisError=vPID->setpoint-processValue; //�õ�ƫ��ֵ

 pError=thisError-vPID->lasterror;

 iError=thisError;

 dError=thisError-2*(vPID->lasterror)+vPID->preerror;

 increment=vPID->proportiongain*pError+vPID->integralgain*iError+vPID->derivativegain*dError;   //��������

 

 vPID->preerror=vPID->lasterror; //���ƫ�������´�����

 vPID->lasterror=thisError;

 vPID->result+=increment;

}
void Pid_blurry_calte()
{
	
	
}