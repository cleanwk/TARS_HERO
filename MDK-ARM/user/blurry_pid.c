#include "blurry_pid.h"

/*
 * @Author: wukai 
 * @Date: 2020-02-11 9:32:56 
 * @Last Modified by:   wukai 
 * @Last Modified time: 2020-02-13 18:18:14 
 */


      


/*实现输入值的模糊化。线性量化函数以及三角隶属度函数*/

/*线性量化操作函数,论域{-6，-5，-4，-3，-2，-1，0，1，2，3，4，5，6}*/

 /*
  *pv 是当前值电机转速
  * qValue 存放e和er计算后的值
  */
 void LinearQuantization(FUZZYPID *vPID,float pv,float *qValue)  

{

  float thisError;

  float deltaError;

 

 thisError=vPID->setpoint-pv;                  //计算偏差值

 deltaError=thisError-vPID->lasterror;         //计算偏差增量

 

 qValue[0]=6.0*thisError/(vPID->maximum-vPID->minimum);

 qValue[1]=3.0*deltaError/(vPID->maximum-vPID->minimum);

}

//对于量化函数实际上可根据需要采用不同的函数，如速降曲线函数，正太分布函数以及其它一元函数等都是可以的，但对于我们在这里的实现目标使用线性函数就足够了。还有隶属度函数也是一样有多种选择，我们这里采用计算量较小的三角隶属度函数：

/*隶属度计算函数*/
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

//kp ki kd规则库的建立
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

//偏差及其增量的量化值转换成ec的具体增量
float LinearRealization(float maxkpar,float minkpar,float qValuek)
{
  float e_change=0;//没啥用
  float ec_change=0;

  ec_change = qValuek*(maxkpar -minkpar)/3;

  return ec_change;
}
//模糊推理的函数的实现

/*解模糊化操作,根据具体的量化函数和隶属度函数调整*/
/*
 *  ->pv电机转速
 * *deltaK->存放本次计算出kp ki kd的增量
 */ 
 void FuzzyComputation (FUZZYPID *vPID,float pv,float *deltaK)

{

  float qValue[2]={0,0};        //偏差及其增量的量化值

  int indexE[2]={0,0};          //偏差隶属度索引

  float msE[2]={0,0};           //偏差隶属度

  int indexEC[2]={0,0};         //偏差增量隶属度索引

  float msEC[2]={0,0};          //偏差增量隶属度

  float qValueK[3];
  


  LinearQuantization(vPID,pv,qValue);  //开始模糊化

  CalcMembership(msE,qValue[0],indexE);   //得到隶属度

  CalcMembership(msEC,qValue[1],indexEC);  //得到隶属度


 qValueK[0]=msE[0]*(msEC[0]*ruleKp[indexE[0]][indexEC[0]]+msEC[1]*ruleKp[indexE[0]][indexEC[1]])  //计算输出量的量化值

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
//  float deltak[3]; // kp ki kd 的变化量，接着对三个参数进行整定即可

//  //ruleinit(ruleKp,ruleKi,ruleKd);
//  FuzzyComputation(vPID,5000.0,deltak);
//   
//}

/*
  * @brief  增量pid实现
  * @param  processValue 电机实际转速
  * @retval void
  * @attention 结合模糊控制器得到的增量来做
  */
void PIDRegulation(PID *vPID, float processValue)

{

  float thisError;

  float increment;

  float pError,dError,iError;

 

 thisError=vPID->setpoint-processValue; //得到偏差值

 pError=thisError-vPID->lasterror;

 iError=thisError;

 dError=thisError-2*(vPID->lasterror)+vPID->preerror;

 increment=vPID->proportiongain*pError+vPID->integralgain*iError+vPID->derivativegain*dError;   //增量计算

 

 vPID->preerror=vPID->lasterror; //存放偏差用于下次运算

 vPID->lasterror=thisError;

 vPID->result+=increment;

}
void Pid_blurry_calte()
{
	
	
}