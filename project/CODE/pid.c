#include "pid.h"

// pid处理函数
PID pidL; //速度环
PID pidR; //速度环
PID pidS; //方向环
PID pidW; //角度环

//增量式pid计算
void PIDRegulation(PID *vPID, float processValue)
{

    float thisError;

    float increment;

    float pError, dError, iError;

    thisError = vPID->setpoint - processValue; //?????

    pError = thisError - vPID->lasterror;

    iError = thisError;

    dError = thisError - 2 * (vPID->lasterror) + vPID->preerror;

    increment = vPID->proportiongain * pError + vPID->integralgain * iError + vPID->derivativegain * dError; //????

    vPID->preerror = vPID->lasterror; //??????????

    vPID->lasterror = thisError;

    vPID->result += increment;

    if (vPID->result > vPID->deadband)
        vPID->result = vPID->deadband;
    if (vPID->result < -vPID->deadband)
        vPID->result = -vPID->deadband;
}

//位置式pid计算
void PIDRegulation1(PID *vPID, float processValue)

{
    float thisError;

    thisError = processValue - vPID->setpoint;

    vPID->integral += thisError;

    vPID->result = vPID->proportiongain * thisError + vPID->integralgain * vPID->integral + vPID->derivativegain * (thisError - vPID->lasterror);

    vPID->lasterror = thisError;
}

//陀螺仪pid消抖
void PIDRegulation2(PID *vPID, float processValue)

{
    float thisError;

    thisError = processValue - vPID->setpoint;

    vPID->integral += thisError;

    vPID->result = vPID->proportiongain * thisError + vPID->integralgain * vPID->integral + vPID->derivativegain * (thisError - vPID->lasterror ) + vPID->Gkd * gyro;

    vPID->lasterror = thisError;
}

//设置速度环目标值
void setSpeedLR(int16 speed)
{
    pidL.setpoint = (float)speed;
    pidR.setpoint = (float)speed;
}
//设置速度环目标值
void setSpeedL(int16 speed)
{
    pidL.setpoint = (float)speed;
}
//设置速度环目标值
void setSpeedR(int16 speed)
{
    pidR.setpoint = (float)speed;
}
//设置速度环参数
void setPidL(int16 p, int16 i, int16 d, int16 deadband)
{
    pidL.proportiongain = p;
    pidL.integralgain = i;
    pidL.derivativegain = d;
    pidL.deadband = deadband;
}
//设置速度环参数
void setPidR(int16 p, int16 i, int16 d, int16 deadband)
{
    pidR.proportiongain = p;
    pidR.integralgain = i;
    pidR.derivativegain = d;
    pidR.deadband = deadband;
}
//设置速度环参数
void setPidLR(int16 p, int16 i, int16 d, int16 deadband)
{
    pidL.proportiongain = p;
    pidL.integralgain = i;
    pidL.derivativegain = d;
    pidR.proportiongain = p;
    pidR.integralgain = i;
    pidR.derivativegain = d;
    pidL.deadband = deadband;
    pidR.deadband = deadband;
}
//设置方向环参数
void setPidS(float p, float i, float d)
{
    pidS.proportiongain = p;
    pidS.integralgain = i;
    pidS.derivativegain = d;
}
//设置方向环参数(加陀螺仪)
void setPidS_2(float p, float i, float d, float gk)
{
    pidS.proportiongain = p;
    pidS.integralgain = i;
    pidS.derivativegain = d;
	pidS.Gkd = gk;
}
//设置角度环参数
void setPidW(float p, float i, float d)
{
    pidW.proportiongain = p;
    pidW.integralgain = i;
    pidW.derivativegain = d;
}