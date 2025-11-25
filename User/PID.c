#include "stm32f10x.h"                  // Device header
#include "PID.h"
#include <math.h>

void PID_Update(PID_t *p)
{
    p->Error2 = p->Error1;     
    p->Error1 = p->Error0;     
    p->Error0 = p->Target - p->Actual;  // 当前误差
    
    if (fabs(p->Error0)<p->DeadZone && fabs(p->DeadZone-0)>=0.1 )
	{
		p->Out=0;
	}
	else if (fabs(p->DeadZone-0)<=0.1 || fabs(p->Error0)>p->DeadZone)
	{
		float deltaOut = p->Kp * (p->Error0 - p->Error1)
             + p->Ki * p->Error0
             + p->Kd * (p->Error0 - 2*p->Error1 + p->Error2);
    
   		p->Out += deltaOut;
	}

    
    if (p->Out > p->OutMax) {p->Out = p->OutMax;}
    if (p->Out < p->OutMin) {p->Out = p->OutMin;}
}
