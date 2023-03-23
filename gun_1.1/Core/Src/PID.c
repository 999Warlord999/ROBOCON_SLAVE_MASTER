#include "PID.h"




void PID_cal(PID_Param *pid, int *pwm)
{
    double alpha;
	e = setVal - currVal;
	uP = kp*e;
	
	uI = uI_P + ki*e*deltaT;
	uI = uI > uI_HLim ? uI_HLim : 
	    (uI < uI_LLim ? uI_LLim : uI);
	    
	uD = kd*(e - e_p)/deltaT;
	uDF = (1-alpha)*uDF_P+alpha*uD;
	
	e_p = e;
	uI_P = uI;
	uDF_P = uDF;
	
	uCurr = uP+uI+uDF;
	uCurr =  uCurr > u_HLim ? u_HLim : 
	   		(uCurr < u_LLim ? u_LLim : uCurr);
	*pwm = (int)abs(uCurr);
}