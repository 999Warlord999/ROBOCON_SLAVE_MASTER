/*
 * gunPID.c
 *
 *  Created on: Mar 27, 2023
 *      Author: SpiritBoi
 */

#include "gunPID.h"

int gunCalPID(PID_Param *pid)
{
	e = setVal - currVal;
	uP = kp*e;
	uI = uI_P + ki*e*deltaT;
	uI = uI > uI_HLim ? uI_HLim :
	(uI < uI_LLim ? uI_LLim : uI);
	e_p = e;
	uI_P = uI;
	uCurr = uP+uI;
	uCurr =  uCurr > u_HLim ? u_HLim :
			(uCurr < u_LLim ? u_LLim : uCurr);
	return uCurr;
}
