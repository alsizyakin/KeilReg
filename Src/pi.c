#include "pi.h"


/************************************************************************
InitPI - function to init the PI Controller
	tPIParam 	//see definition in pi.h
	Kp		//Proportional Gain		 
    Ki		//Integral Gain
    Kc		//Anti-windup Gain
    max 	//PI Output maximum limit
    min 	//PI Output minimum limit
	out		//PI initial output
************************************************************************/
void initPI(TPIParm *pParm, float kp, float ki, float kc, float max, float min, float out)
{
    pParm->qdSum = 0;
    pParm->qKp = kp;
    pParm->qKi = ki;
    pParm->qKc = kc;
    pParm->qOutMax = max;
    pParm->qOutMin = min;
    pParm->qOut = out;

}

/* CalcPI - function to calculate the output of the PI */
void calculatePI( TPIParm *pParm)  {
 /*
;    Error  = Reference - Measurement
;    U  = Sum + Kp * Error
;    if( U > Outmax )
;        Out = Outmax
;    else if( U < OutMin )
;        Out = OutMin
;    else
;        Out = U
;    Exc = U - Out
;    Sum = Sum + Ki * Err - Kc * Exc
*/
    float currentError;
    float U;
    float  outTemp;

    //Error  = Reference - Measurement
    currentError = pParm->qInRef - pParm->qInMeas;

    //U  = Sum + Kp * Error
    U = currentError * pParm->qKp;
    U = U + pParm->qdSum;
    //limit the output between the allowed limits
    //pParm->qOut is the PI output
    outTemp =  (U);
    if(outTemp >  pParm->qOutMax){
        pParm->qOut=  pParm->qOutMax;
	}else if(outTemp < pParm->qOutMin){
		pParm->qOut =  pParm->qOutMin;
	}else{
        pParm->qOut = U;
	}

    //U = Ki * Err
    U = currentError * pParm->qKi;

    //compute the difference between the limited and not limited output
    //currentError is used as a temporary variable
    currentError = outTemp - pParm->qOut;

    //U = U - Kc * Err = Ki * Err - Kc * Exc
    U -= currentError * pParm->qKc;

    //Sum = Sum + U = Sum + Ki * Err - Kc * Exc
    pParm->qdSum = pParm->qdSum + U;
    
}

// PI-controller with clamping saturation

void initPIclamp(TPIParm *pParm, float kp, float ki, float kc, float max, float min, float out)
{
    pParm->qdSum = 0;
    pParm->qKp = kp;
    pParm->qKi = ki;
    pParm->qKc = 0;    //Here Kc is used for previous error value. This could help to achieve more accurate integration 
    pParm->qOutMax = max;
    pParm->qOutMin = min;
    pParm->qOut = out;

}

/* CalcPI - function to calculate the output of the PI */
void calculatePIclamp( TPIParm *pParm)  {
    float currentError;
    float U,Usum;
    float  derror;

    //Error  = Reference - Measurement
    currentError = pParm->qInRef - pParm->qInMeas;

    //U  = Sum + Kp * Error
    U = currentError * pParm->qKp;
        
    derror = (currentError + pParm->qKc)*0.5F*pParm->qKi + pParm->qdSum;
    pParm->qKc = currentError;
    
    Usum = U + derror;

    //limit the output between the allowed limits
    //pParm->qOut is the PI output
    
    if((Usum > pParm->qOutMax)){
        pParm->qOut =  pParm->qOutMax;
        if (U < pParm->qOutMax){
            pParm->qdSum = pParm->qOutMax - U; 
        }
        
    }else if((Usum < pParm->qOutMin)){
        pParm->qOut =  pParm->qOutMin;
        if (U > pParm->qOutMin){
            pParm->qdSum = pParm->qOutMin - U;
        }
        
    }else{
        pParm->qOut = Usum;
        pParm->qdSum = derror;
    }
   
    
}


