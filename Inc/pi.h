#ifndef PI_H
#define PI_H

//------------------  C API for PI routines ---------------------

typedef struct {
    float       qdSum;		        //Integrator sum; 
    float       qKp;			//Proportional Gain		 
    float       qKi;			//Integral Gain
    float       qKc;			//Anti-windup Gain
    float       qOutMax;		//PI Output maximum limit
    float       qOutMin;		//PI Output minimum limit
    float       qInRef; 		//Reference
    float       qInMeas;		//Measurement
    float       qOut;			//PI Output;
    } TPIParm;

/************************************************************************
InitPI - function to init the PI Controller
	tPIParam 	//see definition above
	Kp		//Proportional Gain		 
        Ki		//Integral Gain
        Kc		//Anti-windup Gain
        max 		//PI Output maximum limit
        min 	 	//PI Output minimum limit
	out		//PI initial output
************************************************************************/
void initPI(TPIParm *pParm,float kp,float ki,float kc,float max,float min,float out);
void initPIclamp(TPIParm *pParm,float kp,float ki,float kc,float max,float min,float out);	

/* calculatePI - function to calculate the output of the PI */
void calculatePI( TPIParm *pParm);
void calculatePIclamp( TPIParm *pParm);	

#endif

