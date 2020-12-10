#ifndef RLPSIDETECT
#define RLPSIDETECT

#include "motorcontrol.h"
#include "dfilt.h"
#include "pi.h"
#include "VectCalc.h"
#include "defs.h"
#include "stm32f3xx.h" 
#include "mymath.h"
#include "arm_math.h"
#include "arm_common_tables.h"


#define NMEAS 					200  // period for measuring os phase resistance in PWM tacts
#define MEASOMEG 				0.00125f//


int RLPsicalculateResistance(TPIParm* PIc, TStartVar* SVar);
void RLPsiInitFiltr(void);
void RLPsiSetResistance(TMotorParam* MotorParam);
void RLPsiResetCounter(void);

#endif
