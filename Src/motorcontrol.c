#include "motorcontrol.h"
#include "dfilt.h"
#include "pi.h"
#include "VectCalc.h"
#include "defs.h"
#include "stm32f3xx.h" 
#include "mymath.h"
#include "arm_math.h"
#include "arm_common_tables.h"

float Timer1Period;

TMotorCurrents MotorCurrents; 
TMotorVoltage MotorVoltage; 

TFiltrParm  wFiltrParm;
extern TFiltrParm 	uFiltrParm;
TFiltrParm  rFiltrParm;

TMotorParam MotorParam_1, MotorParam_2; 
TMotorParam *MotorParam;

TMRASVar MRASVar; 
TPWMComp PWMComp;
TStartVar StartVar;
float  wsr, wTarg;
TFlags Flags;

uint32_t 	SwichTimes[3]={0, 0, 0};
float 		PulseTimes[3]={0, 0, 0};
// �� ����������
TPIParm uStartPI, wPI, idPI, iqPI;
float kiStart = 5.F, kpStart = 50.F, kiw = 7.5e-6, kpw = 1.6e-2, kiD = 5.F, kpD = 50.F;

float detected_R;
int res_detect_counter = 0;
float calc_Ra, calc_Rb, calc_Rc, calc_R;

void initVariables(){
	
	Timer1Period = (((float)SystemCoreClock / (float)FPWM)/2.F)-1.F;
	
	// Motor params
	MotorParam_1.R 			= 1.4F;
	MotorParam_1.psi 		= 0.0769;//0.082F;
	MotorParam_1.Ld 		= 2.5e-3F;
	MotorParam_1.Lq 		= 2.5e-3F;
	MotorParam_1.Ld_inv 	= 1.0F / MotorParam_1.Ld;	
	MotorParam_1.Lq_inv 	= 1.0F / MotorParam_1.Lq;
	MotorParam_1.iMax		= 11.5F;
	MotorParam_1.kiw		= 7.5e-6F;//7.5e-6F;
	MotorParam_1.kpw		= 0.01F;//1.6e-2F;
	MotorParam_1.kid		= 0.21F;
	MotorParam_1.kpd		= 3.75F;
	
	// ��������� ������� ����������
	uFiltrParm.index 	= 0;
	uFiltrParm.length 	= 4;
	uFiltrParm.sum   	= 0.F;
	
	// ��������� ������� ��������
#define WFILTRLEN 		512	
	wFiltrParm.index  	= 0;
	wFiltrParm.length 	= WFILTRLEN;
	wFiltrParm.sum    	= 0.F;
	
	// ��������� ������� R
	rFiltrParm.index 	= 0;
	rFiltrParm.length 	= 128;
	rFiltrParm.sum   	= 0.F;
	
	
	
	// ���������� ��� ������� ������
	StartVar.i 			= 0;
	StartVar.id			= 0;
	StartVar.iq			= 0;
	StartVar.didt 		= 100;
	StartVar.eps 		= 50;//50
	StartVar.w 			= 0;
	StartVar.wMax 		= 150;
	StartVar.angle 		= 0;
	StartVar.iMax 		= 8.F;
	StartVar.u 			= 0;
	
	// ���������� ��� ����������� �������� �������
	PWMComp.dt = 0;
	
	// ����
	MotorCurrents.idZad = 0.F;
	MotorCurrents.iqZad = 0.6F;
	
	// �� ����������
	kiStart = 0.21F; 
	kpStart = 3.75F; 
	
	initPI(&uStartPI, kpStart, kiStart, 1.F, 50.F, -50.F, 0.F);
	/*									      17     0		*/
	
	
	
	
	PWMComp.dt = DTF;
	PWMComp.uvd = 1.7F;
	PWMComp.uvt = 1.5F;
	
}

void reinitVariables(){

	MotorParam = &MotorParam_1;
	
	// ��������� ������� ��������
	wFiltrParm.index  	= 0;
	wFiltrParm.length 	= WFILTRLEN;
	wFiltrParm.sum    	= 0.F;
	
	// ���������� ��� ������� ������
	//StartVar.iMax 	= MotorParam->iMax;
	
	// ���������� ��� MRAS
	MRASVar.kim 	= 150000.F;
	MRASVar.kpm 	= 2.05F;
	MRASVar.tetm 	= 0;
	MRASVar.wr	 	= 0;
	MRASVar.sinm 	= 0;
	MRASVar.cosm	= 0;
	
	wsr = 0;
	
	// ����
	MotorCurrents.idZad = 0.F;
	MotorCurrents.iqZad = 0.6F;
	
	initPIclamp		(&wPI, 		MotorParam->kpw, 	MotorParam->kiw, 	1.F, 	StartVar.iMax,  	-1.F, 	0.F);
	initPI			(&idPI, 	MotorParam->kpd, 	MotorParam->kid, 	1.F, 	50.F, 				-50.F, 	0.F);
	initPI			(&iqPI, 	MotorParam->kpd, 	MotorParam->kid,	1.F, 	50.F, 				-50.F, 	0.F);	
	initPI			(&uStartPI, kpStart, kiStart, 1.F, 50.F, -50.F, 0.F);
	
	wTarg = 500.F;//1332.F;  // 212 Hz
	
}


void runFullyControlled(void){
	float angleSVPWM, uAlfa_temp, uBeta_temp, Torque;
	float temp;
	
	PWM_ON();

	//Rotor position estimation and other params
	calculateMRAS( &MotorCurrents, &MotorVoltage, MotorParam, &MRASVar, (float)PRPWM);

			
	MRASVar.sinm = arm_sin_f32(MRASVar.tetm);
	MRASVar.cosm = arm_cos_f32(MRASVar.tetm);
	
	temp = MRASVar.wr;
	wsr = sredN_f(temp, &wFiltrParm);
	//DAC->DHR12R1 = MRASVar.tetm * 600;
					
	//PI controller for speed calculation
	wPI.qInMeas = wsr;
	wPI.qInRef  = wTarg;  
	
	calculatePIclamp(&wPI);
					 
	MotorCurrents.iqZad = wPI.qOut;
	Torque = 4.5F * MotorCurrents.iqZad * MotorParam->psi;
	
	//�������������� �����
	transformFwdParkCurrent( &MotorCurrents, &MRASVar);
						
	/*---------��-���������� ���� �� ���������----------*/

	idPI.qInRef  = MotorCurrents.idZad;
	idPI.qInMeas = MotorCurrents.id;    
	calculatePI(&idPI);
	
	MotorVoltage.ucd = -wsr * MotorParam->Lq * MotorCurrents.iq;
	MotorVoltage.ucq = wsr * (MotorParam->psi + MotorParam->Ld * MotorCurrents.id);
					
	//ud voltage calculation (PI+ crossconnection compensation)
	MotorVoltage.ud = idPI.qOut + MotorVoltage.ucd;
					
	iqPI.qInRef  = MotorCurrents.iqZad;
	iqPI.qInMeas = MotorCurrents.iq;
	calculatePI(&iqPI);
					
	//Uq voltage calculation (PI+ crossconnection compensation)
	MotorVoltage.uq = iqPI.qOut + MotorVoltage.ucq;
	//MotorVoltage.uq = 20;
					
	temp = MotorVoltage.udc_izm * MotorVoltage.udc_izm * 0.30F - MotorVoltage.ud * MotorVoltage.ud;
						
			
	if(MotorVoltage.uref > 0){
		MotorVoltage.uref = 1.F/quickSqrt(temp);
	}else {
		MotorVoltage.uref = 0;
	}
	if(MotorVoltage.uq > MotorVoltage.uref){
		MotorVoltage.uq = MotorVoltage.uref;
	}
	
	//�������� �������������� �����
	transformRewParkVoltage( &MotorVoltage, &MRASVar);
				
	temp = MotorVoltage.uAlfa * MotorVoltage.uAlfa + MotorVoltage.uBeta * MotorVoltage.uBeta;
			
	//temp = MotorVoltage.uref; 
	MotorVoltage.uref = 1.F/quickSqrt(temp);
			 
	//���������� ���� �������� ���� ����� Ud � Uq � ������� ������� ���������
	uAlfa_temp = MotorVoltage.uAlfa;
	uBeta_temp = MotorVoltage.uBeta;
	angleSVPWM = atan_f(uAlfa_temp, uBeta_temp);  
			
	//----------------------------------------------------------------------------
	//������������ ������� ����������
	//----------------------------------------------------------------------------
	SVPWM_f(angleSVPWM, &MotorVoltage, PulseTimes);
	PWMcompensation(Timer1Period, PulseTimes, SwichTimes, &MotorCurrents, &PWMComp);
			
	//������ ��������, �����. � ������� ������� � �������� ��������� �������
	WRITE_REG(TIM1->CCR1, SwichTimes[0]);
	WRITE_REG(TIM1->CCR2, SwichTimes[1]);
	WRITE_REG(TIM1->CCR3, SwichTimes[2]);
	
	if ((wPI.qOutMax += 0.001F) > MotorParam->iMax){
		wPI.qOutMax = MotorParam->iMax;
		
	}
	
	if((wTarg += 0.025F) > 1332.F){
		wTarg = 1332.F;
	}
	
		 	
}




void startMotor(){
	
	float angleSVPWM;
	//static  //return detected_R, calc_R s here
	static int startingCounter = 0;
	
	
	PWM_ON();
	
//#define START_TEST
	
#ifdef START_TEST  //TEST of current sense for this configuration 
	if (startingCounter++ > 2500){
		StartVar.i = 2.F;
		if (startingCounter >5000){
			startingCounter = 0;
		}
	
	}else{
		StartVar.i = 12;
	}
#else	

		
	if(StartVar.i < StartVar.iMax){
		StartVar.i += StartVar.didt * (float)PRPWM;
	}else {
		StartVar.i = StartVar.iMax;
		if(Flags.MotorDetection == 0){
			reinitVariables();
			Flags.MotorDetection = 1;
			res_detect_counter = 0;
		}else if(res_detect_counter < NMEAS) {
			res_detect_counter++;
			calc_R = uStartPI.qOut / uStartPI.qInMeas;
			calc_Ra = sredN_f(calc_R, &rFiltrParm);
		}else if(res_detect_counter < 2 * NMEAS){
			StartVar.angle = M_2PI / 3.F;
			res_detect_counter++;
			calc_R = uStartPI.qOut / uStartPI.qInMeas;
			calc_Rb = sredN_f(calc_R, &rFiltrParm);
		}else if(res_detect_counter < 3 * NMEAS){
			StartVar.angle = M_2PI / 1.5F;
			res_detect_counter++;
			calc_R = uStartPI.qOut / uStartPI.qInMeas;
			calc_Rc = sredN_f(calc_R, &rFiltrParm);	
		}else{
			detected_R = (calc_Ra + calc_Rb + calc_Rc)/3.F;
			StartVar.w 	+= 	StartVar.eps * (float)PRPWM;
			StartVar.angle	+= 	StartVar.w * (float)PRPWM;
			Flags.MRASEnable = 1;
			if(StartVar.angle > M_2PI){
				StartVar.angle -= M_2PI; 
			}
		}
	}
	
	//equation for leaving STARTING_STATE
	if (StartVar.w > StartVar.wMax){
		StartVar.w = StartVar.wMax;
		if (startingCounter++ > 5){
			Flags.CurrentState = STATE_RUNNING;
			startingCounter = 0;
			//res_flag = 1;
		}
	}
			
#endif	/*	END of test	*/

	uStartPI.qOutMax =  (MotorVoltage.udc_izm - 3)*0.5747F;
	uStartPI.qInRef  =  StartVar.i;
	uStartPI.qInMeas =  1.F/quickSqrt(MotorCurrents.iAlfa * MotorCurrents.iAlfa + MotorCurrents.iBeta * MotorCurrents.iBeta);
	calculatePI(&uStartPI);
	
	MotorVoltage.uref = uStartPI.qOut;
			
	angleSVPWM = StartVar.angle;
			
	SVPWM_f(angleSVPWM, &MotorVoltage, PulseTimes);
			
	PWMcompensation(Timer1Period, PulseTimes, SwichTimes, &MotorCurrents, &PWMComp);
			
	//������ ��������, �����. � ������� ������� � �������� ��������� �������
	WRITE_REG(TIM1->CCR1, SwichTimes[0]);
	WRITE_REG(TIM1->CCR2, SwichTimes[1]);
	WRITE_REG(TIM1->CCR3, SwichTimes[2]);
			
	MotorVoltage.uAlfa = MotorVoltage.uref * arm_cos_f32(StartVar.angle);
	MotorVoltage.uBeta = MotorVoltage.uref * arm_sin_f32(StartVar.angle);
	
	if(Flags.MRASEnable){		
		calculateMRAS( &MotorCurrents, &MotorVoltage, MotorParam, &MRASVar, (float)PRPWM);
		transformFwdParkVoltage(&MotorVoltage, &MRASVar);
		StartVar.iq = MotorCurrents.idr;
		StartVar.id = MotorCurrents.iqr;
	}
  
}

void stopMotor(){
	
	PWM_OFF(); 
	NTC_RELAY_OFF();
	
	Flags.MotorDetection = 0;
	
	// ���������� ��� ������� ������
	StartVar.i 			= 0;
	StartVar.id			= 0;
	StartVar.iq			= 0;
	StartVar.didt 		= 100;
	StartVar.eps 		= 50;//50
	StartVar.w 			= 0;
	StartVar.wMax 		= 150;
	StartVar.angle 		= 0;
	StartVar.iMax 		= 5.F;
	StartVar.u 			= 0;
	
	
	WRITE_REG(TIM1->CCR1, Timer1Period/2);
	WRITE_REG(TIM1->CCR2, Timer1Period/2);
	WRITE_REG(TIM1->CCR3, Timer1Period/2);
	
	//initPI(&uStartPI, kpStart, kiStart, 1.F, 17.F, 0.F, 0.F);
	Flags.MRASEnable = 0;

}


