#include "RLPsiDetect.h"

TFiltrParm  rFiltrParm;
float detected_R;
int res_detect_counter = 0;
float calc_Ra, calc_Rb, calc_Rc, calc_R;

int meas_stage = 0;




void RLPsiInitFiltr(){
	// Параметры фильтра R
	rFiltrParm.index 	= 0;
	rFiltrParm.length 	= 128;
	//rFiltrParm.sum   	= 0.F;
}

void RLPsiResetCounter(){
	res_detect_counter = 0;
}



// returns 1 while resistances are being calculated;
// returns 0 if resistance is calculated succesfully; 
// returns 2 if calculated resistances are not close to each other
// returns 3 if calculated resistances are very low
int RLPsicalculateResistance(TPIParm* PIc, TStartVar* SVar){ 
		
	if(res_detect_counter < NMEAS) {
		res_detect_counter++;
		calc_R = PIc ->qOut  / PIc -> qInMeas;
		calc_Ra = sredN_f(calc_R, &rFiltrParm);
		return 1;
		
	}else if(res_detect_counter < 2 * NMEAS){
		if (SVar ->angle < M_2PI / 3.F){
			RLPsiInitFiltr();	
			SVar -> angle += MEASOMEG;
		}else {
			SVar -> angle = M_2PI / 3.F;
			res_detect_counter++;
			calc_R = PIc -> qOut  / PIc -> qInMeas;
			calc_Rb = sredN_f(calc_R, &rFiltrParm);
		}
		return 1;
		
	}else if(res_detect_counter < 3 * NMEAS){
		if (SVar -> angle < M_2PI / 1.5F){
			RLPsiInitFiltr();
			SVar -> angle += MEASOMEG;
		} else {
			SVar -> angle = M_2PI / 1.5F;
			res_detect_counter++;
		}
		calc_R = PIc -> qOut  / PIc -> qInMeas;
		calc_Rc = sredN_f(calc_R, &rFiltrParm);	
		return 1;
		
	} else {
		detected_R = (calc_Ra + calc_Rb + calc_Rc)/3.F;
		return 0;
	}
	

}

void RLPsiSetResistance(TMotorParam* MotorParam){
	
	MotorParam -> R = detected_R;
	
}

