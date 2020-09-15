#include "VectCalc.h"
#include "mymath.h"
#include "arm_math.h"
#include "arm_common_tables.h"
#include "defs.h"


void transormClarkeCurrent(TMotorCurrents *MCurrents){  
	
	MCurrents->iAlfa = MCurrents->iA_izm;
	MCurrents->iBeta = (2.F*(MCurrents->iB_izm)+(MCurrents->iA_izm))*((float)0.57735027);					
	
}


void transformFwdParkCurrent(TMotorCurrents *MCurrents, TMRASVar *MRAS_Var){
	
	MCurrents->id = MCurrents->iAlfa * MRAS_Var->cosm + MCurrents->iBeta * MRAS_Var->sinm;
	MCurrents->iq = MCurrents->iBeta * MRAS_Var->cosm - MCurrents->iAlfa * MRAS_Var->sinm;
	
}

void transformFwdParkVoltage(TMotorVoltage *MVoltage, TMRASVar *MRAS_Var){
	
	MVoltage->ud = MVoltage->uAlfa * MRAS_Var->cosm + MVoltage->uBeta * MRAS_Var->sinm;
	MVoltage->uq = MVoltage->uBeta * MRAS_Var->cosm - MVoltage->uAlfa * MRAS_Var->sinm;
	
}


void transformRewParkVoltage(TMotorVoltage *MVoltage, TMRASVar *MRAS_Var){

	MVoltage->uAlfa = MVoltage->ud * MRAS_Var->cosm - MVoltage->uq * MRAS_Var->sinm;
	MVoltage->uBeta = MVoltage->ud * MRAS_Var->sinm + MVoltage->uq * MRAS_Var->cosm;
                 
}



int calculateMRAS(TMotorCurrents *MCurrents, TMotorVoltage *MVoltage, TMotorParam *MParam, TMRASVar *MRAS_Var, float Tpwm){
                
                
	float dId, dIq, dw = 0;
	static float id_mras = 0, iq_mras = 0, intw = 0;
 
	MCurrents->idr = MCurrents->iAlfa * MRAS_Var->cosm + MCurrents->iBeta * MRAS_Var->sinm;
	MCurrents->iqr = MCurrents->iBeta * MRAS_Var->cosm - MCurrents->iAlfa * MRAS_Var->sinm;
        
	dId = ((-MParam->R * id_mras) + (MRAS_Var->wr * iq_mras * MParam->Lq) + MVoltage->ud)* MParam->Ld_inv;
	
	dIq = ((-MRAS_Var->wr * id_mras * MParam->Ld) - (MParam->R * iq_mras) - (MRAS_Var->wr * MParam->psi) + MVoltage->uq) * MParam->Lq_inv; 
  
	id_mras = id_mras + (dId) * Tpwm;
	iq_mras = iq_mras + (dIq) * Tpwm;

	dw = (MCurrents->idr * iq_mras) - (id_mras * MCurrents->iqr) - (MCurrents->iqr - iq_mras) * MParam->psi * MParam->Ld_inv;
  
	intw += dw * Tpwm * MRAS_Var->kim;
	MRAS_Var->wr = intw + dw * MRAS_Var->kpm;

	MRAS_Var->tetm = MRAS_Var->tetm + MRAS_Var->wr * Tpwm;
  
	//limitation of angle  betwen zero and 2pi
	if(MRAS_Var->tetm > M_2PI){
		MRAS_Var->tetm -= M_2PI;
	}else if(MRAS_Var->tetm < 0.F){
		MRAS_Var->tetm += M_2PI;
	}
	
	/*if ((MRAS_Var->tetm > 6.15F) || (MRAS_Var->tetm < 0.15)){
					DAC_OUT_ON();
	}
	else{
					DAC_OUT_OFF();
	}*/

  
	//sine and cosine calculation for main program
	MRAS_Var->sinm = arm_sin_f32(MRAS_Var->tetm);
	MRAS_Var->cosm = arm_cos_f32(MRAS_Var->tetm);
    
	return(0);
	 
}  


int SVPWM_f(float alpha,  TMotorVoltage *MVoltage, float mas[]){
  
	float tb1, tb2, tb0;
	float sinb, sinbm;
	int   k;
	float beta;
	float dus = MVoltage->uref / MVoltage->udc_izm;
  
	//Определение сектора угла Альфа. 
	k = (int32_t)(alpha/1.04719755F);
	beta = alpha - (float)k * 1.04719755F;
	sinb = arm_sin_f32(beta);
	sinbm = arm_sin_f32(1.04719755F - beta);
 
	tb1 = dus*sinbm*1.73205081F;//Расчет части периода ШИМ во время которой происходит включение первого базового вектора (в относительных единицах)
	tb2 = dus*sinb*1.73205081F;//Расчет части периода ШИМ во время которой происходит включение второго базового вектора (в относительных единицах)
	tb0 = (1.F-tb1-tb2)/2.F;//Расчет части периода ШИМ во время которой происходит включение нулевого базового вектора (в относительных единицах)
      
	
	//Присвоение каждому ключу соответствующего ему времени включения в зависимости от сектора угла Альфа
	if(k > 2){
		if(k > 4){ 
			// Сектор 6
			mas[0] = tb1+tb2+tb0; 		//Расчет времени подключения фазы A к +Ud в период ШИМ (в относительных единицах)
			mas[1] = tb0;         		//Расчет времени подключения фазы B к +Ud в период ШИМ (в относительных единицах)
			mas[2] = tb1+tb0;     		//Расчет времени подключения фазы C к +Ud в период ШИМ (в относительных единицах)
		}else{
			if(k == 3){
				// Сектор 4
				mas[0] = tb0;
				mas[1] = tb1+tb0; 
				mas[2] = tb1+tb2+tb0;
			}else{
				// Сектор 5
				mas[0] = tb2+tb0;  
				mas[1] = tb0;
				mas[2] = tb1+tb2+tb0; 
			}
		}
	}else{
		if(k > 1){
			// Сектор 3
			mas[0] = tb0;
			mas[1] = tb1+tb2+tb0;
			mas[2] = tb2+tb0;
		}else{ 
			if(k == 0){
				// Сектор 1
				mas[0] = tb1+tb2+tb0;
				mas[1] = tb2+tb0;
				mas[2] = tb0;
			}else{
				// Сектор 2
				mas[0] = tb1+tb0;
				mas[1] = tb1+tb2+tb0;
				mas[2] = tb0; 
			}
		}
	}
	return 0;
}


void PWMcompensation (float tpwm, float mas_in[], uint32_t mas_out[], TMotorCurrents *MCurrents, TPWMComp *PWM_Comp){
	float uznam = PWM_Comp->udc - PWM_Comp->uvt + PWM_Comp->uvd;
	float dt2 = PWM_Comp->dt;
	
	if(MCurrents->iA_izm > 0){
		mas_in[0] += dt2;
	}else{
		mas_in[0] -= dt2;
	}
	
	if(MCurrents->iB_izm > 0){
		mas_in[1] += dt2;
	}else{
		mas_in[1] -= dt2;
	}
	
	if((-MCurrents->iA_izm - MCurrents->iB_izm) > 0){
		mas_in[2] += dt2;
	}else{
		mas_in[2] -= dt2;
	}
	
	if(mas_in[0] < 0.F){
		mas_out[0] = 0.F;
	}else {
		if(mas_in[0] > 1.0F){
			mas_out[0] = 1.0F;
		}
	}
	
	if(mas_in[1] < 0.F){
		mas_in[1] = 0.F;
	}else {
		if(mas_in[1] > 1.0F){
			mas_in[1] = 1.0F;
		}
	}
	
	if(mas_in[2] < 0.F){
		mas_in[2] = 0.F;
	}else {
		if(mas_in[2] > 1.0F){
			mas_in[2] = 1.0F; 
		}
	}
	
	
	for(int i = 0; i < 3; i++){
		mas_out[i] = (uint32_t)((1.F-mas_in[i])*tpwm);
	}
	
		
}
							

