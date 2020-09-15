#include "dfilt.h"
#include "measurement.h"
#include "stm32f3xx.h" 

TFiltrParm uFiltrParm;
//TMotorCurrents MotorCurrents; 
//TMotorVoltage MotorVoltage;

float shift = 4;//number of ADC conversions not taken into account, because of first conversions inaccuracy

void measureCurrentBase(TMasADC *Mas_ADC, TMotorCurrents *MCurrents){
	
	MCurrents->iA_bias = 0;
	MCurrents->iB_bias = 0;
	MCurrents->iC_bias = 0;
	
	for(int i = shift; i < CONVERSIONS_COUNT; i++){
	    MCurrents->iA_bias += Mas_ADC->A[i];
		MCurrents->iB_bias += Mas_ADC->B[i];
		MCurrents->iC_bias += Mas_ADC->C[i];
    }
	
	MCurrents->iA_bias /= (float)(CONVERSIONS_COUNT-shift);
	MCurrents->iB_bias /= (float)(CONVERSIONS_COUNT-shift);
	MCurrents->iC_bias /= (float)(CONVERSIONS_COUNT-shift);
}


void getPhaseCurrents(TMasADC *Mas_ADC, TMotorCurrents *MCurrents){
	
	MCurrents->iA_izm = 0;
	MCurrents->iB_izm = 0;
	MCurrents->iC_izm = 0;
	
	for(int i = shift; i < CONVERSIONS_COUNT; i++){                       
		MCurrents->iA_izm += Mas_ADC->A[i];
		MCurrents->iB_izm += Mas_ADC->B[i];
		MCurrents->iC_izm += Mas_ADC->C[i];
	}
	
	MCurrents->iA_izm /= (float)(CONVERSIONS_COUNT - shift);
	MCurrents->iB_izm /= (float)(CONVERSIONS_COUNT - shift);
	MCurrents->iC_izm /= (float)(CONVERSIONS_COUNT - shift);
                         
	MCurrents->iA_izm = -((float)(MCurrents->iA_izm - MCurrents->iA_bias) * ((float)CURRENT_CONV_COEF));
	MCurrents->iB_izm = -((float)(MCurrents->iB_izm - MCurrents->iB_bias) * ((float)CURRENT_CONV_COEF));
	MCurrents->iC_izm = -((float)(MCurrents->iC_izm - MCurrents->iC_bias) * ((float)CURRENT_CONV_COEF));
	
	//DAC->DHR12R1 = 2048 + (int32_t)(MCurrents->iA_izm * 1241); 
	

}

void getUdc(TMasADC *Mas_ADC, TMotorVoltage *MVoltage){
	static TFiltrParm uFiltrParm = {{0.F},VOLTAGE_FILTER_LENGTH,0,0.F};;
	float uFloat;
	
	for(int i = 0; i < CONVERSIONS_COUNT; i++){
	    uFloat += Mas_ADC->VDC[i];
    }
	
	uFloat = (((float)uFloat) / ((float)CONVERSIONS_COUNT) * ((float)VOLTAGE_CONV_COEF));
	
    MVoltage->udc_izm = sredN_f(uFloat, &uFiltrParm);
	
}
/*
void ResetUdcFilter(){
	
		// Параметры фильтра напряжения
	uFiltrParm.index  = 0;
	uFiltrParm.length = 16;
	uFiltrParm.sum    = 0.F;
}
*/

int getTemp(void){
	int temp;
	SET_BIT(ADC1->CR, ADC_CR_JADSTART); 	// ADC group injected conversion start
	while(!(READ_BIT(ADC1->ISR, ADC_ISR_JEOS))){};
	CLEAR_BIT(ADC1->ISR, ADC_ISR_JEOS);
	temp = ADC1->JDR1;	
	return (temp);
	
}

