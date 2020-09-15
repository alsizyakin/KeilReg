#ifndef MEASUREMENT_H
#define MEASUREMENT_H
#include "dfilt.h"
#include "defs.h"
#include "VectCalc.h"

#define VOLTAGE_FILTER_LENGTH 16

#if VOLTAGE_FILTER_LENGTH > MAX_SREDN_FILTER_LENGTH
	#error "VOLTAGE_FILTER_LENGTH couldn't be bigger than MAX_SREDN_FILTER_LENGTH"
#endif

typedef struct {
	uint32_t A[CONVERSIONS_COUNT];			// Ток фазы A
	uint32_t B[CONVERSIONS_COUNT];			// Ток фазы B
	uint32_t C[CONVERSIONS_COUNT];			// Ток фазы C
	uint32_t VDC[CONVERSIONS_COUNT];		// Напряжение шины постоянного тока
} TMasADC;




void measureCurrentBase(TMasADC *Mas_ADC, TMotorCurrents *MCurrents);

void getPhaseCurrents(TMasADC *Mas_ADC, TMotorCurrents *MCurrents);
	
void getUdc(TMasADC *Mas_ADC, TMotorVoltage *MVoltage);

int getTemp(void);

#endif


