#ifndef VECTCALC_H
#define VECTCALC_H
#include <stdint.h>
//#include "measurement.h"

// Структура значений параметров машины
typedef struct {
	float R;				// Сопротивление
	float Ld;				// Индуктивность по оси d
	float Lq;				// Индуктивность по оси q
	float psi;				// Потокосцепление
	float Ld_inv;			// Обратное значение индуктивности по оси d
	float Lq_inv;			// Обратное значение индуктивности по оси q	
	float iMax;
	float kiw;
	float kpw;
	float kid;
	float kpd;
} TMotorParam;

// Переменные для MRAS
typedef struct {
	float tetm;		// 
	float wr;		// 
	float sinm;		// 
	float cosm;		// 
	float kpm;		// 
	float kim;		// 
} TMRASVar;

// Переменные для компенсации мертвого времени и падений напряжения
typedef struct {
	float dt;				// Мертвое время
	float uvt;			// Падение напряжения на транзисторе
	float uvd;			// Падение напряжения на диоде
	float udc;	// Измеренное значение напряжения шины постоянного тока	    
} TPWMComp;
// Структура значений токов
typedef struct {
	float iA_bias; 		// Смещение тока A относительно нуля
	float iB_bias;		// Смещение тока B относительно нуля
	float iC_bias;		// Смещение тока C относительно нуля
	float iA_izm;			// Измеренный ток фазы A
	float iB_izm;			// Измеренный ток фазы B
	float iC_izm;			// Измеренный ток фазы C
	float iAlfa;			// Ток по оси Альфа
	float iBeta;			// Ток по оси Бета
	float id;					// Ток по оси d
	float iq;					// Ток по оси q
	float idr;				// Расчетное значение тока по оси d
	float iqr;				// Расчетное значение тока по оси q
	float idZad;			// Заданное значение тока по оси q
	float iqZad;			// Заданное значение тока по оси q
} TMotorCurrents;

// Структура значений напряжений
typedef struct {
	float udc_izm;		// Измеренное значение напряжения шины постоянного тока
	float uAlfa;			// Напряжение по оси Альфа
	float uBeta;			// Напряжение по оси Бета
	float ud;					// Напряжение по оси d
	float uq;					// Напряжение по оси q
	float uref;				// Напряжение 
	float ucd;				// Компенсация перекрестных связей
	float ucq;				// Компенсация перекрестных связей 
} TMotorVoltage;

//------------Function Prototypes-----------------------------------------------------------------

void transormClarkeCurrent(TMotorCurrents *MCurrents);

void transformFwdParkCurrent(TMotorCurrents *MCurrents, TMRASVar *MRAS_Var);
	
void transformFwdParkVoltage(TMotorVoltage *MVoltage, TMRASVar *MRAS_Var);

void transformRewParkVoltage(TMotorVoltage *MVoltage, TMRASVar *MRAS_Var);

int calculateMRAS(TMotorCurrents *MCurrents, TMotorVoltage *MVoltage, TMotorParam *MParam,\
					TMRASVar *MRAS_Var, float Tpwm);

int SVPWM_f(float alpha,  TMotorVoltage *MVoltage, float mas[]);

void PWMcompensation (float tpwm, float mas_in[], uint32_t mas_out[], TMotorCurrents *MCurrents, TPWMComp *PWM_Comp);

#endif

