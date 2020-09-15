#ifndef VECTCALC_H
#define VECTCALC_H
#include <stdint.h>
//#include "measurement.h"

// ��������� �������� ���������� ������
typedef struct {
	float R;				// �������������
	float Ld;				// ������������� �� ��� d
	float Lq;				// ������������� �� ��� q
	float psi;				// ���������������
	float Ld_inv;			// �������� �������� ������������� �� ��� d
	float Lq_inv;			// �������� �������� ������������� �� ��� q	
	float iMax;
	float kiw;
	float kpw;
	float kid;
	float kpd;
} TMotorParam;

// ���������� ��� MRAS
typedef struct {
	float tetm;		// 
	float wr;		// 
	float sinm;		// 
	float cosm;		// 
	float kpm;		// 
	float kim;		// 
} TMRASVar;

// ���������� ��� ����������� �������� ������� � ������� ����������
typedef struct {
	float dt;				// ������� �����
	float uvt;			// ������� ���������� �� �����������
	float uvd;			// ������� ���������� �� �����
	float udc;	// ���������� �������� ���������� ���� ����������� ����	    
} TPWMComp;
// ��������� �������� �����
typedef struct {
	float iA_bias; 		// �������� ���� A ������������ ����
	float iB_bias;		// �������� ���� B ������������ ����
	float iC_bias;		// �������� ���� C ������������ ����
	float iA_izm;			// ���������� ��� ���� A
	float iB_izm;			// ���������� ��� ���� B
	float iC_izm;			// ���������� ��� ���� C
	float iAlfa;			// ��� �� ��� �����
	float iBeta;			// ��� �� ��� ����
	float id;					// ��� �� ��� d
	float iq;					// ��� �� ��� q
	float idr;				// ��������� �������� ���� �� ��� d
	float iqr;				// ��������� �������� ���� �� ��� q
	float idZad;			// �������� �������� ���� �� ��� q
	float iqZad;			// �������� �������� ���� �� ��� q
} TMotorCurrents;

// ��������� �������� ����������
typedef struct {
	float udc_izm;		// ���������� �������� ���������� ���� ����������� ����
	float uAlfa;			// ���������� �� ��� �����
	float uBeta;			// ���������� �� ��� ����
	float ud;					// ���������� �� ��� d
	float uq;					// ���������� �� ��� q
	float uref;				// ���������� 
	float ucd;				// ����������� ������������ ������
	float ucq;				// ����������� ������������ ������ 
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

