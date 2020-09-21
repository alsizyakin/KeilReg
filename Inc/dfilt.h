#ifndef DFILT_H
#define DFILT_H
#include <stdint.h>
#define MAX_SREDN_FILTER_LENGTH 512

// ��������� �������� ���������� �������
typedef struct {
	float 		buff[MAX_SREDN_FILTER_LENGTH];		// �����
	uint32_t 	length;			// ������ �������
	uint32_t	index;			// ������
	float 		sum;				// �����
} TFiltrParm;


float sredN_f(float x, TFiltrParm *FiltrParm);

#endif

