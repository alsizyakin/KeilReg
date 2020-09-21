#ifndef DFILT_H
#define DFILT_H
#include <stdint.h>
#define MAX_SREDN_FILTER_LENGTH 512

// Структура значений параметров фильтра
typedef struct {
	float 		buff[MAX_SREDN_FILTER_LENGTH];		// Буфер
	uint32_t 	length;			// Длинна буффера
	uint32_t	index;			// Индекс
	float 		sum;				// Сумма
} TFiltrParm;


float sredN_f(float x, TFiltrParm *FiltrParm);

#endif

