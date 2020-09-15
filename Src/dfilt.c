#include "dfilt.h"


float sredN_f(float x, TFiltrParm *FiltrParm){

	FiltrParm->sum = FiltrParm->sum + ( x - FiltrParm->buff[FiltrParm->index]);
	FiltrParm->buff[FiltrParm->index] = x;
	FiltrParm->index = (FiltrParm->index + 1) & (FiltrParm->length - 1);
	return FiltrParm->sum/(FiltrParm->length);
 
}

