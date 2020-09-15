
#include <stdint.h> 
#include "delay.h"

/*void Delay(uint32_t ms){
       	
	for(int i = (7200 * ms); i>0; i--);
		
	
}*/

/*
void DelayMC(int mc){
	for(int i = 72; i>0; i--);
		
}*/

void Delay(uint32_t ms){
	
        volatile uint32_t nCount;
				nCount=72000000;
	
        nCount=(nCount/10000)*ms;
        for (; nCount!=0; nCount--);
}


void DelayMC(uint32_t mc){
	
        volatile uint32_t nCount;
				nCount=72000000;
	
        nCount=(nCount/10000000)*mc;
        for (; nCount!=0; nCount--);
}
