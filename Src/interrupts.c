
#include "stm32f3xx.h"
#include "defs.h"
#include "motorcontrol.h"


/**********************************************************************
Timer1 PWM Interrupt Service Routine()
**********************************************************************/
void TIM1_UP_TIM16_IRQHandler(void){
    
	DMA_ENABLE();

	TIM1_IF_CLEAR();	//clear TIM1 interrupt flag
	 
}


/**********************************************************************
DMA1  Interrupt Service Routine()
**********************************************************************/
extern TFlags Flags; 

void DMA1_Channel1_IRQHandler(void){
	
	while(DMA_NOT_READY()){};
												
	DMA_IF_CLEAR();
				
	DMA_DISABLE();	
				
	DMA_CNDTR_RESET()			
		
	Flags.MeasReady = 1;

}

/**********************************************************************
ADC1-4  Interrupt Service Routines()
**********************************************************************/
void ADC1_2_IRQHandler(void){
	
	if (READ_BIT(ADC1->ISR,	ADC_ISR_AWD1)){
		stopMotor();
		SET_BIT(ADC1->ISR,	ADC_ISR_AWD1);
		Flags.CurrentState = STATE_FAULT;
		LED_ON(BUTTON_1);
	}
	
	if (READ_BIT(ADC2->ISR,	ADC_ISR_AWD1)){
		stopMotor();
		SET_BIT(ADC2->ISR,	ADC_ISR_AWD1);
		Flags.CurrentState = STATE_FAULT;
		LED_ON(BUTTON_1);
	}
	
}

void ADC3_IRQHandler(void){
	
	if (READ_BIT(ADC3->ISR,	ADC_ISR_AWD1)){
		stopMotor();
		SET_BIT(ADC3->ISR,	ADC_ISR_AWD1);
		Flags.CurrentState = STATE_FAULT;
		LED_ON(BUTTON_1);
	}
	
}

void ADC4_IRQHandler(void){
	
	if (READ_BIT(ADC4->ISR,	ADC_ISR_AWD1)){
		stopMotor();
		SET_BIT(ADC4->ISR,	ADC_ISR_AWD1);
		Flags.CurrentState = STATE_FAULT;
		LED_ON(BUTTON_1);
		LED_ON(BUTTON_2);
	}
}



