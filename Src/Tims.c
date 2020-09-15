#include "Tims.h"
#include "stm32f3xx.h"  
#include "defs.h"


void init_timer1(void){
	
	uint32_t Timer1Period;
		
	// TIM1 clock enable
	SET_BIT(RCC->APB2ENR,		RCC_APB2ENR_TIM1EN); 
	
    // Compute the value to be set in ARR register to generate signal frequency at FPWM
	Timer1Period = (((float)SystemCoreClock / (float)FPWM)/2.F)-1.F;

	// Auto-reload preload enable 
	SET_BIT(TIM1->CR1, 			TIM_CR1_ARPE);  
  
	// 01: Center-aligned mode 1
	SET_BIT(TIM1->CR1, 			TIM_CR1_CMS_0); 
	CLEAR_BIT(TIM1->CR1,		TIM_CR1_CMS_1);
	
	// Master Mode Selection
	SET_BIT(TIM1->CR2, 			TIM_CR2_MMS_1); // 010: Update - The update event is selected as trigger output (TRGO).
  
	// Capture/Compare Preloaded Control
	SET_BIT(TIM1->CR2, 			TIM_CR2_CCPC);  
  
	// Output Compare 1,2,3 Preload enable
	SET_BIT(TIM1->CCMR1, 		TIM_CCMR1_OC2PE | TIM_CCMR1_OC1PE);
	SET_BIT(TIM1->CCMR2, 		TIM_CCMR2_OC3PE);
  
	// TIM auto-reload register
	WRITE_REG(TIM1->ARR, 		Timer1Period);
	// TIM prescaler
	WRITE_REG(TIM1->PSC, 		0);
  
	//TIM repetition counter register
	WRITE_REG(TIM1->RCR, 		1);
  
	//Update generation
	SET_BIT(TIM1->EGR, 			TIM_EGR_UG); 
  
	
	// Capture/Compare output enable
    SET_BIT(TIM1->CCER, 		TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E);
    SET_BIT(TIM1->CCER, 		TIM_CCER_CC1NE | TIM_CCER_CC2NE | TIM_CCER_CC3NE);

    SET_BIT(TIM1->CCMR1, 		TIM_CCMR1_OC1M_0|TIM_CCMR1_OC1M_1|TIM_CCMR1_OC1M_2);//0x0070); //PWM mode 2 
    SET_BIT(TIM1->CCMR1,		TIM_CCMR1_OC2M_0|TIM_CCMR1_OC2M_1|TIM_CCMR1_OC2M_2);//0x7000); //PWM mode 2 
    SET_BIT(TIM1->CCMR2, 		TIM_CCMR2_OC3M_0|TIM_CCMR2_OC3M_1|TIM_CCMR2_OC3M_2);//0x0070); //PWM mode 2 

	// TIM capture/compare registers
    WRITE_REG(TIM1->CCR1, 		Timer1Period/2);   
    WRITE_REG(TIM1->CCR2, 		Timer1Period/2);   
    WRITE_REG(TIM1->CCR3, 		Timer1Period/2);   
  
	// Update interrupt enable
    SET_BIT(TIM1->DIER, 		TIM_DIER_UIE);
  
	// Off-State Selection for Idle mode and for Run mode
    SET_BIT(TIM1->BDTR, 		TIM_BDTR_OSSI | TIM_BDTR_OSSR);
  
    // Dead time
	MODIFY_REG(TIM1->BDTR, 		TIM_BDTR_DTG,0x006D);//SET_BIT(TIM1->BDTR,0x006D);// 1.5 us
  
	//Capture/Compare control update generation
    SET_BIT(TIM1->EGR, 			TIM_EGR_COMG);  
  
	// Main Output enable
    CLEAR_BIT(TIM1->BDTR,		TIM_BDTR_MOE); 
	
	
	// Counter enable
    CLEAR_BIT(TIM1->CR1,		TIM_CR1_CEN);
		
		NVIC_SetPriority(TIM1_UP_TIM16_IRQn, 1); 
		NVIC_EnableIRQ(TIM1_UP_TIM16_IRQn ); 
  
}
