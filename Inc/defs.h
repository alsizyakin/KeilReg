#ifndef DEFS_H
#define DEFS_H


#define USE_INDICATION_BOARD 

/***** Definitions *****/

#define FPWM 					10000						//PWM frequency in Hz.
#define PRPWM    				(1.F/(float)FPWM)   		//PWM period duration
	
#define DT              		1.5F   //invertor dedtime in microseconds
#define DTF             		DT*FPWM*0.000001F

#define SYS_FREQ 				100			//in Hz.
#define R360 					(float)6.283185
	
#define CONVERSIONS_COUNT 		6 // 10 kHz
//#define CURRENT_CONV_COEF 		0.00535 	// R = 22 kOm 
//#define CURRENT_CONV_COEF 		0.00912 	// R = 13 kOm
#define CURRENT_CONV_COEF 		0.00988 	// R = 12 kOm
#define VOLTAGE_CONV_COEF 		0.08138F

//#define AWD_CUR_HT	0x09FA 	// Analog watchdog higher threshold	5A
//#define AWD_CUR_LT	0x0606	// Analog watchdog lower threshold	5A
//#define AWD_CUR_HT	0x0BF4 	// 10A
//#define AWD_CUR_LT	0x040C 	// 10A
#define AWD_CUR_HT	0x0DDE//0x0CC6//0x0FA0 	// 15A
#define AWD_CUR_LT	0x0050//0x0160//0x0212 	// 15A

#define AWD_VOLT_HT	0x0FDC	// Analog watchdog higher threshold	330V
 
#define TEMP_MAX 				2746 // 90 deg
#define TEMP_MIN 				3659 // 50 deg
#define CHECK_TEMP()			READ_BIT(COMP7->CSR, COMP_CSR_COMPxOUT)

#define STOP_VOLTS 				150.F
#define START_VOLTS    		280.F

#ifdef USE_INDICATION_BOARD 
		// Buttons
		#define BUTTON_1 		GPIO_ODR_8	//PB8
		#define BUTTON_2 		GPIO_ODR_9	//PB9

		// LEDS
		#define LED_2 			GPIO_ODR_3	//PB3
		#define LED_1 			GPIO_ODR_4	//PB4
		#define LED_3 			GPIO_ODR_5	//PB5
		#define LED_ALL 		((uint16_t)GPIO_ODR_3|GPIO_ODR_4|GPIO_ODR_5)	//PB3-5

		#define LED_ON(n)		WRITE_REG(GPIOB->BSRR, n);
		#define LED_OFF(n)		WRITE_REG(GPIOB->BSRR, (n<<16));
		#define LED_TOG(n)		GPIOB->ODR ^= n;
		
		#define DAC_OUT_ON()		WRITE_REG(GPIOA->BSRR,	GPIO_ODR_4);
		#define DAC_OUT_OFF()		WRITE_REG(GPIOA->BSRR, (GPIO_ODR_4<<16));
		#define DAC_OUT_TOG()		GPIOA->ODR ^= GPIO_ODR_4;

#else
		#define USER_PIN_1 		GPIO_PIN_3	//PB3
		#define USER_PIN_2 		GPIO_PIN_4	//PB4
		#define USER_PIN_3 		GPIO_PIN_5	//PB5
		#define USER_PIN_4 		GPIO_PIN_8	//PB8
		#define USER_PIN_5 		GPIO_PIN_9	//PB9
		#define USER_PIN_ALL 	((uint16_t)0x0338U)	
		
		#define PIN_SET(n)		WRITE_REG(GPIOB->BSRR, n);
		#define PIN_RESET(n)	WRITE_REG(GPIOB->BSRR, (n<<16));
		#define PIN_TOG(n)		GPIOB->ODR ^= n;		
#endif

// PWM

#define PWM_ON()						SET_BIT(TIM1->BDTR,  TIM_BDTR_MOE);
#define PWM_OFF()						CLEAR_BIT(TIM1->BDTR, TIM_BDTR_MOE);

#define TIM1_ON()						SET_BIT(TIM1->CR1,  TIM_CR1_CEN);
#define TIM1_OFF()					CLEAR_BIT(TIM1->CR1, TIM_CR1_CEN);

#define TIM1_IF_CLEAR()			CLEAR_BIT(TIM1->SR , TIM_SR_UIF);

#define DMA_NOT_READY()   	!(READ_BIT(DMA1->ISR, DMA_ISR_TCIF1) && READ_BIT(DMA2->ISR, DMA_ISR_TCIF1) &&\
														READ_BIT(DMA2->ISR, DMA_ISR_TCIF2) && READ_BIT(DMA2->ISR, DMA_ISR_TCIF5))

#define DMA_IF_CLEAR()			SET_BIT(DMA1->IFCR, DMA_IFCR_CTCIF1);\
														SET_BIT(DMA2->IFCR, DMA_IFCR_CTCIF1);\
														SET_BIT(DMA2->IFCR, DMA_IFCR_CTCIF2);\
														SET_BIT(DMA2->IFCR, DMA_IFCR_CTCIF5);

#define DMA_ENABLE()				SET_BIT(DMA1_Channel1->CCR, 	DMA_CCR_EN);\
														SET_BIT(DMA2_Channel1->CCR,	 	DMA_CCR_EN);\
														SET_BIT(DMA2_Channel5->CCR,  	DMA_CCR_EN);\
														SET_BIT(DMA2_Channel2->CCR,  	DMA_CCR_EN); 			

#define DMA_DISABLE()				CLEAR_BIT(DMA1_Channel1->CCR, 	DMA_CCR_EN);\
														CLEAR_BIT(DMA2_Channel1->CCR,	 	DMA_CCR_EN);\
														CLEAR_BIT(DMA2_Channel5->CCR,  	DMA_CCR_EN);\
														CLEAR_BIT(DMA2_Channel2->CCR,  	DMA_CCR_EN);
														
#define DMA_CNDTR_RESET()		WRITE_REG(DMA1_Channel1->CNDTR, CONVERSIONS_COUNT);\
														WRITE_REG(DMA2_Channel1->CNDTR, CONVERSIONS_COUNT);\
														WRITE_REG(DMA2_Channel5->CNDTR, CONVERSIONS_COUNT);\
														WRITE_REG(DMA2_Channel2->CNDTR, CONVERSIONS_COUNT);
	
#define THERM					GPIO_ODR_0	//PA0
#define VFO						GPIO_ODR_11	//PA11
#define ITRIP	  			GPIO_ODR_12	//PA12
#define ITRIP_ON()		WRITE_REG(GPIOA->BSRR, GPIO_BSRR_BS_12);
#define ITRIP_OFF()		WRITE_REG(GPIOA->BSRR , GPIO_BSRR_BR_12);


#define START_ADC()		  		ADC1->CR |= ADC_CR_ADSTART;\
														ADC2->CR |= ADC_CR_ADSTART;\
														ADC3->CR |= ADC_CR_ADSTART;\
														ADC4->CR |= ADC_CR_ADSTART; 	



#define NTC_RELAY_OFF() 		GPIOA->BSRR = GPIO_BSRR_BR_15;
#define NTC_RELAY_ON()			GPIOA->BSRR = GPIO_BSRR_BS_15;


#endif
