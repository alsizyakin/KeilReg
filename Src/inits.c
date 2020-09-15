#include "inits.h"
#include "measurement.h"
#include "delay.h" 
#include "stm32f3xx.h" 


void init_ports(void){
 
	/***** Clock enable *****/
  
	// GPIOA clock enable
	SET_BIT(RCC->AHBENR, RCC_AHBENR_GPIOAEN);

	// GPIOB clock enable
	SET_BIT(RCC->AHBENR, RCC_AHBENR_GPIOBEN);

	// GPIOC clock enable
	SET_BIT(RCC->AHBENR, RCC_AHBENR_GPIOCEN);

	// SYSCFG clock enable
	SET_BIT(RCC->AHBENR, RCC_APB2ENR_SYSCFGEN);


#ifdef USE_INDICATION_BOARD 	
	/***** LEDs *****/
	
	/* 
	 PB3  = LED_1
	 PB4  = LED_2
	 PB5  = LED_3
	*/
	// Configure IOs in output push-pull mode to drive external LEDs 
			
	SET_BIT(RCC->AHBENR, 		RCC_AHBENR_GPIOBEN); 
	CLEAR_BIT(GPIOB->MODER, 	GPIO_MODER_MODER3|GPIO_MODER_MODER4|GPIO_MODER_MODER5);
	SET_BIT(GPIOB->MODER, 		GPIO_MODER_MODER3_0|GPIO_MODER_MODER4_0|GPIO_MODER_MODER5_0);
	
	CLEAR_BIT(GPIOB->OTYPER, 	(GPIO_OTYPER_OT_3|GPIO_OTYPER_OT_4|GPIO_OTYPER_OT_5));
	CLEAR_BIT(GPIOB->OSPEEDR,	(GPIO_OSPEEDER_OSPEEDR3|GPIO_OSPEEDER_OSPEEDR4|GPIO_OSPEEDER_OSPEEDR5));
	CLEAR_BIT(GPIOB->PUPDR, 	(GPIO_PUPDR_PUPDR3|GPIO_PUPDR_PUPDR4|GPIO_PUPDR_PUPDR5));
			
	/***** Buttons *****/
					
	/* 
	 PB8  = BUTTON_1
	 PB9  = BUTTON_2
	*/
			
	// Configure PB3, PB4 in input mode
	//SET_BIT(RCC->AHBENR, 		RCC_AHBENR_GPIOBEN); 
	//CLEAR_BIT(GPIOB->MODER, 	(GPIO_MODER_MODER8_0|GPIO_MODER_MODER9_0));
	//CLEAR_BIT(GPIOB->PUPDR,		(GPIO_PUPDR_PUPDR8|GPIO_PUPDR_PUPDR9));
	
	CLEAR_BIT(GPIOB->MODER, 	GPIO_MODER_MODER8|GPIO_MODER_MODER9);
	SET_BIT(GPIOB->MODER, 		GPIO_MODER_MODER8_0|GPIO_MODER_MODER9_0);
	
	CLEAR_BIT(GPIOB->OTYPER, 	(GPIO_OTYPER_OT_8|GPIO_OTYPER_OT_9));
	CLEAR_BIT(GPIOB->OSPEEDR,	(GPIO_OSPEEDER_OSPEEDR8|GPIO_OSPEEDER_OSPEEDR9));
	CLEAR_BIT(GPIOB->PUPDR, 	(GPIO_PUPDR_PUPDR8|GPIO_PUPDR_PUPDR9));
	
#else
			/***** USER PINs 	*****/
				
			// Configure IOs in output push-pull mode 
			GPIO_InitStructure.Pin	 = USER_PIN_1 | USER_PIN_2 | USER_PIN_3 | USER_PIN_4 | USER_PIN_5;
			GPIO_InitStructure.Mode  = GPIO_MODE_OUTPUT_PP;
			GPIO_InitStructure.Pull  = GPIO_NOPULL;
			GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
			HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
#endif
	
	/***** NTC Relay PA15 *****/
	SET_BIT(RCC->AHBENR, 		RCC_AHBENR_GPIOAEN);
	
	CLEAR_BIT(GPIOA->MODER,		GPIO_MODER_MODER15);
	SET_BIT(GPIOA->MODER, 		GPIO_MODER_MODER15_0);
	
	CLEAR_BIT(GPIOA->OTYPER, 	(GPIO_OTYPER_OT_15));	
	SET_BIT(GPIOA->OSPEEDR, 	(GPIO_OSPEEDER_OSPEEDR15));
	
	CLEAR_BIT(GPIOA->PUPDR,   	(GPIO_PUPDR_PUPDR15));
	

  /***** PWM outputs *****/
      
       /* 
         PA8  = HIN_U
         PA9  = HIN_V 
         PA10 = HIN_W
         PB13 = LIN_U
         PB14 = LIN_V 
         PB15 = LIN_W
      */

      // Configure PA8-PA10 in TIM1 Alternate Function mode  

	  
	SET_BIT(RCC->AHBENR,     	RCC_AHBENR_GPIOAEN); 
	
	CLEAR_BIT(GPIOA->MODER,  	GPIO_MODER_MODER8|GPIO_MODER_MODER9|GPIO_MODER_MODER10);
	SET_BIT(GPIOA->MODER,    	GPIO_MODER_MODER8_1|GPIO_MODER_MODER9_1|GPIO_MODER_MODER10_1);
	
	CLEAR_BIT(GPIOA->OTYPER, 	(GPIO_OTYPER_OT_8|GPIO_OTYPER_OT_9|GPIO_OTYPER_OT_10));
	
	CLEAR_BIT(GPIOA->OSPEEDR, 	(GPIO_OSPEEDER_OSPEEDR8|GPIO_OSPEEDER_OSPEEDR9|GPIO_OSPEEDER_OSPEEDR10));
	SET_BIT(GPIOA->OSPEEDR, 	(GPIO_OSPEEDER_OSPEEDR8|GPIO_OSPEEDER_OSPEEDR9|GPIO_OSPEEDER_OSPEEDR10));
	
	CLEAR_BIT(GPIOA->PUPDR, 	(GPIO_PUPDR_PUPDR8|GPIO_PUPDR_PUPDR9|GPIO_PUPDR_PUPDR10));
  
	MODIFY_REG(GPIOA->AFR[1],	GPIO_AFRH_AFRH0_Msk,(6 << GPIO_AFRH_AFRH0_Pos));
	MODIFY_REG(GPIOA->AFR[1],	GPIO_AFRH_AFRH1_Msk,(6 << GPIO_AFRH_AFRH1_Pos));
	MODIFY_REG(GPIOA->AFR[1],	GPIO_AFRH_AFRH2_Msk,(6 << GPIO_AFRH_AFRH2_Pos));
	
      
    // Configure PB13-PB15 in TIM1 Alternate Function mode  
	  
	  
	SET_BIT(RCC->AHBENR, 		RCC_AHBENR_GPIOBEN); 
	SET_BIT(GPIOB->MODER, 		GPIO_MODER_MODER13_1|GPIO_MODER_MODER14_1|GPIO_MODER_MODER15_1);
	CLEAR_BIT(GPIOB->OTYPER, 	(GPIO_OTYPER_OT_13|GPIO_OTYPER_OT_14|GPIO_OTYPER_OT_15));
	SET_BIT(GPIOB->OSPEEDR, 	(GPIO_OSPEEDER_OSPEEDR13|GPIO_OSPEEDER_OSPEEDR14|GPIO_OSPEEDER_OSPEEDR15));
	CLEAR_BIT(GPIOB->PUPDR, 	(GPIO_PUPDR_PUPDR13|GPIO_PUPDR_PUPDR14|GPIO_PUPDR_PUPDR15));
  
	MODIFY_REG(GPIOB->AFR[1],	GPIO_AFRH_AFRH5_Msk,(6 << GPIO_AFRH_AFRH5_Pos));
	MODIFY_REG(GPIOB->AFR[1],	GPIO_AFRH_AFRH6_Msk,(6 << GPIO_AFRH_AFRH6_Pos));
	MODIFY_REG(GPIOB->AFR[1],	GPIO_AFRH_AFRH7_Msk,(4 << GPIO_AFRH_AFRH7_Pos));
      
			
	/***** Other *****/
    /* 
	 PA0   = THERM
	 PA11  = VFO
	 PA12  = ITRIP
	*/
			
	// Configure thermistor input in analog mode 
				
	SET_BIT(RCC->AHBENR, 		RCC_AHBENR_GPIOAEN); 
	SET_BIT(GPIOA->MODER,  		GPIO_MODER_MODER0);
	CLEAR_BIT(GPIOA->PUPDR,  	GPIO_PUPDR_PUPDR0);		

	// Configure VFO in input mode
	
	SET_BIT(RCC->AHBENR, 		RCC_AHBENR_GPIOAEN); 
	CLEAR_BIT(GPIOA->MODER,  	GPIO_MODER_MODER11_0);
	CLEAR_BIT(GPIOA->PUPDR,  	(GPIO_PUPDR_PUPDR11));
			
	// Configure ITRIP in output push-pull mode
	
	SET_BIT(RCC->AHBENR, 		RCC_AHBENR_GPIOAEN); 
	CLEAR_BIT(GPIOA->MODER, 	GPIO_MODER_MODER12);
	SET_BIT(GPIOA->MODER, 		GPIO_MODER_MODER12_0);
	
	CLEAR_BIT(GPIOA->OTYPER, 	(GPIO_OTYPER_OT_12));
	SET_BIT(GPIOA->OSPEEDR,  	(GPIO_OSPEEDER_OSPEEDR12));
	CLEAR_BIT(GPIOA->PUPDR,   	(GPIO_PUPDR_PUPDR12));


}

void init_comp(void){
	
		/***** COMP 7 for Therm *****/
		SET_BIT(GPIOA->MODER, 	GPIO_MODER_MODER0 | GPIO_MODER_MODER4);  //Analog Mode  PA0 and PA4
	
		COMP7->CSR |= COMP_CSR_COMPxMODE_1; 	// Mode: Medium speed
		COMP7->CSR |= COMP_CSR_COMPxINSEL_2;	// Inverting input: PA4 or DAC1 
		COMP7->CSR &= ~COMP_CSR_COMPxNONINSEL;// Non inverting input: PA0
	
	  COMP7->CSR |= COMP_CSR_COMPxPOL; 			// Output is inverted
    
		/***** DAC 1  for COMP 7 *****/
		RCC->APB1ENR |= RCC_APB1ENR_DAC1EN;
    DAC->CR &= ~ DAC_CR_TEN1; 						//DAC channel 1 trigger disabled
	
    DAC->CR 	|= DAC_CR_EN1;					// DAC enable
		COMP7->CSR |= COMP_CSR_COMPxEN;			// COMP enable
	
		DAC->DHR12R1 = TEMP_MAX;
  
}

void init_opamp(void){
	
	// SYSCFG clock enable
	SET_BIT(RCC->APB2ENR, 		RCC_APB2ENR_SYSCFGEN);


	/***** OPAMP 1 *****/
	
    /*
			PA3 = OPAMP1_VINM
			PA1 = OPAMP1_VINP
			PA2 = OPAMP1_VOUT
		*/
	
    SET_BIT(RCC->AHBENR, 		RCC_AHBENR_GPIOAEN); 
    SET_BIT(GPIOA->MODER,   	GPIO_MODER_MODER1|GPIO_MODER_MODER2|GPIO_MODER_MODER3);
  
  	SET_BIT(OPAMP1->CSR, 		OPAMP1_CSR_VPSEL); //PA1 used as OPAMP1 non inverting input
	
		SET_BIT(OPAMP1->CSR, 		OPAMP1_CSR_VMSEL_0); //PA3 (VM1) used as OPAMP1 inverting input
  
    SET_BIT(OPAMP1->CSR, 		OPAMP1_CSR_OPAMP1EN); //OPAMP1 enable
	
	/***** OPAMP 2 *****/
	
    /*
			PA5 = OPAMP2_VINM
			PA7 = OPAMP2_VINP
			PA6 = OPAMP2_VOUT
	*/
	
    SET_BIT(RCC->AHBENR, 		RCC_AHBENR_GPIOAEN);
	SET_BIT(GPIOA->MODER,   	GPIO_MODER_MODER5|GPIO_MODER_MODER6|GPIO_MODER_MODER7);
	
	SET_BIT(OPAMP2->CSR, 		OPAMP2_CSR_VPSEL); //PA7 used as OPAMP2 non inverting input
	
	SET_BIT(OPAMP2->CSR, 		OPAMP2_CSR_VMSEL_0); //PA5 (VM1) used as OPAMP2 inverting input
  
    SET_BIT(OPAMP2->CSR,		OPAMP2_CSR_OPAMP2EN); //OPAMP2 enable
	
	/***** OPAMP 3 *****/
	
   /*
			PB2 = OPAMP3_VINM
			PB0 = OPAMP3_VINP
			PB1 = OPAMP3_VOUT
	 */
	
    SET_BIT(RCC->AHBENR, 		RCC_AHBENR_GPIOBEN);
	SET_BIT(GPIOB->MODER,   	GPIO_MODER_MODER0|GPIO_MODER_MODER1|GPIO_MODER_MODER2);
	
	SET_BIT(OPAMP3->CSR, 		OPAMP3_CSR_VPSEL); //PB0 used as OPAMP3 non inverting input
	
	SET_BIT(OPAMP3->CSR, 		OPAMP3_CSR_VMSEL_0); //PB2 (VM1) used as OPAMP3 inverting input
  
    SET_BIT(OPAMP3->CSR,		OPAMP3_CSR_OPAMP3EN); //OPAMP3 enable
  
	/***** OPAMP 4 *****/
	
   /*
			PB10 = OPAMP4_VINM
			PB11 = OPAMP4_VINP
			PB12 = OPAMP4_VOUT
	 */
	

  
    SET_BIT(RCC->AHBENR, 		RCC_AHBENR_GPIOBEN);
	SET_BIT(GPIOB->MODER,   	GPIO_MODER_MODER10|GPIO_MODER_MODER11|GPIO_MODER_MODER12);
	
	SET_BIT(OPAMP4->CSR,		OPAMP4_CSR_VPSEL_0); //PB11 used as OPAMP4 non inverting input
	
	CLEAR_BIT(OPAMP4->CSR, 		OPAMP4_CSR_VMSEL); //PB10 (VM0) used as OPAMP4 inverting input
  
    SET_BIT(OPAMP4->CSR, 		OPAMP4_CSR_OPAMP4EN); //OPAMP4 enable
	
}

void init_awd(void){
	
	/***** ADC1 analog watchdog enable *****/
	
	SET_BIT(ADC1->CFGR,	ADC_CFGR_AWD1EN); 	// ADC 1 analog watchdog 1 enable 
	SET_BIT(ADC1->CFGR,	ADC_CFGR_AWD1SGL);	// Enable the watchdog 1 on a single channel
	
	SET_BIT(ADC1->CFGR,	ADC_CFGR_AWD1CH_0 | ADC_CFGR_AWD1CH_1);	// ADC analog input channel 3 monitored by AWD1
	
	SET_BIT(ADC1->IER,	ADC_IER_AWD1IE);	// Analog watchdog 1 interrupt enable
	
	CLEAR_REG(ADC1->TR1);
	ADC1->TR1 |= (AWD_CUR_HT)<<16;	// Analog watchdog 1 higher threshold
	ADC1->TR1 |= AWD_CUR_LT;		// Analog watchdog 1 lower threshold
	
	/***** ADC2 analog watchdog enable *****/
	
	SET_BIT(ADC2->CFGR,	ADC_CFGR_AWD1EN); 	// ADC 2 analog watchdog 1 enable 
	SET_BIT(ADC2->CFGR,	ADC_CFGR_AWD1SGL);	// Enable the watchdog 1 on a single channel
	
	SET_BIT(ADC2->CFGR,	ADC_CFGR_AWD1CH_0);	// ADC analog input channel 3 monitored by AWD1
	SET_BIT(ADC2->CFGR,	ADC_CFGR_AWD1CH_0);
	
	SET_BIT(ADC2->IER,	ADC_IER_AWD1IE);	// Analog watchdog 1 interrupt enable
	
	CLEAR_REG(ADC2->TR1);
	ADC2->TR1 |= (AWD_CUR_HT)<<16;	// Analog watchdog 1 higher threshold
	ADC2->TR1 |= AWD_CUR_LT;		// Analog watchdog 1 lower threshold
	
	NVIC_SetPriority(ADC1_2_IRQn, 1); 
	NVIC_EnableIRQ(ADC1_2_IRQn); 
	
	/***** ADC3 analog watchdog enable *****/
	
	SET_BIT(ADC3->CFGR,	ADC_CFGR_AWD1EN); 	// ADC 3 analog watchdog 1 enable 
	SET_BIT(ADC3->CFGR,	ADC_CFGR_AWD1SGL);	// Enable the watchdog 1 on a single channel
	
	SET_BIT(ADC3->CFGR,	ADC_CFGR_AWD1CH_0);	// ADC analog input channel 1 monitored by AWD1
	
	SET_BIT(ADC3->IER,	ADC_IER_AWD1IE);	// Analog watchdog 1 interrupt enable
	
	CLEAR_REG(ADC3->TR1);
	ADC3->TR1 |= ((AWD_CUR_HT)<<16);	// Analog watchdog 1 higher threshold
	ADC3->TR1 |= AWD_CUR_LT;		// Analog watchdog 1 lower threshold
	
	NVIC_SetPriority(ADC3_IRQn, 1); 
	NVIC_EnableIRQ(ADC3_IRQn); 
	
	/***** ADC4 analog watchdog enable *****/
	
	SET_BIT(ADC4->CFGR,	ADC_CFGR_AWD1EN); 	// ADC analog watchdog 1 enable 
	SET_BIT(ADC4->CFGR,	ADC_CFGR_AWD1SGL);	// Enable the watchdog 1 on a single channel
	
	SET_BIT(ADC4->CFGR,	ADC_CFGR_AWD1CH_0 | ADC_CFGR_AWD1CH_1);	// ADC analog input channel 3 monitored by AWD1
	
	SET_BIT(ADC4->IER,	ADC_IER_AWD1IE);	// Analog watchdog 1 interrupt enable
	
	CLEAR_REG(ADC4->TR1);
	ADC4->TR1 |= ((AWD_VOLT_HT)<<16);	// Analog watchdog 1 higher threshold
	ADC4->TR1 |= 0x0000;			// Analog watchdog 1 lower threshold
	
	NVIC_SetPriority(ADC4_IRQn, 1); 
	NVIC_EnableIRQ(ADC4_IRQn);

}

void init_adc(void){
	
	  // ADC1/ ADC2 clock enable
	//SET_BIT(RCC->CFGR2, 		RCC_CFGR2_ADCPRE12_DIV2);
	SET_BIT(RCC->CFGR2, 		RCC_CFGR2_ADCPRE12_DIV1);
  SET_BIT(RCC->AHBENR, 		RCC_AHBENR_ADC12EN);
	
	// ADC3/ ADC4 clock enable
	//SET_BIT(RCC->CFGR2,			RCC_CFGR2_ADCPRE34_DIV2);
	SET_BIT(RCC->CFGR2,			RCC_CFGR2_ADCPRE34_DIV1);
	SET_BIT(RCC->AHBENR, 		RCC_AHBENR_ADC34EN);
	
	
	/***** ADC voltage regulator enable *****/
	CLEAR_BIT(ADC1->CR, 		ADC_CR_ADVREGEN); 		// 00: Intermediate state 
	SET_BIT(ADC1->CR,  			ADC_CR_ADVREGEN_0); 	// 01: ADC Voltage regulator enabled.
	
	CLEAR_BIT(ADC2->CR, 		ADC_CR_ADVREGEN); 		// 00: Intermediate state 
	SET_BIT(ADC2->CR, 			ADC_CR_ADVREGEN_0); 	// 01: ADC Voltage regulator enabled.
	
	CLEAR_BIT(ADC3->CR, 		ADC_CR_ADVREGEN); 		// 00: Intermediate state 
	SET_BIT(ADC3->CR,  			ADC_CR_ADVREGEN_0); 	// 01: ADC Voltage regulator enabled.
	
	CLEAR_BIT(ADC4->CR, 		ADC_CR_ADVREGEN); 		// 00: Intermediate state 
	SET_BIT(ADC4->CR,  			ADC_CR_ADVREGEN_0); 	// 01: ADC Voltage regulator enabled.
	
	DelayMC(10); // Wait for the startup time of the ADC voltage regulator
			
	
	/***** ADC 1 For OPAMP1 *****/
	CLEAR_BIT(ADC1->CR, 		ADC_CR_ADCALDIF); 	// Single-ended inputs Mode
	SET_BIT(ADC1->CR,  			ADC_CR_ADCAL); 			// ADC calibration
	while(READ_BIT(ADC1->CR ,	ADC_CR_ADCAL)); 		// Wait for end of ADC calibration
		
	SET_BIT(ADC1->SQR1, 		ADC_SQR1_SQ1_0 | ADC_SQR1_SQ1_1); // 1st conversion in sequence Channel 3
		
	//ADC1->SMPR1 |= ADC_SMPR1_SMP3_1; // Sampling time = 2.5 ADC clock cycles
		
	//ADC1->CFGR	|= ADC_CFGR_DISCEN; 	// ADC group regular sequencer discontinuous mode
	SET_BIT(ADC1->CFGR,			ADC_CFGR_CONT); 		// ADC group regular continuous conversion mode
		
	SET_BIT(ADC1->CFGR,			ADC_CFGR_DMAEN); 		// ADC DMA enable
	SET_BIT(ADC1->CFGR,			ADC_CFGR_DMACFG); 	// DMA Circular Mode selected
	SET_BIT(ADC1->CFGR,			ADC_CFGR_AUTDLY); 		// Delayed conversion mode
		
	//SET_BIT(ADC1->CFGR, 		ADC_CFGR_EXTEN_0);		// Hardware trigger detection on the rising edge
	//SET_BIT(ADC1->CFGR,	 		ADC_CFGR_EXTSEL_0 | ADC_CFGR_EXTSEL_3); // EXT9 TIM1_TRGO event
		
	SET_BIT(ADC1->CR, 			ADC_CR_ADEN); 
    // ADC enable
	while(!(READ_BIT(ADC1->ISR, ADC_ISR_ADRDY))){};  // Wait untill ADC is ready to start conversion
			
	SET_BIT(ADC1->CR, 			ADC_CR_ADSTART); 	//Start ADC Convertion
		
	/***** ADC 1 For THERM *****/
	//SET_BIT(ADC1->CFGR,  		ADC_CFGR_JDISCEN); 	// ADC Discontinuous mode on injected channels		
			
	//SET_BIT(ADC1->JSQR,			ADC_JSQR_JSQ1_0);		// Conversion 1 in the injected sequence Channel 1

	//CLEAR_BIT(ADC1->JSQR, 		ADC_JSQR_JL);    		// 00: 1 conversion
			
	/***** ADC 2 For OPAMP2 *****/ 
	CLEAR_BIT(ADC2->CR, 		ADC_CR_ADCALDIF); 	// Single-ended inputs Mode
	SET_BIT(ADC2->CR ,   		ADC_CR_ADCAL); 			// ADC calibration
	while(READ_BIT(ADC2->CR,	ADC_CR_ADCAL)); 		// Wait for end of ADC calibration
		
	SET_BIT(ADC2->SQR1,			ADC_SQR1_SQ1_0 | ADC_SQR1_SQ1_1); // 1st conversion in sequence Channel 3
			
	//ADC2->CFGR	|= ADC_CFGR_DISCEN; 								// ADC group regular sequencer discontinuous mode
    SET_BIT(ADC2->CFGR, 		ADC_CFGR_CONT); 		// ADC group regular continuous conversion mode

	SET_BIT(ADC2->CFGR, 		ADC_CFGR_DMAEN); 		// ADC DMA enable
	SET_BIT(ADC2->CFGR,			ADC_CFGR_DMACFG); 	// DMA Circular Mode selected
	SET_BIT(ADC2->CFGR,			ADC_CFGR_AUTDLY); 		// Delayed conversion mode

	//SET_BIT(ADC2->CFGR, 		ADC_CFGR_EXTEN_0);		// Hardware trigger detection on the rising edge
	//SET_BIT(ADC2->CFGR, 		ADC_CFGR_EXTSEL_0 | ADC_CFGR_EXTSEL_3); // EXT9 TIM1_TRGO event
		
	SET_BIT(ADC2->CR, 			ADC_CR_ADEN); 						// ADC enable
	while(!(READ_BIT(ADC2->ISR, ADC_ISR_ADRDY))){};  // Wait untill ADC is ready to start conversion
			
	SET_BIT(ADC2->CR, ADC_CR_ADSTART); 	//Start ADC Convertion
			

	/***** ADC 3 For OPAMP3 *****/ 
	CLEAR_BIT(ADC3->CR, 		ADC_CR_ADCALDIF); 	// Single-ended inputs Mode
	SET_BIT(ADC3->CR,   		ADC_CR_ADCAL); 			// ADC calibration
	while(READ_BIT(ADC3->CR, 	ADC_CR_ADCAL)); 		// Wait for end of ADC calibration
		
	SET_BIT(ADC3->SQR1, 		ADC_SQR1_SQ1_0); 			// 1st conversion in sequence Channel 1
			
	//ADC3->CFGR	|= ADC_CFGR_DISCEN; 		// ADC group regular sequencer discontinuous mode
	SET_BIT(ADC3->CFGR, 		ADC_CFGR_CONT); 		// ADC group regular continuous conversion mode
				
	SET_BIT(ADC3->CFGR, 		ADC_CFGR_DMAEN); 		// ADC DMA enable
	SET_BIT(ADC3->CFGR,			ADC_CFGR_DMACFG); 	// DMA Circular Mode selected
	SET_BIT(ADC3->CFGR,			ADC_CFGR_AUTDLY); 		// Delayed conversion mode

	//SET_BIT(ADC3->CFGR, 		ADC_CFGR_EXTEN_0);		// Hardware trigger detection on the rising edge
	//SET_BIT(ADC3->CFGR, 		ADC_CFGR_EXTSEL_0 | ADC_CFGR_EXTSEL_3); // EXT9 TIM1_TRGO event
			
	//ADC3->IER		|= ADC_IER_EOCIE; 			// ADC group regular end of unitary conversion interrupt 
			
	SET_BIT(ADC3->CR, 			ADC_CR_ADEN); 						// ADC enable
	while(!(READ_BIT(ADC3->ISR, ADC_ISR_ADRDY))){};  // Wait untill ADC is ready to start conversion
			
	SET_BIT(ADC3->CR, 			ADC_CR_ADSTART); 	//Start ADC Convertion
		

	/***** ADC 4 For OPAMP4 *****/ 
    CLEAR_BIT(ADC4->CR, 		ADC_CR_ADCALDIF); 	// Single-ended inputs Mode
	SET_BIT(ADC4->CR,   		ADC_CR_ADCAL); 			// ADC calibration
	while(READ_BIT(ADC4->CR, 	ADC_CR_ADCAL)); 		// Wait for end of ADC calibration
		
	SET_BIT(ADC4->SQR1,			ADC_SQR1_SQ1_0 | ADC_SQR1_SQ1_1); // 1st conversion in sequence Channel 3
			
	//ADC4->CFGR	|= ADC_CFGR_DISCEN; 								// ADC group regular sequencer discontinuous mode
	SET_BIT(ADC4->CFGR, 		ADC_CFGR_CONT); 		// ADC group regular continuous conversion mode
			
	SET_BIT(ADC4->CFGR, 		ADC_CFGR_DMAEN); 		// ADC DMA enable
	SET_BIT(ADC4->CFGR,			ADC_CFGR_DMACFG); 	// DMA Circular Mode selected
	SET_BIT(ADC4->CFGR,			ADC_CFGR_AUTDLY); 		// Delayed conversion mode
			
	//SET_BIT(ADC4->CFGR, 		ADC_CFGR_EXTEN_0);		// Hardware trigger detection on the rising edge
	//SET_BIT(ADC4->CFGR, 		ADC_CFGR_EXTSEL_0 | ADC_CFGR_EXTSEL_3); // EXT9 TIM1_TRGO event
		
	SET_BIT(ADC4->CR, 			ADC_CR_ADEN); 						// ADC enable
	while(!(READ_BIT(ADC4->ISR, ADC_ISR_ADRDY))){};  // Wait untill ADC is ready to start conversion
			
	SET_BIT(ADC4->CR, ADC_CR_ADSTART); 	//Start ADC Convertion
        
}

void init_dma(TMasADC *MasADC){
	
	// DMA1 and DMA2 clock enable
	SET_BIT(RCC->AHBENR,  			RCC_AHBENR_DMA1EN);
	SET_BIT(RCC->AHBENR,  			RCC_AHBENR_DMA2EN);

	/***** DMA1 Channel 1 ADC1 *****/
	
	WRITE_REG(DMA1_Channel1->CPAR, 	(uint32_t)&ADC1->DR); // Peripheral address
	WRITE_REG(DMA1_Channel1->CMAR, 	(uint32_t)&MasADC->A[0]); // Memory 0 address
	
	WRITE_REG(DMA1_Channel1->CNDTR, CONVERSIONS_COUNT);									 // Number of data
	
	CLEAR_BIT(DMA1_Channel1->CCR,	DMA_CCR_DIR); 			 // Read from peripheral
	SET_BIT(DMA1_Channel1->CCR, 	DMA_CCR_MINC); 		 // Memory increment mode enabled
	
	SET_BIT(DMA1_Channel1->CCR, 	DMA_CCR_PSIZE_1); 	 // Peripheral size = 32-bits
	SET_BIT(DMA1_Channel1->CCR, 	DMA_CCR_MSIZE_1); 	 // Memory size = 32-bits
	
	SET_BIT(DMA1_Channel1->CCR, 	DMA_CCR_PL); 			 // Channel priority level = Very high
	
	SET_BIT(DMA1_Channel1->CCR, 	DMA_CCR_TCIE); 		 // Transfer complete interrupt enable
	
	//SET_BIT(DMA1_Channel1->CCR, 	DMA_CCR_CIRC); 		 // Circular mode 
	
	NVIC_SetPriority(DMA1_Channel1_IRQn, 0); 
	NVIC_EnableIRQ(DMA1_Channel1_IRQn); 
	
	//SET_BIT(DMA1_Channel1->CCR, 	DMA_CCR_EN); 			 // Channel enable
	
	
	/***** DMA2 Channel 1 ADC2 *****/
	
	WRITE_REG(DMA2_Channel1->CPAR, 	(uint32_t)&ADC2->DR); // Peripheral address
	WRITE_REG(DMA2_Channel1->CMAR, 	(uint32_t)&MasADC->B[0]); // Memory 0 address
	
	WRITE_REG(DMA2_Channel1->CNDTR, CONVERSIONS_COUNT);									 // Number of data
	
	CLEAR_BIT(DMA2_Channel1->CCR, 	DMA_CCR_DIR); 			 // Read from peripheral
	SET_BIT(DMA2_Channel1->CCR,  	DMA_CCR_MINC); 		 // Memory increment mode enabled
	
	SET_BIT(DMA2_Channel1->CCR,	 	DMA_CCR_PSIZE_1); 	 // Peripheral size = 32-bits
	SET_BIT(DMA2_Channel1->CCR,	 	DMA_CCR_MSIZE_1); 	 // Memory size = 32-bits
	
	SET_BIT(DMA2_Channel1->CCR,  	DMA_CCR_PL); 			 // Channel priority level = Very high
	//DMA2_Channel1->CCR	|=  DMA_CCR_PL_1; 			 // Channel priority level = High
	
	
	SET_BIT(DMA2_Channel1->CCR, DMA_CCR_TCIE); 		 // Transfer complete interrupt enable
		
	//SET_BIT(DMA2_Channel1->CCR,  	DMA_CCR_CIRC); 		 // Circular mode 
	
	//SET_BIT(DMA2_Channel1->CCR,	 	DMA_CCR_EN); 			 // Channel enable
	
		
	
	/***** DMA2 Channel 5 ADC3 *****/
	
	WRITE_REG(DMA2_Channel5->CPAR, 	(uint32_t)&ADC3->DR); // Peripheral address
	WRITE_REG(DMA2_Channel5->CMAR, 	(uint32_t)&MasADC->C[0]); // Memory 0 address
	
	WRITE_REG(DMA2_Channel5->CNDTR, CONVERSIONS_COUNT);									 // Number of data
	
	CLEAR_BIT(DMA2_Channel5->CCR, 	DMA_CCR_DIR); 			 // Read from peripheral
	SET_BIT(DMA2_Channel5->CCR,  	DMA_CCR_MINC); 		 // Memory increment mode enabled
	
	SET_BIT(DMA2_Channel5->CCR,  	DMA_CCR_PSIZE_1); 	 // Peripheral size = 32-bits
	SET_BIT(DMA2_Channel5->CCR,  	DMA_CCR_MSIZE_1); 	 // Memory size = 32-bits
	
	SET_BIT(DMA2_Channel5->CCR,  	DMA_CCR_PL); 			 // Channel priority level = Very high
	//DMA2_Channel5->CCR	|=  DMA_CCR_PL_1; 		// Channel priority level = High
	
	SET_BIT(DMA2_Channel5->CCR,  	DMA_CCR_TCIE); 		 // Transfer complete interrupt enable
		
	//SET_BIT(DMA2_Channel5->CCR,  	DMA_CCR_CIRC); 		 // Circular mode 
	
	//SET_BIT(DMA2_Channel5->CCR,  	DMA_CCR_EN); 			 // Channel enable
	
	
	/***** DMA2 Channel 2 ADC4 *****/
	
	WRITE_REG(DMA2_Channel2->CPAR, 	(uint32_t)&ADC4->DR);   // Peripheral address
	WRITE_REG(DMA2_Channel2->CMAR, 	(uint32_t)&MasADC->VDC[0]); // Memory 0 address
	
	WRITE_REG(DMA2_Channel2->CNDTR, CONVERSIONS_COUNT);									 // Number of data
	
	CLEAR_BIT(DMA2_Channel2->CCR, 	DMA_CCR_DIR); 			 // Read from peripheral
	SET_BIT(DMA2_Channel2->CCR,  	DMA_CCR_MINC); 		 // Memory increment mode enabled
	
	SET_BIT(DMA2_Channel2->CCR,  	DMA_CCR_PSIZE_1); 	 // Peripheral size = 32-bits
	SET_BIT(DMA2_Channel2->CCR,  	DMA_CCR_MSIZE_1); 	 // Memory size = 32-bits
	
	SET_BIT(DMA2_Channel2->CCR,  	DMA_CCR_PL); 			 // Channel priority level = Very high
	//DMA2_Channel2->CCR	|=  DMA_CCR_PL_0;			 // Channel priority level = Medium
	
	SET_BIT(DMA2_Channel2->CCR,  	DMA_CCR_TCIE); 		 // Transfer complete interrupt enable
		
	//SET_BIT(DMA2_Channel2->CCR,  	DMA_CCR_CIRC); 		 // Circular mode 
	
	//SET_BIT(DMA2_Channel2->CCR,  	DMA_CCR_EN); 			 // Channel enable

}

