
#include "stm32f3xx.h"
#include "defs.h"
#include "motorcontrol.h"
#include "uart.h"

int faultCounterAdc1, faultCounterAdc2, faultCounterAdc3;
/**********************************************************************
Timer1 PWM Interrupt Service Routine()
**********************************************************************/
void TIM1_UP_TIM16_IRQHandler(void){
    
	DMA_ENABLE();
	//USART1->TDR = 0xAB;

	TIM1_IF_CLEAR();	//clear TIM1 interrupt flag
	faultCounterAdc1 = 0;
	faultCounterAdc2 = 0;
	faultCounterAdc3 = 0;
	 
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

// USART1 TX
void DMA1_Channel4_IRQHandler(void){
	SET_BIT(DMA1->IFCR, DMA_IFCR_CTCIF4);
	
	//RESET DMA
	CLEAR_BIT(DMA1_Channel4->CCR, 	DMA_CCR_EN);
	WRITE_REG(DMA1_Channel4->CNDTR, UART_DATA_SIZE);
	//SET_BIT(DMA1_Channel4->CCR, 	DMA_CCR_EN);
}

// USART1 RX
void DMA1_Channel5_IRQHandler(void){
	
	extern uint8_t  uart_rx_data[];
	extern uint8_t  uart_tx_data[];
	extern float wTarg;
	
	uint32_t command = 0;
	
	SET_BIT(DMA1->IFCR, DMA_IFCR_CTCIF5);
	
	if(uart_rx_data[0] == 'c'){
		if(uart_rx_data[1] == 's' && uart_rx_data[2] == 't' && uart_rx_data[3] == 'r'){
			// Start Motor
			
		} else if(uart_rx_data[1] == 's' && uart_rx_data[2] == 't' && uart_rx_data[3] == 'p'){
			// Stop Motor
		
		}
	} else if(uart_rx_data[0] == 's'){
		float tmp = 0;
		tmp += (uart_rx_data[1]<<8) + (uart_rx_data[2]);
		wTarg = tmp;
	}
	
	//RESET DMA
	CLEAR_BIT(DMA1_Channel5->CCR, 	DMA_CCR_EN);
	WRITE_REG(DMA1_Channel5->CNDTR, UART_DATA_SIZE);
	SET_BIT(DMA1_Channel5->CCR, 	DMA_CCR_EN);
	
}

/**********************************************************************
ADC1-4  Interrupt Service Routines()
**********************************************************************/
void ADC1_2_IRQHandler(void){
	
	if (READ_BIT(ADC1->ISR,	ADC_ISR_AWD1)){
		SET_BIT(ADC1->ISR,	ADC_ISR_AWD1);
		if (faultCounterAdc1 ++ > 2) {
			/*stopMotor();
			
			Flags.CurrentState = STATE_FAULT;
			LED_ON(BUTTON_1);*/
		}
	}
	
	if (READ_BIT(ADC2->ISR,	ADC_ISR_AWD1)){
		SET_BIT(ADC2->ISR,	ADC_ISR_AWD1);
		if (faultCounterAdc2 ++ > 2) {
			/*stopMotor();
			Flags.CurrentState = STATE_FAULT;
			LED_ON(BUTTON_1);*/
		}
	}
	
}

void ADC3_IRQHandler(void){
	
	if (READ_BIT(ADC3->ISR,	ADC_ISR_AWD1)){
		SET_BIT(ADC3->ISR,	ADC_ISR_AWD1);
		if (faultCounterAdc3 ++ > 2){
			/*stopMotor();
			Flags.CurrentState = STATE_FAULT;
			LED_ON(BUTTON_1);*/
		}
	}
	
}

void ADC4_IRQHandler(void){
	
	if (READ_BIT(ADC4->ISR,	ADC_ISR_AWD1)){
		SET_BIT(ADC4->ISR,	ADC_ISR_AWD1);
		/*stopMotor();
		Flags.CurrentState = STATE_FAULT;
		LED_ON(BUTTON_1);
		LED_ON(BUTTON_2);*/
	}
}



