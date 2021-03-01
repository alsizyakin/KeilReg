
/* Includes ------------------------------------------------------------------*/
                  
#include "defs.h"
#include "uart.h"

void init_uart(uint8_t* rx_data, uint8_t* tx_data){
	
	SET_BIT(RCC->APB2ENR, RCC_APB2ENR_USART1EN);
	
	/*
	 * Init Usart Pins
	 */
	
	SET_BIT(RCC->AHBENR, 			RCC_AHBENR_GPIOBEN); 
	SET_BIT(GPIOB->MODER, 		GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1);
	CLEAR_BIT(GPIOB->OTYPER, 	GPIO_OTYPER_OT_6 | GPIO_OTYPER_OT_7);
	SET_BIT(GPIOB->OSPEEDR, 	GPIO_OSPEEDER_OSPEEDR6 | GPIO_OSPEEDER_OSPEEDR7);
	CLEAR_BIT(GPIOB->PUPDR, 	GPIO_PUPDR_PUPDR6 | GPIO_PUPDR_PUPDR7);
  
	MODIFY_REG(GPIOB->AFR[0],	GPIO_AFRL_AFRL6_Msk,(7 << GPIO_AFRL_AFRL6_Pos));
	MODIFY_REG(GPIOB->AFR[0],	GPIO_AFRL_AFRL7_Msk,(7 << GPIO_AFRL_AFRL7_Pos));
	
	/*
	 * Init Usart 
	 */
	
	// Word length = 1 Start bit, 8 data bits, n stop bits
	CLEAR_BIT(USART1->CR1, USART_CR1_M);	
	CLEAR_BIT(USART1->CR1, USART_CR1_M0);	
	
	// STOP bits = 1 stop bit
	CLEAR_BIT(USART1->CR2, USART_CR2_STOP);
	
	// Baud Rate = 9600
	WRITE_REG(USART1->BRR, 72000000/9600);
	
	SET_BIT(USART1->CR3, USART_CR3_DMAR);
	
	SET_BIT(USART1->CR3, USART_CR3_DMAT);
	
	
	/*
	 * Init DMA
	 */
	 
	/***** DMA1 Channel 4 USART1_TX *****/
	
	WRITE_REG(DMA1_Channel4->CPAR, 	(uint32_t)&USART1->TDR); // Peripheral address
	WRITE_REG(DMA1_Channel4->CMAR, 	(uint32_t)tx_data); // Memory 0 address
	
	WRITE_REG(DMA1_Channel4->CNDTR, UART_DATA_SIZE);									 // Number of data
	
	SET_BIT(DMA1_Channel4->CCR, 	DMA_CCR_DIR); 			 // Read from memory
	SET_BIT(DMA1_Channel4->CCR,  	DMA_CCR_MINC); 		 // Memory increment mode enabled
	
	CLEAR_BIT(DMA1_Channel4->CCR,  	DMA_CCR_PSIZE); 	 // Peripheral size = 8-bits
	CLEAR_BIT(DMA1_Channel4->CCR,  	DMA_CCR_MSIZE); 	 // Memory size = 8-bits
	
	DMA1_Channel4->CCR	|=  DMA_CCR_PL_1; 		// Channel priority level = High
	
	SET_BIT(DMA1_Channel4->CCR,  	DMA_CCR_TCIE); 		 // Transfer complete interrupt enable
	
	NVIC_SetPriority(DMA1_Channel4_IRQn, 5); 
	NVIC_EnableIRQ(DMA1_Channel4_IRQn); 
	
	//SET_BIT(DMA1_Channel4->CCR,  	DMA_CCR_EN); 			 // Channel enable
	
	/***** DMA1 Channel 5 USART1_RX *****/
	
	WRITE_REG(DMA1_Channel5->CPAR, 	(uint32_t)&USART1->RDR); // Peripheral address
	WRITE_REG(DMA1_Channel5->CMAR, 	(uint32_t)rx_data); // Memory 0 address
	
	WRITE_REG(DMA1_Channel5->CNDTR, UART_DATA_SIZE);									 // Number of data
	
	CLEAR_BIT(DMA1_Channel5->CCR, 	DMA_CCR_DIR); 			 // Read from peripheral
	SET_BIT(DMA1_Channel5->CCR,  	DMA_CCR_MINC); 		 // Memory increment mode enabled
	
	CLEAR_BIT(DMA1_Channel5->CCR,  	DMA_CCR_PSIZE); 	 // Peripheral size = 8-bits
	CLEAR_BIT(DMA1_Channel5->CCR,  	DMA_CCR_MSIZE); 	 // Memory size = 8-bits
	
	DMA1_Channel5->CCR	|=  DMA_CCR_PL_1; 		// Channel priority level = High
	
	SET_BIT(DMA1_Channel5->CCR,  	DMA_CCR_TCIE); 		 // Transfer complete interrupt enable
	
	//SET_BIT(DMA1_Channel5->CCR,  	DMA_CCR_EN); 			 // Channel enable
	
	NVIC_SetPriority(DMA1_Channel5_IRQn, 5); 
	NVIC_EnableIRQ(DMA1_Channel5_IRQn);
	
	SET_BIT(USART1->CR1, USART_CR1_UE); 						// Enable Uart
	
		// Reciever enable
	SET_BIT(USART1->CR1, USART_CR1_RE);
	
	// Transmiter enable
	SET_BIT(USART1->CR1, USART_CR1_TE);
	
	SET_BIT(DMA1_Channel5->CCR,  	DMA_CCR_EN); 			 // DMA for uart rx enable
	SET_BIT(DMA1_Channel4->CCR,  	DMA_CCR_EN); 		
	
}



