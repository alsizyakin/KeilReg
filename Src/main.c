/*
  ******************************************************************************
  * @file    main.c
  * @brief   CIPOS test
  ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx.h"                  
#include "defs.h"
#include "VectCalc.h"
#include "inits.h"
#include "motorcontrol.h"
#include "delay.h"
#include "Tims.h"
#include "uart.h"


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void stoppedState(void);
void startingState(void);
void runningState(void);

float Therm = 0;
TMasADC MasADC; 

uint8_t uart_rx_data[UART_DATA_SIZE];
uint8_t uart_tx_data[UART_DATA_SIZE];

extern TFlags Flags;
extern TMotorCurrents MotorCurrents; 
extern TMotorVoltage MotorVoltage; 
extern TMRASVar MRASVar;

int fltCnt = 0;
/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void){
	
	SET_BIT(DBGMCU->APB2FZ,  DBGMCU_APB2_FZ_DBG_TIM1_STOP);
 
	for(int i = 0; i < UART_DATA_SIZE; i++){
		uart_tx_data[i] = i;
	}

	/* Configure the system clock to 72 MHz */
	SystemClock_Config();
	
	__disable_irq();
	
	Delay(650);

	init_ports();
	init_opamp();
	init_comp();
	//init_dac();
	init_adc();
	init_awd();
	init_dma(&MasADC);
	init_uart(&uart_rx_data[0], &uart_tx_data[0]); 
	init_timer1();
	
	initVariables();
	
	SET_BIT(USART1->ICR, USART_ICR_IDLECF);
		
	Flags.CurrentMotor = 0;
	Flags.MeasReady = 0;
	Flags.OverTemp = 0;
	Flags.MotorDetection = 0;
	Flags.CurrentState = STATE_STOPPED;
	
 	NTC_RELAY_OFF();
	
	ITRIP_OFF();
	
	__enable_irq();
	
	TIM1_ON();
	PWM_ON();


	while (1){
		// State Machine
		switch(Flags.CurrentState){
			
			case STATE_STOPPED:
				LED_OFF(LED_ALL);
				LED_ON(LED_1);
				if (Flags.MeasReady == 1){
					Flags.MeasReady = 0;
					stoppedState();
				}
				break;

			case STATE_STARTING:
				LED_OFF(LED_ALL);
				LED_ON(LED_2);
				if (Flags.MeasReady == 1){
					Flags.MeasReady = 0;
					startingState();
				}
				break;
				
			case STATE_RUNNING:
				LED_OFF(LED_ALL);
				LED_ON(LED_3);
				if (Flags.MeasReady == 1){
					Flags.MeasReady = 0;
					runningState();
				}
				break;
									
			case STATE_FAULT:
				LED_ON(LED_ALL);
				stopMotor();
				if (fltCnt++ > 50000){
					fltCnt = 0;
				    Flags.CurrentState = STATE_STOPPED;
				}
				break;
					
			case STATE_STOPPING:
				stopMotor();
				Flags.CurrentState = STATE_STOPPED;
				break;
					
			default:
				break;
		}	
				
	}
	
}

void stoppedState(void){
//#define SLOWVOLTDETECT // turn on the system when DC-voltage raises slowly because of capacitors are charged
	
	static uint16_t nb_delay = 0;
#ifdef SLOWVOLTDETECT 
	static float old_volts = 0;
	float dudcdt = 100.F;  //DC-link voltage increasing rate
	static int volt_counter = 0;
#endif
	measureCurrentBase(&MasADC, &MotorCurrents);
	getUdc(&MasADC, &MotorVoltage);
	
	if(CHECK_TEMP()){
		LED_ON(LED_ALL);
		LED_ON(BUTTON_2);
		Flags.OverTemp = 1;
		Flags.CurrentState = STATE_FAULT;
		DAC->DHR12R1 = TEMP_MIN;
	}
	
	if(!CHECK_TEMP() && Flags.OverTemp){
		Flags.OverTemp = 0;
		DAC->DHR12R1 = TEMP_MAX;
	}
	
	
#ifndef SLOWVOLTDETECT	
	if ((MotorVoltage.udc_izm > START_VOLTS) && (Flags.OverTemp == 0)){
		if((nb_delay++) >= 10000){
			
			NTC_RELAY_ON();
			if (nb_delay > 15000){
				Flags.CurrentState = STATE_STARTING;
				nb_delay = 0;
			}
		}
	}
#else	
	
	if (volt_counter++ > 500){
		dudcdt = MotorVoltage.udc_izm - old_volts;
		if ((Flags.OverTemp == 0) && (dudcdt < 0.025F) && (MotorVoltage.udc_izm > STOP_VOLTS)) {
			if((nb_delay++)>= 5){
				nb_delay = 0;
			
				LED_ON(BUTTON_2);
			
				NTC_RELAY_ON();
				Flags.CurrentState = STATE_STARTING;
			}
		} else {
			nb_delay = 0;
			LED_OFF(BUTTON_2);
			if (MotorVoltage.udc_izm < STOP_VOLTS){
				NTC_RELAY_OFF();
			}
		}
		old_volts = MotorVoltage.udc_izm;
		volt_counter = 0;
	}
	
#endif	
	
}

void startingState(void){

	getPhaseCurrents(&MasADC, &MotorCurrents);
	getUdc(&MasADC, &MotorVoltage);

	transormClarkeCurrent(&MotorCurrents);

	startMotor();	

	if (MotorVoltage.udc_izm < STOP_VOLTS){
		Flags.CurrentState = STATE_STOPPING;
	}
			
	if (CHECK_TEMP()){
		LED_ON(LED_ALL);
		LED_ON(BUTTON_2);
		Flags.OverTemp = 1;
		Flags.CurrentState = STATE_FAULT;
	}	

}

void runningState(void){
	
	static uint16_t n_delay = 0;
	
	getPhaseCurrents(&MasADC, &MotorCurrents);
	getUdc(&MasADC, &MotorVoltage);

	transormClarkeCurrent(&MotorCurrents);

	runFullyControlled();
		
	/*if((MRASVar.wr > 1000) && (MotorCurrents.iqr < 4)){
		if((n_delay++) >= 10000){
					n_delay = 0;
					LED_ON(LED_ALL);
					Flags.CurrentState = STATE_FAULT;
		}
	}*/
		
	if(MotorVoltage.udc_izm < STOP_VOLTS){
		Flags.CurrentState = STATE_STOPPING;
	}
			
	if(CHECK_TEMP()){
		LED_ON(LED_ALL);
		LED_ON(BUTTON_2);
		Flags.OverTemp = 1;
		Flags.CurrentState = STATE_FAULT;
	}			

}



/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 72000000
  *            HCLK(Hz)                       = 72000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 2
  *            APB2 Prescaler                 = 1
  *            HSE Frequency(Hz)              = 8000000
  *            PREDIV                         = RCC_PREDIV_DIV1 (1)
  *            PLLMUL                         = RCC_PLL_MUL9 (9)
  *            Flash Latency(WS)              = 2
  * @param  None
  * @retval None
  */
void SystemClock_Config(void){
	/* *///HSI Oscillator already ON after system reset, activate PLL with HSI as source

  
	CLEAR_BIT(RCC->CR,RCC_CR_PLLON);    // turn off PLL
  SET_BIT(RCC->CR,RCC_CR_HSEON);//Enable HSE
  while (!READ_BIT(RCC->CR, RCC_CR_HSERDY));
	
  SET_BIT(RCC->CFGR, RCC_CFGR_PLLSRC_HSE_PREDIV); // HSE is source for PLL
  SET_BIT(RCC->CFGR, RCC_CFGR_PLLMUL9);// PLL multiplyer x9
	SET_BIT(RCC->CR,  RCC_CR_PLLON);    // turn on PLL
	while (!READ_BIT(RCC->CR, RCC_CR_PLLRDY)){};
	//SystemCoreClock = 72000000;
	
  
	SET_BIT(FLASH -> ACR, FLASH_ACR_LATENCY_2);
  
  
	CLEAR_BIT(RCC -> CFGR, RCC_CFGR_PPRE1);   //PLLP for BUS divider (2)
	SET_BIT(RCC -> CFGR, RCC_CFGR_PPRE1_2);
  
  
   //RCC->CFGR2 |= RCC_CFGR2_ADCPRE12_4;   //PLLP for BUS divider (2)
  
   //RCC->CFGR2 |= RCC_CFGR2_ADCPRE34_4;   //PLLP for BUS divider (2)

  
	CLEAR_BIT(RCC->CFGR, RCC_CFGR_SW);
	SET_BIT(RCC->CFGR, RCC_CFGR_SW_PLL);// PLL is system bus
	while((READ_BIT(RCC->CFGR, RCC_CFGR_SWS)) != RCC_CFGR_SWS_PLL);// wait for sw ready
		
	SystemCoreClockUpdate();	
}

