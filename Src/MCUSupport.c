#include "MCUSupport.h"
#include "stm32f3xx.h" 

void SetTim1Freeze(void){
	SET_BIT(DBGMCU->APB2FZ,  DBGMCU_APB2_FZ_DBG_TIM1_STOP);
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
