
#ifndef UART
#define UART
#include "stm32f3xx.h"

#define UART_DATA_SIZE 4

void init_uart(uint8_t* rx_data, uint8_t* tx_data);

#endif

