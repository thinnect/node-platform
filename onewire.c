/*
 * Universal EFR32 onewire.
 *
 * NB: Series 0 has not been tested.
 */
#include "onewire.h"

#include <stdio.h>
#include "em_cmu.h"
#include "em_gpio.h"
#include "em_usart.h"

#include "onewire_config.h"

void onewire_init(){
	USART_TypeDef *usart = ONEWIRE_UART;
	USART_InitAsync_TypeDef init = USART_INITASYNC_DEFAULT;

	CMU_ClockEnable(ONEWIRE_UART_CLOCK, true);

	init.enable = usartDisable;
	init.baudrate = 9600;
	init.parity = usartNoParity;
	USART_InitAsync(usart, &init);

#if defined(GPIO_USART_ROUTEEN_TXPEN) // Series 2
	GPIO->USARTROUTE[ONEWIRE_UART_INDEX].ROUTEEN = GPIO_USART_ROUTEEN_TXPEN | GPIO_USART_ROUTEEN_RXPEN;
	GPIO->USARTROUTE[ONEWIRE_UART_INDEX].TXROUTE = (ONEWIRE_PORT << _GPIO_USART_TXROUTE_PORT_SHIFT) | (ONEWIRE_PIN << _GPIO_USART_TXROUTE_PIN_SHIFT);
	GPIO->USARTROUTE[ONEWIRE_UART_INDEX].RXROUTE = (ONEWIRE_PORT << _GPIO_USART_RXROUTE_PORT_SHIFT) | (ONEWIRE_PIN << _GPIO_USART_RXROUTE_PIN_SHIFT);
#elif defined(USART_ROUTEPEN_RXPEN)  // Series 1
	usart->ROUTEPEN = USART_ROUTEPEN_RXPEN | USART_ROUTEPEN_TXPEN;
	usart->ROUTELOC0 = (usart->ROUTELOC0 & ~(_USART_ROUTELOC0_TXLOC_MASK|_USART_ROUTELOC0_RXLOC_MASK) )
	    | (ONEWIRE_TX_LOCATION << _USART_ROUTELOC0_TXLOC_SHIFT)
	    | (ONEWIRE_RX_LOCATION << _USART_ROUTELOC0_RXLOC_SHIFT);
#else // Series 0
		usart->ROUTE = USART_ROUTE_RXPEN | USART_ROUTE_TXPEN | ONEWIRE_LOCATION;
#endif
	// GPIO clock?

	GPIO_PinModeSet(gpioPortB, 0, gpioModeWiredAndPullUp, 1);

	ONEWIRE_UART->CMD = USART_CMD_CLEARTX | USART_CMD_CLEARRX;
	ONEWIRE_UART->CMD = USART_CMD_TXEN | USART_CMD_RXEN;
	//USART_Enable(usart, usartEnable);
}

void onewire_deinit(){
	CMU_ClockEnable(ONEWIRE_UART_CLOCK, false);
	GPIO_PinModeSet(gpioPortB, 0, gpioModeDisabled, 0);
}

uint8_t onewire_reset(){
	volatile uint32_t i;
	ONEWIRE_UART->CLKDIV = 256LLU * ((ONEWIRE_PERIPHERAL_CLOCK / (16LLU * 9600LLU)) - 1LLU);
	ONEWIRE_UART->CMD = USART_CMD_CLEARTX | USART_CMD_CLEARRX;
	(void)ONEWIRE_UART->RXDATA;
	ONEWIRE_UART->TXDATA = 0xF0;
	while(!(ONEWIRE_UART->STATUS & USART_STATUS_TXBL));
	for(i = 0; i < 65536; i++){
		if((ONEWIRE_UART->STATUS & USART_STATUS_RXDATAV)){
			// printf("R: %02X\n", (unsigned int)ONEWIRE_UART->RXDATA);
			if((ONEWIRE_UART->RXDATA & 0xFF) != 0xF0)return(1);
			return(0);
		}
	}
	return(0);
}

uint8_t onewire_read(void){
	volatile uint32_t i;
	ONEWIRE_UART->CLKDIV = 256LLU * ((ONEWIRE_PERIPHERAL_CLOCK / (16LLU * 115200LLU)) - 1LLU);
	ONEWIRE_UART->CMD = USART_CMD_CLEARTX | USART_CMD_CLEARRX;
	(void)ONEWIRE_UART->RXDATA;
	ONEWIRE_UART->TXDATA = 0xFF;
	while(!(ONEWIRE_UART->STATUS & USART_STATUS_TXBL));
	for(i = 0; i < 65536; i++){
		if((ONEWIRE_UART->STATUS & USART_STATUS_RXDATAV)){
			// printf("R: %02X\n", (unsigned int)ONEWIRE_UART->RXDATA);
			if((ONEWIRE_UART->RXDATA & 0xFF) == 0xFF)return(1);
			return(0);
		}
	}
	return(0);
}

void onewire_write(uint8_t bit){
	volatile uint32_t i;
	ONEWIRE_UART->CLKDIV = 256LLU * ((ONEWIRE_PERIPHERAL_CLOCK / (16LLU * 115200LLU)) - 1LLU);
	ONEWIRE_UART->CMD = USART_CMD_CLEARTX | USART_CMD_CLEARRX;
	(void)ONEWIRE_UART->RXDATA;
	if(bit == 0){
		ONEWIRE_UART->TXDATA = 0x00;
	}else{
		ONEWIRE_UART->TXDATA = 0xFF;
	}
	while(!(ONEWIRE_UART->STATUS & USART_STATUS_TXBL));
	for(i = 0; i < 65536; i++){
		if((ONEWIRE_UART->STATUS & USART_STATUS_RXDATAV)){
			// printf("R: %02X\n", (unsigned int)ONEWIRE_UART->RXDATA);
			return;
		}
	}
}
