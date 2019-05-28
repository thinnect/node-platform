#include "spi.h"
#include <stdio.h>
#include "em_cmu.h"
#include "em_gpio.h"
#include "em_usart.h"
#include "mask.h"

#include "retargetspiconfig.h"


void RETARGET_SpiInit(){
	USART_InitSync_TypeDef init = USART_INITSYNC_DEFAULT;
	init.msbf = true; // MSB first transmission for SPI compatibility

	CMU_ClockEnable(RETARGET_SPI_CLOCK, true);

	//RETARGET_SPI_UART->CTRL = /*USART_CTRL_CLKPHA | USART_CTRL_CLKPOL |*/ USART_CTRL_SYNC | USART_CTRL_MSBF;
	//RETARGET_SPI_UART->FRAME = mask32(_USART_FRAME_DATABITS_MASK, 5) | mask32(_USART_FRAME_PARITY_MASK, 0) | mask32(_USART_FRAME_STOPBITS_MASK, 1);
	//RETARGET_SPI_UART->CLKDIV = 256LLU * ((38400000LLU / (2LLU * 1000000LLU)) - 1LLU);

	CMU_ClockEnable(cmuClock_GPIO, true);

	GPIO_PinModeSet(RETARGET_SPI_MISO_PORT, RETARGET_SPI_MISO_PIN, gpioModeInput, 0);
	GPIO_PinModeSet(RETARGET_SPI_MOSI_PORT, RETARGET_SPI_MOSI_PIN, gpioModePushPull, 0);
	GPIO_PinModeSet(RETARGET_SPI_SCK_PORT, RETARGET_SPI_SCK_PIN, gpioModePushPull, 0);
	GPIO_PinModeSet(RETARGET_SPI_CS_PORT, RETARGET_SPI_CS_PIN, gpioModePushPull, 1);

	#if defined(GPIO_USART_ROUTEEN_TXPEN) // Series 2
		GPIO->USARTROUTE[RETARGET_SPI_UART_INDEX].RXROUTE =
		                               (RETARGET_SPI_MISO_PORT << _GPIO_USART_RXROUTE_PORT_SHIFT)
		                             | (RETARGET_SPI_MISO_PIN << _GPIO_USART_RXROUTE_PIN_SHIFT);
		GPIO->USARTROUTE[RETARGET_SPI_UART_INDEX].TXROUTE =
		                               (RETARGET_SPI_MOSI_PORT << _GPIO_USART_TXROUTE_PORT_SHIFT)
		                             | (RETARGET_SPI_MOSI_PIN << _GPIO_USART_TXROUTE_PIN_SHIFT);
		GPIO->USARTROUTE[RETARGET_SPI_UART_INDEX].CLKROUTE =
		                               (RETARGET_SPI_SCK_PORT << _GPIO_USART_CLKROUTE_PORT_SHIFT)
		                             | (RETARGET_SPI_SCK_PIN << _GPIO_USART_CLKROUTE_PIN_SHIFT);

		// Enable USART interface pins
		GPIO->USARTROUTE[RETARGET_SPI_UART_INDEX].ROUTEEN =
		    GPIO_USART_ROUTEEN_RXPEN |  // MISO
		    GPIO_USART_ROUTEEN_TXPEN |  // MOSI
		    GPIO_USART_ROUTEEN_CLKPEN;  // CLK
	#elif defined(USART_ROUTEPEN_RXPEN)  // Series 1
		mask32_set(RETARGET_SPI_UART->ROUTELOC0, _USART_ROUTELOC0_RXLOC_MASK, RETARGET_SPI_RX_LOCATION);
		mask32_set(RETARGET_SPI_UART->ROUTELOC0, _USART_ROUTELOC0_TXLOC_MASK, RETARGET_SPI_TX_LOCATION);
		mask32_set(RETARGET_SPI_UART->ROUTELOC0, _USART_ROUTELOC0_CLKLOC_MASK, RETARGET_SPI_CLK_LOCATION);

		GPIO_PinModeSet(gpioPortC, 10, gpioModePushPull, 1);
		GPIO_PinModeSet(gpioPortC, 11, gpioModePushPull, 1);
		GPIO_PinModeSet(gpioPortC, 9, gpioModePushPull, 1);

		RETARGET_SPI_UART->ROUTEPEN |= USART_ROUTEPEN_TXPEN | USART_ROUTEPEN_RXPEN | /*USART_ROUTEPEN_CSPEN |*/ USART_ROUTEPEN_CLKPEN;
	#else
		#error Series 0 not supported!
	#endif

	USART_InitSync(RETARGET_SPI_UART, &init);
	USART_Enable(RETARGET_SPI_UART, usartEnable);
}

void RETARGET_SpiDisable(){
	USART_Enable(RETARGET_SPI_UART, usartDisable);
	//GPIO_PinModeSet(RETARGET_SPI_MISO_PORT, RETARGET_SPI_MISO_PIN, gpioModeDisabled, 0);
	//GPIO_PinModeSet(RETARGET_SPI_MOSI_PORT, RETARGET_SPI_MOSI_PIN, gpioModeDisabled, 0);
	//GPIO_PinModeSet(RETARGET_SPI_SCK_PORT, RETARGET_SPI_SCK_PIN, gpioModeDisabled, 0);
	//GPIO_PinModeSet(RETARGET_SPI_CS_PORT, RETARGET_SPI_CS_PIN, gpioModeDisabled, 0);
}

void spi_cs(uint8_t active){
	if(active){
		GPIO_PinOutClear(RETARGET_SPI_CS_PORT, RETARGET_SPI_CS_PIN);
	}else{
		GPIO_PinOutSet(RETARGET_SPI_CS_PORT, RETARGET_SPI_CS_PIN);
	}
}

int RETARGET_SpiTransferHalf(int cs, const void *out, int32_t out_len, void *in, int32_t in_len){
	int32_t i;
	if((cs != 0) || (out_len < 0) || (in_len < 0))return(-1);
	RETARGET_SPI_UART->CMD = USART_CMD_CLEARTX | USART_CMD_CLEARRX;
	spi_cs(1);
	if(out_len){
		RETARGET_SPI_UART->CMD = USART_CMD_TXTRIDIS;
		for(i = 0; i < out_len; i++){
			USART_SpiTransfer(RETARGET_SPI_UART, ((uint8_t *)out)[i]);
		}
	}
	RETARGET_SPI_UART->CMD = USART_CMD_TXTRIEN;
	for(i = 0; i < in_len; i++){
		((uint8_t *)in)[i] = USART_SpiTransfer(RETARGET_SPI_UART, 0xFF);
	}
	spi_cs(0);
	return(0);
}
