/*
 * SiLabs RETARGET_Serial inspired SPI retargeting solution.
 *
 * Copyright Thinnect Inc. 2019
 * @license MIT
 * @author Raido Pahtma, Veiko Rütter
 */

#include "retargetspi.h"
#include "retargetspiconfig.h"

#include <stdio.h>
#include "em_cmu.h"
#include "em_gpio.h"
#include "em_usart.h"

void RETARGET_SpiInit() {
	USART_InitSync_TypeDef init = USART_INITSYNC_DEFAULT;
	init.msbf = true;
	init.enable = usartDisable;

	CMU_ClockEnable(cmuClock_GPIO, true);
	GPIO_PinModeSet(RETARGET_SPI_CS_PORT, RETARGET_SPI_CS_PIN, gpioModePushPull, 1);
#ifdef RETARGET_SPI_CS1_PORT
	GPIO_PinModeSet(RETARGET_SPI_CS1_PORT, RETARGET_SPI_CS1_PIN, gpioModePushPull, 1);
#endif
#ifdef RETARGET_SPI_CS2_PORT
	GPIO_PinModeSet(RETARGET_SPI_CS2_PORT, RETARGET_SPI_CS2_PIN, gpioModePushPull, 1);
#endif
	GPIO_PinModeSet(RETARGET_SPI_MISO_PORT, RETARGET_SPI_MISO_PIN, gpioModeInput, 1);
	GPIO_PinModeSet(RETARGET_SPI_MOSI_PORT, RETARGET_SPI_MOSI_PIN, gpioModePushPull, 1);
	GPIO_PinModeSet(RETARGET_SPI_SCK_PORT, RETARGET_SPI_SCK_PIN, gpioModePushPull, 1);

	CMU_ClockEnable(RETARGET_SPI_CLOCK, true);
	USART_InitSync(RETARGET_SPI_UART, &init);

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
		RETARGET_SPI_UART->ROUTELOC0 = (RETARGET_SPI_CLK_LOC)
									 | (RETARGET_SPI_TX_LOC)
									 | (RETARGET_SPI_RX_LOC);

		// Enable USART interface pins
		RETARGET_SPI_UART->ROUTEPEN |= USART_ROUTEPEN_TXPEN | USART_ROUTEPEN_RXPEN | USART_ROUTEPEN_CLKPEN;
	#else
		#error Series 0 not supported!
	#endif

	USART_Enable(RETARGET_SPI_UART, usartEnable);
}

void RETARGET_SpiDisable() {
	USART_Enable(RETARGET_SPI_UART, usartDisable);
	//GPIO_PinModeSet(RETARGET_SPI_MISO_PORT, RETARGET_SPI_MISO_PIN, gpioModeDisabled, 0);
	//GPIO_PinModeSet(RETARGET_SPI_MOSI_PORT, RETARGET_SPI_MOSI_PIN, gpioModeDisabled, 0);
	//GPIO_PinModeSet(RETARGET_SPI_SCK_PORT, RETARGET_SPI_SCK_PIN, gpioModeDisabled, 0);
	//GPIO_PinModeSet(RETARGET_SPI_CS_PORT, RETARGET_SPI_CS_PIN, gpioModeDisabled, 0);
}

void spi_cs(int cs, uint8_t active) {
	if((cs == 0) && active) {
		GPIO_PinOutClear(RETARGET_SPI_CS_PORT, RETARGET_SPI_CS_PIN);
	}else{
		GPIO_PinOutSet(RETARGET_SPI_CS_PORT, RETARGET_SPI_CS_PIN);
	}
#ifdef RETARGET_SPI_CS1_PORT
	if((cs == 1) && active) {
		GPIO_PinOutClear(RETARGET_SPI_CS1_PORT, RETARGET_SPI_CS1_PIN);
	}else{
		GPIO_PinOutSet(RETARGET_SPI_CS1_PORT, RETARGET_SPI_CS1_PIN);
	}
#endif
#ifdef RETARGET_SPI_CS2_PORT
	if((cs == 2) && active) {
		GPIO_PinOutClear(RETARGET_SPI_CS2_PORT, RETARGET_SPI_CS2_PIN);
	}else{
		GPIO_PinOutSet(RETARGET_SPI_CS2_PORT, RETARGET_SPI_CS2_PIN);
	}
#endif
}

int RETARGET_SpiTransfer(int cs, const void *out, void *in, int32_t len) {
	if(len < 1)return(len);
	spi_cs(cs, 1);
	for(int32_t i = 0; i < len; i++) {
		((uint8_t *)in)[i] = USART_SpiTransfer(RETARGET_SPI_UART, ((uint8_t *)out)[i]);
	}
	spi_cs(cs, 0);
	return 0;
}

int RETARGET_SpiTransferHalf(int cs, const void *out, int32_t out_len, void *in, int32_t in_len) {
	int32_t i;
	if((out_len < 0) || (in_len < 0))return(-1);
	RETARGET_SPI_UART->CMD = USART_CMD_CLEARTX | USART_CMD_CLEARRX;
	spi_cs(cs, 1);
	if(out_len) {
		RETARGET_SPI_UART->CMD = USART_CMD_TXTRIDIS;
		for(i = 0; i < out_len; i++) {
			USART_SpiTransfer(RETARGET_SPI_UART, ((uint8_t *)out)[i]);
		}
	}
	RETARGET_SPI_UART->CMD = USART_CMD_TXTRIEN;
	for(i = 0; i < in_len; i++) {
		((uint8_t *)in)[i] = USART_SpiTransfer(RETARGET_SPI_UART, 0xFF);
	}
	spi_cs(cs, 0);
	return(0);
}
