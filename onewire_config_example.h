#pragma once

#define ONEWIRE_ONEWIRE_PERIPHERAL_CLOCK 38400000LU

#define ONEWIRE_UART        USART1
#define ONEWIRE_UART_CLOCK  cmuClock_USART1
#define ONEWIRE_PORT        gpioPortB
#define ONEWIRE_PIN         0

// NB: There are some differences for setting up the pin routing depending
// on the EFX32 series being used.

// Series 0
#define ONEWIRE_LOCATION    BSP_SERIAL_APP_ROUTE_LOC
// Series 1
#define ONEWIRE_TX_LOCATION _USART_ROUTELOC0_TXLOC_LOC15
#define ONEWIRE_RX_LOCATION _USART_ROUTELOC0_RXLOC_LOC15
// Series 2
#define ONEWIRE_UART_INDEX  1
