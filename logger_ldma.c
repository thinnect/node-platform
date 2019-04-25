/**
 * Debug logger with LDMA on SiLabs EFR32 with CMSIS.
 *
 * The logger starts an LDMA transfer when data arrives.
 * Data during a transfer is appended to the buffer.
 * Logger is protected with mutexes.
 *
 * Transfer state is checked with a timeout timer, if it has finished and more
 * data is available, the transfer is restarted. This is a suboptimal solution,
 * a correct approach would start a new transfer from the LDMA done interrupt or
 * a deferred interrupt handler.
 *
 * @author Konstantin Bilozor, Raido Pahtma
 * @license MIT
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "em_device.h"
#include "em_ldma.h"

#include "cmsis_os2.h"

#include "logger_ldma.h"

static uint8_t buf[LOGGER_LDMA_BUFFER_LENGTH];

static bool ldmaDone = false;
osTimerId_t ldmaTimer;

static uint16_t bufStart = 0;
static uint16_t bufEnd = 0;
static uint16_t bufPos = 0;

static osMutexId_t log_mutex;


#ifdef LOGGER_LDMA_UART0
static const LDMA_TransferCfg_t periTransferTx = LDMA_TRANSFER_CFG_PERIPHERAL(ldmaPeripheralSignal_LEUART0_TXBL);
#endif//LOGGER_LDMA_UART0
#ifdef LOGGER_LDMA_UART1
static const LDMA_TransferCfg_t periTransferTx = LDMA_TRANSFER_CFG_PERIPHERAL(ldmaPeripheralSignal_USART1_TXBL);
#endif//LOGGER_LDMA_UART1
#ifdef LOGGER_LDMA_UART2
static const LDMA_TransferCfg_t periTransferTx = LDMA_TRANSFER_CFG_PERIPHERAL(ldmaPeripheralSignal_USART2_TXBL);
#endif//LOGGER_LDMA_UART2


static void ldmaStart(void) {
	uint16_t length = 0;

	bufStart = bufPos;
	if (bufStart >= LOGGER_LDMA_BUFFER_LENGTH) {
		bufStart = 0;
	}
	if (bufEnd >= LOGGER_LDMA_BUFFER_LENGTH) {
		bufEnd = 0;
	}

	if (bufEnd < bufStart) {
		bufPos = 0;
		length = LOGGER_LDMA_BUFFER_LENGTH-bufStart;
	} else if (bufEnd > bufStart) {
		bufPos = bufEnd;
		length = bufPos-bufStart;
	}

	if (length > 0) {
		ldmaDone = false;
		osTimerStart(ldmaTimer, 1 + length/10); // TODO timer based on baudrate - currently assumes 10 bytes per millisecond

		#ifdef LOGGER_LDMA_UART0
		LDMA_Descriptor_t xfer = LDMA_DESCRIPTOR_SINGLE_M2P_BYTE(&buf[bufStart], &LEUART0->TXDATA, length);
		#endif//LOGGER_LDMA_UART0
		#ifdef LOGGER_LDMA_UART1
		LDMA_Descriptor_t xfer = LDMA_DESCRIPTOR_SINGLE_M2P_BYTE(&buf[bufStart], &USART1->TXDATA, length);
		#endif//LOGGER_LDMA_UART1
		#ifdef LOGGER_LDMA_UART2
		LDMA_Descriptor_t xfer = LDMA_DESCRIPTOR_SINGLE_M2P_BYTE(&buf[bufStart], &USART2->TXDATA, length);
		#endif//LOGGER_LDMA_UART2

		//LDMA_Descriptor_t xfer = LDMA_DESCRIPTOR_LINKREL_M2P_BYTE(&buf[bufStart], &USART1->TXDATA, bufPos-bufStart,1);

		xfer.xfer.dstInc  = ldmaCtrlDstIncNone;
		xfer.xfer.doneIfs = 0;
		LDMA_StartTransfer(0, (void*)&periTransferTx, (void*)&xfer);
	}
}


void LDMA_IRQHandler(void) {
	uint32_t pending = LDMA_IntGet();

	while (pending & LDMA_IF_ERROR) {
		while (1); // panic
	}

	LDMA_IntClear(pending);

	uint32_t mask = 0x1;
	if (pending & mask) {
		//LDMA->IFC = mask;
		ldmaDone = true;
	}
}


static void ldma_timer_callback(void* argument) {
	while (osMutexAcquire(log_mutex, 1000) != osOK);
	if (ldmaDone) {
		ldmaStart();
	} else {
		osTimerStart(ldmaTimer, 10);
	}
	if (osMutexRelease(log_mutex) != osOK) {
		while (1);  // panic
	}
}


int logger_ldma_init() {
	log_mutex = osMutexNew(NULL);
	ldmaDone = true;
	ldmaTimer = osTimerNew(&ldma_timer_callback, osTimerOnce, NULL, NULL);

	LDMA_Init_t initLdma = LDMA_INIT_DEFAULT;
	LDMA_Init(&initLdma);

	return 0;
}


int logger_ldma(const char *ptr, int len) {
	uint16_t space;

	while (osMutexAcquire(log_mutex, 1000) != osOK);

	space = bufStart - bufEnd;

	if (bufStart <= bufEnd) {
		space += LOGGER_LDMA_BUFFER_LENGTH;
		if (len > space) {
			len = space;
		}
		if ((len + bufEnd) > LOGGER_LDMA_BUFFER_LENGTH) {
			memcpy(&buf[bufEnd], ptr, LOGGER_LDMA_BUFFER_LENGTH-bufEnd);
			memcpy(&buf[0], ptr + LOGGER_LDMA_BUFFER_LENGTH-bufEnd, len - (LOGGER_LDMA_BUFFER_LENGTH - bufEnd));
		} else {
			memcpy(&buf[bufEnd], ptr, len);
		}
	} else {
		if (len > space) {
			len = space;
		}
		memcpy(&buf[bufEnd], ptr, len);
	}

	bufEnd = bufEnd + len;
	if (bufEnd > LOGGER_LDMA_BUFFER_LENGTH) {
		bufEnd = bufEnd - LOGGER_LDMA_BUFFER_LENGTH;
	}

	if (ldmaDone) {
		ldmaStart();
	}

	if (osMutexRelease(log_mutex) != osOK) {
		while (1);  // panic
	}

	return len;
}
