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
 * Copyright Thinnect Inc. 2019
 * @author Konstantin Bilozor, Raido Pahtma
 * @license MIT
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "em_device.h"
#include "em_ldma.h"
#include "em_gpio.h"
#include "em_cmu.h"
#include "em_usart.h"

#include "sleep.h"

#include "retargetserial.h"
#include "retargetserialconfig.h"

#include "cmsis_os2.h"

#include "logger_ldma.h"

#define LOGGER_LDMA_CHANNEL 0
#define LOGGER_LDMA_DONE_THREAD_FLAG 0x00000001U
#define STR_LOGGER_FULL_MESSAGE "\nfull\n"

static uint8_t m_ldma_buf[LOGGER_LDMA_BUFFER_LENGTH];

static uint16_t m_buf_start = 0;
static uint16_t m_buf_end = 0;
static uint16_t m_buf_pos = 0;
static bool m_buf_full = false;

static osThreadId_t m_ldma_thread;
static osMutexId_t m_log_mutex;
static bool m_ldma_idle;

#ifdef LOGGER_LDMA_LEUART0
static const LDMA_TransferCfg_t periTransferTx = LDMA_TRANSFER_CFG_PERIPHERAL(ldmaPeripheralSignal_LEUART0_TXBL);
#endif//LOGGER_LDMA_LEUART0
#ifdef LOGGER_LDMA_USART0
static const LDMA_TransferCfg_t periTransferTx = LDMA_TRANSFER_CFG_PERIPHERAL(ldmaPeripheralSignal_USART0_TXBL);
#endif//LOGGER_LDMA_USART0
#ifdef LOGGER_LDMA_USART1
static const LDMA_TransferCfg_t periTransferTx = LDMA_TRANSFER_CFG_PERIPHERAL(ldmaPeripheralSignal_USART1_TXBL);
#endif//LOGGER_LDMA_USART1
#ifdef LOGGER_LDMA_USART2
static const LDMA_TransferCfg_t periTransferTx = LDMA_TRANSFER_CFG_PERIPHERAL(ldmaPeripheralSignal_USART2_TXBL);
#endif//LOGGER_LDMA_USART2


static void unsafe_try_ldma_start (void)
{
	uint16_t length = 0;

	if (m_buf_end < m_buf_start)
	{
		length = LOGGER_LDMA_BUFFER_LENGTH-m_buf_start;
	}
	else if (m_buf_end > m_buf_start)
	{
		length = m_buf_end-m_buf_start;
	}
	if(length > 512)
	{
		length = 512;
	}
	m_buf_pos = m_buf_start+length;
	if (m_buf_pos >= LOGGER_LDMA_BUFFER_LENGTH)
	{
		m_buf_pos = 0;
	}

	if (length > 0)
	{
		if (m_ldma_idle)
		{
			m_ldma_idle = false;

		 	// USART works in EM0 and EM1
			SLEEP_SleepBlockBegin(sleepEM2);

			// Enable debug serial and wait for 1ms
			RETARGET_SerialInit();
			osDelay(1);
		}

		// Configure LDMA transfer
		#ifdef LOGGER_LDMA_LEUART0
		LDMA_Descriptor_t xfer = LDMA_DESCRIPTOR_SINGLE_M2P_BYTE(&m_ldma_buf[m_buf_start], &LEUART0->TXDATA, length);
		#endif//LOGGER_LDMA_LEUART0
		#ifdef LOGGER_LDMA_USART0
		LDMA_Descriptor_t xfer = LDMA_DESCRIPTOR_SINGLE_M2P_BYTE(&m_ldma_buf[m_buf_start], &USART0->TXDATA, length);
		#endif//LOGGER_LDMA_USART0
		#ifdef LOGGER_LDMA_USART1
		LDMA_Descriptor_t xfer = LDMA_DESCRIPTOR_SINGLE_M2P_BYTE(&m_ldma_buf[m_buf_start], &USART1->TXDATA, length);
		#endif//LOGGER_LDMA_USART1
		#ifdef LOGGER_LDMA_USART2
		LDMA_Descriptor_t xfer = LDMA_DESCRIPTOR_SINGLE_M2P_BYTE(&m_ldma_buf[m_buf_start], &USART2->TXDATA, length);
		#endif//LOGGER_LDMA_USART2
		xfer.xfer.dstInc  = ldmaCtrlDstIncNone;
		xfer.xfer.doneIfs = 0;
		LDMA_StartTransfer(0, (void*)&periTransferTx, (void*)&xfer);
	}
	else
	{
		while (!(RETARGET_UART->STATUS & USART_STATUS_TXC))
		{
			osDelay(1); // Wait for transfer to complete
		}

		if (false == m_ldma_idle)
		{
			m_ldma_idle = true;
			SLEEP_SleepBlockEnd(sleepEM2);

			// Disable debug serial
			USART_Enable(RETARGET_UART, usartDisable);
			CMU_ClockEnable(RETARGET_CLK, false);
			GPIO_PinModeSet(RETARGET_TXPORT, RETARGET_TXPIN, gpioModeDisabled, 0);
			GPIO_PinModeSet(RETARGET_RXPORT, RETARGET_RXPIN, gpioModeDisabled, 0);
		}
	}
}


void LDMA_IRQHandler (void)
{
	uint32_t pending = LDMA_IntGet();

	while (pending & LDMA_IF_ERROR)
	{
		while (1); // panic
	}

	LDMA_IntClear(pending);

	if (pending & (1<<LOGGER_LDMA_CHANNEL))
	{
		osThreadFlagsSet(m_ldma_thread, LOGGER_LDMA_DONE_THREAD_FLAG);
	}
}


static void ldma_thread (void* argument)
{

    for(;;)
    {
    	osThreadFlagsWait(LOGGER_LDMA_DONE_THREAD_FLAG, osFlagsWaitAny, osWaitForever);

		while (osMutexAcquire(m_log_mutex, osWaitForever) != osOK);

		m_buf_full = false;
		m_buf_start = m_buf_pos;
		if ((m_buf_start >= LOGGER_LDMA_BUFFER_LENGTH)||(m_buf_end >= LOGGER_LDMA_BUFFER_LENGTH))
		{
			while(1); // panic
		}

		unsafe_try_ldma_start();

		osMutexRelease(m_log_mutex);
	}
}


int logger_ldma_init ()
{
	const osThreadAttr_t ldma_thread_attr = { .name = "ldma" };
	m_ldma_idle = true;
	m_log_mutex = osMutexNew(NULL);
	m_ldma_thread = osThreadNew(ldma_thread, NULL, &ldma_thread_attr);

	LDMA_Init_t initLdma = LDMA_INIT_DEFAULT;
	initLdma.ldmaInitIrqPriority = LDMA_INTERRUPT_PRIORITY;
	LDMA_Init(&initLdma);
	return 0;
}


static void logger_append_data (const char* ptr, int len)
{
	if ((m_buf_end + len) > LOGGER_LDMA_BUFFER_LENGTH)
	{
		uint16_t pol = LOGGER_LDMA_BUFFER_LENGTH - m_buf_end;
		memcpy(&m_ldma_buf[m_buf_end], ptr, pol);
		memcpy(&m_ldma_buf[0], ptr + pol, len - pol);
	}
	else
	{
		memcpy(&m_ldma_buf[m_buf_end], ptr, len);
	}
	m_buf_end += len;
	if (m_buf_end >= LOGGER_LDMA_BUFFER_LENGTH)
	{
		m_buf_end -= LOGGER_LDMA_BUFFER_LENGTH;
	}
}


int logger_ldma (const char *ptr, int len)
{
	uint16_t space;

	while (osMutexAcquire(m_log_mutex, osWaitForever) != osOK);

	if (m_buf_full == false)
	{
		space = m_buf_start - m_buf_end;

		if (m_buf_start <= m_buf_end)
		{
			space += LOGGER_LDMA_BUFFER_LENGTH;
		}
		if (len >= space - strlen(STR_LOGGER_FULL_MESSAGE))
		{
			len = space - strlen(STR_LOGGER_FULL_MESSAGE);
			m_buf_full = true;
		}
		logger_append_data(ptr, len);

		if (m_buf_full)
		{
			logger_append_data(STR_LOGGER_FULL_MESSAGE, strlen(STR_LOGGER_FULL_MESSAGE));
		}

		if (m_ldma_idle)
		{
			unsafe_try_ldma_start();
		}
	}

	osMutexRelease(m_log_mutex);

	return len;
}
