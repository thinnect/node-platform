/**
 * Debug logger with LDMA on SiLabs EFR32 with CMSIS.
 *
 * The logger starts an LDMA transfer when data arrives.
 * Data during a transfer is appended to the buffer.
 * Logger is protected with mutexes.
 *
 * Logger can be configured (by defining LOGGER_LDMA_DMADRV) to go through
 * DMADRV if DMA needs to be shared. It will then request a channel from DMADRV
 * and keep it forever.
 *
 * Copyright Thinnect Inc. 2019
 * @author Konstantin Bilozor, Raido Pahtma
 * @license MIT
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "em_device.h"
#include "em_gpio.h"
#include "em_cmu.h"
#include "em_usart.h"

#ifdef LOGGER_LDMA_DMADRV
#include "dmadrv.h"
#else
#include "em_ldma.h"
#endif//LOGGER_LDMA_DMADRV

#include "sleep.h"

#include "retargetserial.h"
#include "retargetserialconfig.h"

#include "cmsis_os2.h"

#include "loglevels.h" // No logging from here, but panic uses the fixtures
#define __MODUUL__ "logldma"
#include "sys_panic.h"

#include "logger_ldma.h"

#define LOGGER_LDMA_CHANNEL 0
#define LOGGER_LDMA_MAX_TRANSFER 512
#define LOGGER_THREAD_FLAG_LDMA_DONE 0x00000001U
#define LOGGER_THREAD_FLAG_NEW_DATA  0x00000010U
#define LOGGER_THREAD_FLAGS          0x00000011U
#define STR_LOGGER_FULL_MESSAGE "\nfull\n"

static uint8_t m_ldma_buf[LOGGER_LDMA_BUFFER_LENGTH];

static uint16_t m_buf_start = 0;
static uint16_t m_buf_end = 0;
static uint16_t m_buf_pos = 0;
static bool m_buf_full = false;

static osThreadId_t m_ldma_thread;
static osMutexId_t m_log_mutex;
static bool m_ldma_idle;
static bool m_uart_active;

static unsigned int m_dma_channel = LOGGER_LDMA_CHANNEL;


#ifdef LOGGER_LDMA_DMADRV
static bool dmadrv_callback (unsigned int channel, unsigned int sequenceNo, void * data)
{
	osThreadFlagsSet(m_ldma_thread, LOGGER_THREAD_FLAG_LDMA_DONE);
	return false;
}
#else
void LDMA_IRQHandler (void)
{
	uint32_t pending = LDMA_IntGet();

	while (pending & LDMA_IF_ERROR)
	{
		sys_panic("ldma if");
	}

	LDMA_IntClear(pending);

	if (pending & (1<<LOGGER_LDMA_CHANNEL))
	{
		osThreadFlagsSet(m_ldma_thread, LOGGER_THREAD_FLAG_LDMA_DONE);
	}
}
#endif//LOGGER_LDMA_DMADRV


static bool try_ldma_start (void)
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
	if(length > LOGGER_LDMA_MAX_TRANSFER)
	{
		length = LOGGER_LDMA_MAX_TRANSFER;
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

			if (false == m_uart_active)
			{
				// Enable debug serial and wait for 1ms
				RETARGET_SerialInit();
				osDelay(1);
				m_uart_active = true;
			}
		}

		// Configure LDMA transfer
		#ifdef LOGGER_LDMA_DMADRV
			#ifdef LOGGER_LDMA_LEUART0
			DMADRV_PeripheralSignal_t dmasignal = dmadrvPeripheralSignal_LEUART0_TXBL;
			void * dst = (void*)&(LEUART0->TXDATA);
			#endif//LOGGER_LDMA_LEUART0
			#ifdef LOGGER_LDMA_USART0
			DMADRV_PeripheralSignal_t dmasignal = dmadrvPeripheralSignal_USART0_TXBL;
			void * dst = (void*)&(USART0->TXDATA);
			#endif//LOGGER_LDMA_USART0
			#ifdef LOGGER_LDMA_USART1
			DMADRV_PeripheralSignal_t dmasignal = dmadrvPeripheralSignal_USART1_TXBL;
			void * dst = (void*)&(USART1->TXDATA);
			#endif//LOGGER_LDMA_USART1
			#ifdef LOGGER_LDMA_USART2
			DMADRV_PeripheralSignal_t dmasignal = dmadrvPeripheralSignal_USART2_TXBL;
			void * dst = (void*)&(USART2->TXDATA);
			#endif//LOGGER_LDMA_USART2

			Ecode_t err = DMADRV_MemoryPeripheral(
			                  m_dma_channel,            // unsigned int channelId
			                  dmasignal,                // DMADRV_PeripheralSignal_t
			                  dst,                      // void * dst
			                  &m_ldma_buf[m_buf_start], // void * src
			                  true,                     // bool srcInc
			                  length,                   // int len
			                  dmadrvDataSize1,          // DMADRV_DataSize_t
			                  dmadrv_callback,          // DMADRV_Callback_t
			                  NULL                      // void * cbUserParam
			              );
			if (ECODE_OK == err)
			{
				return true;
			}
		#else//Use LDMA directly
			#ifdef LOGGER_LDMA_LEUART0
			static const LDMA_TransferCfg_t periTransferTx = LDMA_TRANSFER_CFG_PERIPHERAL(ldmaPeripheralSignal_LEUART0_TXBL);
			LDMA_Descriptor_t xfer = LDMA_DESCRIPTOR_SINGLE_M2P_BYTE(&m_ldma_buf[m_buf_start], &LEUART0->TXDATA, length);
			#endif//LOGGER_LDMA_LEUART0
			#ifdef LOGGER_LDMA_USART0
			static const LDMA_TransferCfg_t periTransferTx = LDMA_TRANSFER_CFG_PERIPHERAL(ldmaPeripheralSignal_USART0_TXBL);
			LDMA_Descriptor_t xfer = LDMA_DESCRIPTOR_SINGLE_M2P_BYTE(&m_ldma_buf[m_buf_start], &USART0->TXDATA, length);
			#endif//LOGGER_LDMA_USART0
			#ifdef LOGGER_LDMA_USART1
			static const LDMA_TransferCfg_t periTransferTx = LDMA_TRANSFER_CFG_PERIPHERAL(ldmaPeripheralSignal_USART1_TXBL);
			LDMA_Descriptor_t xfer = LDMA_DESCRIPTOR_SINGLE_M2P_BYTE(&m_ldma_buf[m_buf_start], &USART1->TXDATA, length);
			#endif//LOGGER_LDMA_USART1
			#ifdef LOGGER_LDMA_USART2
			static const LDMA_TransferCfg_t periTransferTx = LDMA_TRANSFER_CFG_PERIPHERAL(ldmaPeripheralSignal_USART2_TXBL);
			LDMA_Descriptor_t xfer = LDMA_DESCRIPTOR_SINGLE_M2P_BYTE(&m_ldma_buf[m_buf_start], &USART2->TXDATA, length);
			#endif//LOGGER_LDMA_USART2

			xfer.xfer.dstInc  = ldmaCtrlDstIncNone;
			xfer.xfer.doneIfs = 0;
			LDMA_StartTransfer(m_dma_channel, (void*)&periTransferTx, (void*)&xfer);

			return true;
		#endif//LOGGER_LDMA_DMADRV
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

			// Disable debug serial if low-power operation is not disabled
			#ifndef LOGGER_LDMA_DISABLE_SLEEP
				USART_Enable(RETARGET_UART, usartDisable);
				CMU_ClockEnable(RETARGET_CLK, false);
				GPIO_PinModeSet(RETARGET_TXPORT, RETARGET_TXPIN, gpioModeDisabled, 0);
				GPIO_PinModeSet(RETARGET_RXPORT, RETARGET_RXPIN, gpioModeDisabled, 0);
				m_uart_active = false;
			#endif
		}
	}

	return false;
}


static void ldma_thread (void* argument)
{
	bool busy = false;
    for(;;)
    {
    	uint32_t flags = osThreadFlagsWait(LOGGER_THREAD_FLAGS, osFlagsWaitAny, osWaitForever);

		while (osOK != osMutexAcquire(m_log_mutex, osWaitForever));

		if (flags & LOGGER_THREAD_FLAG_LDMA_DONE)
		{
			busy = false;
			m_buf_full = false;
			m_buf_start = m_buf_pos;
		}

		if ((m_buf_start >= LOGGER_LDMA_BUFFER_LENGTH)||(m_buf_end >= LOGGER_LDMA_BUFFER_LENGTH))
		{
			sys_panic("ldma buf");
		}

		if (false == busy)
		{
			busy = try_ldma_start();
		}

		osMutexRelease(m_log_mutex);
	}
}


int logger_ldma_init ()
{
	const osThreadAttr_t ldma_thread_attr = { .name = "ldma", .stack_size = 340 };
	m_ldma_idle = true;
	m_uart_active = false;
	m_log_mutex = osMutexNew(NULL);
	m_ldma_thread = osThreadNew(ldma_thread, NULL, &ldma_thread_attr);

	#ifdef LOGGER_LDMA_DMADRV
		Ecode_t result = DMADRV_AllocateChannel(&m_dma_channel, NULL);
		if (ECODE_OK != result)
		{
			return 1; // should perhaps panic instead
		}
	#else
		LDMA_Init_t initLdma = LDMA_INIT_DEFAULT;
		initLdma.ldmaInitIrqPriority = LDMA_INTERRUPT_PRIORITY;
		LDMA_Init(&initLdma);
	#endif

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

	while (osOK != osMutexAcquire(m_log_mutex, osWaitForever));

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

		osThreadFlagsSet(m_ldma_thread, LOGGER_THREAD_FLAG_NEW_DATA);
	}

	osMutexRelease(m_log_mutex);

	return len;
}
