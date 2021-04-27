#include "logger_dma.h"

#include "dma.h"
#include "error.h"
#include "loggers_ext.h"
#include "cmsis_os2.h"
#include "string.h"



#include "loglevels.h"
#define __MODUUL__ "dma"
#define __LOG_LEVEL__ ( LOG_LEVEL_main & 0xFFFF )
#include "log.h"

#define UART_0_BASE 0x40004000
#define LOGGER_DMA_BUF_SIZE 512
#define LOGGER_LDMA_MAX_TRANSFER 512
#define LOGGER_THREAD_FLAG_LDMA_DONE 0x00000001U
#define LOGGER_THREAD_FLAG_NEW_DATA  0x00000010U
#define LOGGER_THREAD_FLAGS          0x00000011U
#define STR_LOGGER_FULL_MESSAGE "\nfull\n"


static DMA_CH_CFG_t m_cfg;

static uint8_t m_ldma_buf[LOGGER_DMA_BUF_SIZE];


static uint16_t m_buf_start = 0;
static uint16_t m_buf_end = 0;
static uint16_t m_buf_pos = 0;
static bool m_buf_full = false;
static osThreadId_t m_id = 0; 
static osMutexId_t m_log_mutex;
static bool m_ldma_idle;
static bool m_uart_active;

void dma_callback(DMA_CH_t chn)
{
		osThreadFlagsSet(m_id, LOGGER_THREAD_FLAG_LDMA_DONE);
}


static bool try_dma_start()
{	
	uint16_t length = 0;

	if (m_buf_end < m_buf_start)
	{
		length = LOGGER_DMA_BUF_SIZE-m_buf_start;
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
	if (m_buf_pos >= LOGGER_DMA_BUF_SIZE)
	{
		m_buf_pos = 0;
	}
	
	if(length > 0)
	{
		
	  m_cfg.transf_size = length;
				
		m_cfg.sinc = DMA_INC_INC;
		m_cfg.src_tr_width = DMA_WIDTH_BYTE;
		m_cfg.src_msize = DMA_BSIZE_1;
		m_cfg.src_addr = (uint32_t)&m_ldma_buf[m_buf_start];
			
		m_cfg.dinc = DMA_INC_NCHG;
		m_cfg.dst_tr_width = DMA_WIDTH_BYTE;
		m_cfg.dst_msize = DMA_BSIZE_1;
		m_cfg.dst_addr = (uint32_t)UART_0_BASE;
			
		m_cfg.enable_int = true;
		int retval = hal_dma_config_channel(DMA_CH_0,&m_cfg);
    if(retval == PPlus_SUCCESS)
    {
        hal_dma_start_channel(DMA_CH_0);
        if(m_cfg.enable_int == false)
        {
            hal_dma_wait_channel_complete(DMA_CH_0);
        }
				return true;				
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

		if ((m_buf_start >= LOGGER_DMA_BUF_SIZE)||(m_buf_end >= LOGGER_DMA_BUF_SIZE))
		{
			 //sys_panic("ldma buf");
		}

		if (false == busy)
		{
			busy = try_dma_start();
		}

		osMutexRelease(m_log_mutex);
	}
}

static void logger_append_data (const char* ptr, int len)
{
	if ((m_buf_end + len) > LOGGER_DMA_BUF_SIZE)
	{
		uint16_t pol = LOGGER_DMA_BUF_SIZE - m_buf_end;
		memcpy(&m_ldma_buf[m_buf_end], ptr, pol);
		memcpy(&m_ldma_buf[0], ptr + pol, len - pol);
	}
	else
	{
		memcpy(&m_ldma_buf[m_buf_end], ptr, len);
	}
	m_buf_end += len;
	if (m_buf_end >= LOGGER_DMA_BUF_SIZE)
	{
		m_buf_end -= LOGGER_DMA_BUF_SIZE;
	}
}


int logger_dma (const char *ptr, int len)
{
	uint16_t space;

	while (osOK != osMutexAcquire(m_log_mutex, osWaitForever));

	if (m_buf_full == false)
	{
		space = m_buf_start - m_buf_end;

		if (m_buf_start <= m_buf_end)
		{
			space += LOGGER_DMA_BUF_SIZE;
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

		osThreadFlagsSet(m_id, LOGGER_THREAD_FLAG_NEW_DATA);
	}

	osMutexRelease(m_log_mutex);

	return len;
}

bool logger_dma_init()
{
		bool ret = true;
		ret = hal_dma_init();
		const osThreadAttr_t ldma_thread_attr = { .name = "dma", .stack_size = 2048};
		m_log_mutex = osMutexNew(NULL);

	  HAL_DMA_t ch_cfg;
		ch_cfg.dma_channel = DMA_CH_0;
		ch_cfg.evt_handler = &dma_callback;
    ret &= hal_dma_init_channel(ch_cfg);
	
		ret &= hal_dma_stop_channel(DMA_CH_0);
		
		m_id = osThreadNew(ldma_thread, NULL, &ldma_thread_attr);
		
		return ret;
}
