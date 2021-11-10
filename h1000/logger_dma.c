/**
 * DMA Logging for H1002.
 *
 * Copyright Thinnect Inc. 2020
 * @license MIT
*/
#include "logger_dma.h"

#include "dma.h"
#include "error.h"

#include "cmsis_os2.h"

#include <string.h>

extern void dbg_printf (const char *format, ...); // The phyplussdk debug function

// FIXME: UART_0_BASE should be included ... and the UART selectable
#define UART_0_BASE 0x40004000

#ifndef LOGGER_DMA_CHANNEL
#define LOGGER_DMA_CHANNEL DMA_CH_0
#endif//LOGGER_DMA_CHANNEL

#ifndef LOGGER_DMA_BUF_SIZE
#define LOGGER_DMA_BUF_SIZE 1048
#endif//LOGGER_DMA_BUF_SIZE

#ifndef LOGGER_LDMA_MAX_TRANSFER
#define LOGGER_LDMA_MAX_TRANSFER 512
#endif//LOGGER_LDMA_MAX_TRANSFER

#define LOGGER_THREAD_FLAG_LDMA_DONE 0x00000001U
#define LOGGER_THREAD_FLAG_NEW_DATA  0x00000010U
#define LOGGER_THREAD_FLAG_LDMA_FAIL 0x00000100U
#define LOGGER_THREAD_FLAGS          0x00000111U

#define STR_LOGGER_FULL_MESSAGE "\nfull\n"

static DMA_CH_CFG_t m_cfg;

#ifdef DUMP_LOG_BUF
uint8_t m_log_dma_buf[LOGGER_DMA_BUF_SIZE];
#else
static uint8_t m_log_dma_buf[LOGGER_DMA_BUF_SIZE];
#endif

static uint16_t m_buf_start = 0;
static uint16_t m_buf_end = 0;
static uint16_t m_buf_pos = 0;
static bool m_buf_full = false;
static osThreadId_t m_id = 0;
static osMutexId_t m_log_mutex;
static osTimerId_t m_fbtimer;


static void dma_callback (DMA_CH_t chn)
{
    osThreadFlagsSet(m_id, LOGGER_THREAD_FLAG_LDMA_DONE);
}


static void fallback_timer_cb (void * arg)
{
    hal_dma_stop_channel(LOGGER_DMA_CHANNEL);
    dbg_printf("E|logdma:%d|TO %d/%d/%d\n", __LINE__, (int)m_buf_start, (int)m_buf_end, (int)m_buf_pos);
    osThreadFlagsSet(m_id, LOGGER_THREAD_FLAG_LDMA_FAIL);
}


static bool configure_dma (void)
{
    int retval = hal_dma_init(); // FIXME: DMA initialization should be elsewhere, others may use it also
    if (PPlus_SUCCESS != retval)
    {
        dbg_printf("E|logdma:%d|init %d\n", __LINE__, retval);
        return false;
    }

    if (DMA_GET_MAX_TRANSPORT_SIZE(LOGGER_DMA_CHANNEL) < LOGGER_LDMA_MAX_TRANSFER)
    {
        dbg_printf("E|logdma:%d|trs %d < \n", __LINE__, (int)DMA_GET_MAX_TRANSPORT_SIZE(LOGGER_DMA_CHANNEL), (int)LOGGER_LDMA_MAX_TRANSFER);
        return false;
    }

    HAL_DMA_t ch_cfg;
    ch_cfg.dma_channel = LOGGER_DMA_CHANNEL;
    ch_cfg.evt_handler = &dma_callback;

    retval = hal_dma_init_channel(ch_cfg);
    if (PPlus_SUCCESS != retval)
    {
        dbg_printf("E|logdma:%d|init ch %d\n", __LINE__, retval);
        return false;
    }

    retval = hal_dma_stop_channel(LOGGER_DMA_CHANNEL);
    if (PPlus_SUCCESS != retval)
    {
        dbg_printf("E|logdma:%d|STOP %d\n", __LINE__, retval);
        return false;
    }

    return true;
}


static void reset_dma (void)
{
    int retval = hal_dma_stop_channel(LOGGER_DMA_CHANNEL);
    if (PPlus_SUCCESS != retval)
    {
        dbg_printf("E|logdma:%d|STOP %d\n", __LINE__, retval);

        retval = hal_dma_deinit();
        if (PPlus_SUCCESS != retval)
        {
            dbg_printf("E|logdma:%d|deinit %d\n", __LINE__, retval);
        }

        configure_dma();
    }
}


static bool try_dma_start ()
{
    uint16_t length = 0;

    if (m_buf_end < m_buf_start)
    {
        length = LOGGER_DMA_BUF_SIZE - m_buf_start;
    }
    else if (m_buf_end > m_buf_start)
    {
        length = m_buf_end - m_buf_start;
    }
    else if (m_buf_full)
    {
        length = LOGGER_DMA_BUF_SIZE;
    }

    if (length > LOGGER_LDMA_MAX_TRANSFER)
    {
        length = LOGGER_LDMA_MAX_TRANSFER;
    }

    m_buf_pos = m_buf_start + length;
    if (m_buf_pos >= LOGGER_DMA_BUF_SIZE)
    {
        m_buf_pos = 0;
    }

    if (length > 0)
    {
        m_cfg.transf_size = length;

        m_cfg.sinc = DMA_INC_INC;
        m_cfg.src_tr_width = DMA_WIDTH_BYTE;
        m_cfg.src_msize = DMA_BSIZE_1;
        m_cfg.src_addr = (uint32_t)&m_log_dma_buf[m_buf_start];

        m_cfg.dinc = DMA_INC_NCHG;
        m_cfg.dst_tr_width = DMA_WIDTH_BYTE;
        m_cfg.dst_msize = DMA_BSIZE_1;
        m_cfg.dst_addr = (uint32_t)UART_0_BASE;

        m_cfg.enable_int = true;

        int retval = hal_dma_config_channel(LOGGER_DMA_CHANNEL, &m_cfg);
        if (PPlus_SUCCESS == retval)
        {
            retval = hal_dma_start_channel(LOGGER_DMA_CHANNEL);
            if (PPlus_SUCCESS == retval)
            {
                if (m_cfg.enable_int == false)
                {
                    hal_dma_wait_channel_complete(LOGGER_DMA_CHANNEL);
                }
                osTimerStart(m_fbtimer, 10000);
                return true;
            }
            else
            {
                dbg_printf("E|logdma:%d|START %d\n", __LINE__, retval);
                reset_dma();
            }
        }
        else
        {
            dbg_printf("E|logdma:%d|CFG %d\n", __LINE__, retval);
            reset_dma();
        }
    }

    return false;
}


static void log_dma_thread (void * argument)
{
    bool busy = false;
    for (;;)
    {
        uint32_t flags = osThreadFlagsWait(LOGGER_THREAD_FLAGS, osFlagsWaitAny, osWaitForever);

        if (osFlagsErrorTimeout == flags)
        {
            flags = 0;
        }

        while (osOK != osMutexAcquire(m_log_mutex, osWaitForever));

        if (flags & LOGGER_THREAD_FLAG_LDMA_DONE)
        {
            osTimerStop(m_fbtimer);
            busy = false;
            m_buf_full = false;
            m_buf_start = m_buf_pos;
        }

        if (flags & LOGGER_THREAD_FLAG_LDMA_FAIL)
        {
            dbg_printf("E|logdma:%d|TOT\n", __LINE__);
            busy = false;
            m_buf_full = false;
            m_buf_start = m_buf_pos;
        }

        if ((m_buf_start >= LOGGER_DMA_BUF_SIZE)||(m_buf_end >= LOGGER_DMA_BUF_SIZE))
        {
            //sys_panic("ldma buf");
            dbg_printf("E|logdma:%d|BAD %d/%d\n", __LINE__, (int)m_buf_start, (int)m_buf_end);
            while(1);
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
        memcpy(&m_log_dma_buf[m_buf_end], ptr, pol);
        memcpy(&m_log_dma_buf[0], ptr + pol, len - pol);
    }
    else
    {
        memcpy(&m_log_dma_buf[m_buf_end], ptr, len);
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


bool logger_dma_init (void)
{
    bool ret = configure_dma();

    if (ret)
    {
        const osMutexAttr_t mattr = { .attr_bits = osMutexPrioInherit };
        m_log_mutex = osMutexNew(&mattr);
        m_fbtimer = osTimerNew(fallback_timer_cb, osTimerOnce, NULL, NULL);

        const osThreadAttr_t log_dma_thread_attr = { .name = "log", .stack_size = 512, .priority = osPriorityLow };
        m_id = osThreadNew(log_dma_thread, NULL, &log_dma_thread_attr);
    }

    return ret;
}
