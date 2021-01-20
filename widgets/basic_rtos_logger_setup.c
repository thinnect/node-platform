/**
 * A supposedly common and basic logger setup procedure.
 *
 * !!! Look at the code, make sure your use-case falls under the same
 *     understanding of basic setup. !!!
 *
 * Copyright Thinnect Inc. 2020
 * @license MIT
 */

#include "cmsis_os2_ext.h"

#include "retargetserial.h"
#include "em_usart.h"
#include "logger_fwrite_basic.h"

#include "platform_mutex.h"

#include "loggers_ext.h"
#if defined(LOGGER_LDMA)
#include "logger_ldma.h"
#elif defined(LOGGER_FWRITE)
#include "logger_fwrite.h"
#endif//LOGGER_*

static platform_mutex_t m_log_mutex;

void basic_noos_logger_setup ()
{
    USART_Reset(RETARGET_UART);
    RETARGET_SerialInit();
    log_init(BASE_LOG_LEVEL, &logger_fwrite_basic, NULL, NULL);
}

void basic_rtos_logger_setup ()
{
    osDelay(1); // Do a small delay to clear buffers

    m_log_mutex = platform_mutex_new("log");

    #if defined(LOGGER_LDMA)
        logger_ldma_init();
        #if defined(LOGGER_TIMESTAMP)
            log_init(BASE_LOG_LEVEL, &logger_ldma, &osCounterGetMilli, m_log_mutex);
        #else
            log_init(BASE_LOG_LEVEL, &logger_ldma, NULL, m_log_mutex);
        #endif
    #elif defined(LOGGER_FWRITE)
        logger_fwrite_init();
        #if defined(LOGGER_TIMESTAMP)
            log_init(BASE_LOG_LEVEL, &logger_fwrite, &osCounterGetMilli, m_log_mutex);
        #else
            log_init(BASE_LOG_LEVEL, &logger_fwrite, NULL, m_log_mutex);
        #endif
    #else
        #warning "No LOGGER_* has been defined!"
    #endif//LOGGER_*
}
