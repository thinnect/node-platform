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

#include "loggers_ext.h"
#if defined(LOGGER_LDMA)
#include "logger_ldma.h"
#elif defined(LOGGER_FWRITE)
#include "logger_fwrite.h"
#endif//LOGGER_*

void basic_noos_logger_setup ()
{
    USART_Reset(RETARGET_UART);
    RETARGET_SerialInit();
    log_init(BASE_LOG_LEVEL, &logger_fwrite_basic, NULL);
}

void basic_rtos_logger_setup ()
{
    osDelay(10); // And do a small delay to clear buffers
    #if defined(LOGGER_LDMA)
        logger_ldma_init();
        #if defined(LOGGER_TIMESTAMP)
            log_init(BASE_LOG_LEVEL, &logger_ldma, &osCounterGetMilli);
        #else
            log_init(BASE_LOG_LEVEL, &logger_ldma, NULL);
        #endif
    #elif defined(LOGGER_FWRITE)
        logger_fwrite_init();
        #if defined(LOGGER_TIMESTAMP)
            log_init(BASE_LOG_LEVEL, &logger_fwrite, &osCounterGetMilli);
        #else
            log_init(BASE_LOG_LEVEL, &logger_fwrite, NULL);
        #endif
    #else
        #warning "No LOGGER_* has been defined!"
    #endif//LOGGER_*
}
