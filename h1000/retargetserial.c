/**
 * UART retargetting.
 *
 * Copyright Thinnect Inc. 2021
 * @license MIT
 */
#include "retargetserial.h"

#include "uart.h"

int RETARGET_SerialInit (void)
{
    uart_Cfg_t cfg = {
#ifndef TEST_SYSTEM
          .tx_pin = P9,
          .rx_pin = P10,
#else
          .tx_pin = P14,
          .rx_pin = P15,
#endif
        .rts_pin = GPIO_DUMMY,
        .cts_pin = GPIO_DUMMY,
        .baudrate = 1000000,
        .use_fifo = TRUE,
        .hw_fwctrl = FALSE,
        .use_tx_buf = FALSE,
        .parity     = FALSE,
        .evt_handler = NULL,
    };
    return hal_uart_init(cfg, UART0);//uart init
}
