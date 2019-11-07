/*
 * SiLabs HDLC-UART for TinyOS Serial protocol.
 *
 * Uses a thread for callbacks, has an RX queue.
 *
 * Normal TX happens in the user thread, but acks may be sent using the
 * callback thread.
 *
 * Copyright Thinnect Inc. 2019
 * @license MIT
 * @author Raido Pahtma, Konstantin Bilozor
 */

#include "serial_hdlc.h"

#include "retargethdlcconfig.h"

#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include "em_cmu.h"
#include "em_gpio.h"
#include "em_usart.h"

#include "cmsis_os2.h" // OS functions in general

#include "hdlc_tools.h"
#include "checksum.h"

#include "loglevels.h"
#define __MODUUL__ "serh"
#define __LOG_LEVEL__ (LOG_LEVEL_serial_hdlc & BASE_LOG_LEVEL)
#include "log.h"

#define HDLC_RX_QUEUE_LENGTH 5

static osMessageQueueId_t m_rx_queue;
static osMessageQueueId_t m_free_queue; // osMemoryPool not supported on FreeRTOS
static hdlc_decoder_t m_decoders[HDLC_RX_QUEUE_LENGTH];

static osMutexId_t m_send_mutex;

static hdlc_decoder_t * p_decoder;
static serial_hdlc_receive_f * mf_receiver;
static void * mp_receiver;

static int mErrorsHdlc = 0;
static int mErrorsQueue = 0;
static int mQueued = 0;
static int mErrorsEmpty = 0;

static void serial_hdlc_thread ();

void serial_hdlc_init (serial_hdlc_receive_f * rcvf, void* rcvr)
{
    USART_InitAsync_TypeDef init = USART_INITASYNC_DEFAULT;
    init.enable = usartDisable;

    CMU_ClockEnable(cmuClock_GPIO, true);
    GPIO_PinModeSet(SERIAL_HDLC_TXPORT, SERIAL_HDLC_TXPIN, gpioModePushPull, 1);
    GPIO_PinModeSet(SERIAL_HDLC_RXPORT, SERIAL_HDLC_RXPIN, gpioModeInput, 0);

    CMU_ClockEnable(SERIAL_HDLC_CLOCK, true);
    USART_InitAsync(SERIAL_HDLC_UART, &init);

    #if defined(GPIO_USART_ROUTEEN_TXPEN) // Series 2
        GPIO->USARTROUTE[SERIAL_HDLC_UART_INDEX].RXROUTE =
            (SERIAL_HDLC_RXPORT << _GPIO_USART_RXROUTE_PORT_SHIFT)
           |(SERIAL_HDLC_RXPIN << _GPIO_USART_RXROUTE_PIN_SHIFT);
        GPIO->USARTROUTE[SERIAL_HDLC_UART_INDEX].TXROUTE =
           (SERIAL_HDLC_TXPORT << _GPIO_USART_TXROUTE_PORT_SHIFT)
          |(SERIAL_HDLC_TXPIN << _GPIO_USART_TXROUTE_PIN_SHIFT);
        // Enable USART interface pins
        GPIO->USARTROUTE[SERIAL_HDLC_UART_INDEX].ROUTEEN =
            (GPIO_USART_ROUTEEN_RXPEN)|(GPIO_USART_ROUTEEN_TXPEN);
    #elif defined(USART_ROUTEPEN_RXPEN)  // Series 1
        SERIAL_HDLC_UART->ROUTELOC0 = (SERIAL_HDLC_TXLOC)|(SERIAL_HDLC_RXLOC);
        // Enable USART interface pins
        SERIAL_HDLC_UART->ROUTEPEN = (USART_ROUTEPEN_RXPEN)|(USART_ROUTEPEN_TXPEN);
    #else
        #error Series 0 not supported!
    #endif

    m_send_mutex = osMutexNew(NULL);

    // Set up RX queue
    m_rx_queue = osMessageQueueNew(HDLC_RX_QUEUE_LENGTH, sizeof(hdlc_decoder_t*), NULL);
    if (m_rx_queue == NULL)
    {
        err1("rxq");
        return;
    }

    // Simulate a pool with an array and a queue
    m_free_queue = osMessageQueueNew(HDLC_RX_QUEUE_LENGTH, sizeof(hdlc_decoder_t*), NULL);
    if (m_free_queue == NULL)
    {
        err1("rxp");
        return;
    }
    for (uint8_t i=0;i<HDLC_RX_QUEUE_LENGTH;i++) {
        hdlc_decoder_t* decoder = &(m_decoders[i]);
        osMessageQueuePut(m_free_queue, &decoder, 0, 0);
    }

    // Provide initial decoder for interrupt
    if (osOK != osMessageQueueGet(m_free_queue, &p_decoder, NULL, 0))
    {
        err1("dec");
        return;
    }
    hdlc_decoder_init(p_decoder);

    mf_receiver = rcvf;
    mp_receiver = rcvr;
}

void serial_hdlc_enable ()
{
    const osThreadAttr_t thread_attr = { .name = "ser" };
    osThreadNew(serial_hdlc_thread, NULL, &thread_attr);

    // Clear any pending interrupts
    USART_IntClear(SERIAL_HDLC_UART, USART_IF_RXDATAV);
    NVIC_ClearPendingIRQ(SERIAL_HDLC_IRQn);

    // set priority and enable interupts
    NVIC_SetPriority(SERIAL_HDLC_IRQn, 3); // not shifted, but once shifted = 01100000 -> TODO not a magic number
    USART_IntEnable(SERIAL_HDLC_UART, USART_IF_RXDATAV);
    NVIC_EnableIRQ(SERIAL_HDLC_IRQn);

    // Enable UART
    USART_Enable(SERIAL_HDLC_UART, usartEnable);
}

void serial_hdlc_disable ()
{
    USART_Enable(SERIAL_HDLC_UART, usartDisable);
    NVIC_DisableIRQ(SERIAL_HDLC_IRQn);
    GPIO_PinModeSet(SERIAL_HDLC_TXPORT, SERIAL_HDLC_TXPIN, gpioModeDisabled, 0);
    GPIO_PinModeSet(SERIAL_HDLC_RXPORT, SERIAL_HDLC_RXPIN, gpioModeDisabled, 0);
}

int serial_hdlc_send (const uint8_t* out, uint8_t len)
{
    uint16_t crc = crc_xmodem(out, len);
    uint8_t checksum[2];
    checksum[0] = (uint8_t)crc;
    checksum[1] = (uint8_t)(crc >> 8);

    hdlc_encoder_t enc;
    hdlc_encoder_init(&enc, out, len, checksum, sizeof(checksum));

    while(osMutexAcquire(m_send_mutex, 1000) != osOK);

    uint8_t data;
    while (hdlc_encoder_next(&enc, &data))
    {
        debug1("snd %02X", (unsigned int)data);
        USART_Tx(SERIAL_HDLC_UART, data);
        while (!(SERIAL_HDLC_UART->STATUS & USART_STATUS_TXC));
    }

    osMutexRelease(m_send_mutex);
    return 0;
}

static void serial_hdlc_thread (void * arg)
{
    for (;;)
    {
        hdlc_decoder_t* decoder;
        if (osOK == osMessageQueueGet(m_rx_queue, &decoder, NULL, 0))
        {
            uint8_t plen = 0;
            debugb1("data", decoder->data, decoder->length);
            if (decoder->length > 2)
            {
                plen = decoder->length - 2;
                uint16_t pcrc = decoder->data[plen]|((uint16_t)(decoder->data[plen+1]) << 8);
                uint16_t ccrc = crc_xmodem(decoder->data, plen);
                if (pcrc == ccrc)
                {
                    debug1("crc ok");
                    if (NULL != mf_receiver)
                    {
                        mf_receiver(mp_receiver, decoder->data, plen);
                    }
                }
                else
                {
                    warn1("crc %"PRIx16"!=%"PRIx16, ccrc, pcrc);
                }
            }

            if (osOK != osMessageQueuePut(m_free_queue, &decoder, 0, 0))
            {
                err1("Fput");
                for (;;); // TODO panic
            };
        }

        if (mErrorsHdlc||mErrorsQueue||mErrorsEmpty)
        {
            debug1("r %d %d %d %d", mErrorsHdlc, mErrorsQueue, mErrorsEmpty, mQueued);
            while (1);
        }
    }
}

void SERIAL_HDLC_IRQ_NAME(void)
{
    if (USART_STATUS_RXDATAV & USART_StatusGet(SERIAL_HDLC_UART))
    {
        int outlen = hdlc_decoder_append(p_decoder, USART_RxDataGet(SERIAL_HDLC_UART));
        if (outlen < 0)
        {
            // TODO mark failure - hdlc overflow
            hdlc_decoder_init(p_decoder);
            mErrorsHdlc++;
        }
        else if (outlen > 0)
        {
            hdlc_decoder_t* deliver = p_decoder; // enqueue current decoder
            if (osOK == osMessageQueueGet(m_free_queue, &p_decoder, NULL, 0)) // allocate new decoder
            {
                if (osOK != osMessageQueuePut(m_rx_queue, &deliver, 0, 0))
                {
                    osMessageQueuePut(m_free_queue, &deliver, 0, 0); // Put it into pool instead
                    // TODO ERROR queue overflow
                    mErrorsQueue++;
                }
                else
                {
                    mQueued++;
                }
            }
            else // Must always have a decoder - drop packet
            {
                p_decoder = deliver; // Reuse undelivered decoder
                // TODO ERROR pool empty
                mErrorsEmpty++;
            }
            hdlc_decoder_init(p_decoder);
        }

        USART_IntClear(SERIAL_HDLC_UART, USART_IF_RXDATAV);
    }
}
