/*
 * TinyOS Serial packet protocol implementation on top of serial_hdlc.
 *
 * The serial protocol covers acknowledged packets, their acks and
 * packets that do not expect an acknowledgement.
 *
 * Copyright Thinnect Inc. 2019
 * @license MIT
 * @author Raido Pahtma, Konstantin Bilozor
 */

#include "serial_protocol.h"
#include "serial_protocol_packets.h"

#include "cmsis_os2.h"

#include <string.h>

#include "loglevels.h"
#define __MODUUL__ "serp"
#define __LOG_LEVEL__ (LOG_LEVEL_serial_protocol & BASE_LOG_LEVEL)
#include "log.h"

static void serial_protocol_sent_cb(void* argument);
static void serial_protocol_send_cb(void* argument);
static void serial_protocol_timeout(void* argument)
{
    osThreadFlagsSet((osThreadId_t*)argument, 2); // Sent
}

static void sp_thread_loop(void * argument)
{
    serial_protocol_t * sp = (serial_protocol_t*)argument;
    for(;;)
    {
        uint32_t flags = osThreadFlagsWait(3, osFlagsWaitAny, osWaitForever);
        if(flags & 1)
        {
            serial_protocol_send_cb(argument);
        }
        if(flags & 2)
        {
            serial_protocol_sent_cb(argument);
        }
    }
}

bool serial_protocol_init (serial_protocol_t * sp,
                           raw_serial_send_f * sendf,
                           serial_receive_f * dflt_rcvr)
{
    sp->tx_seq_num = 0;
    sp->rx_seq_num = 0;

    sp->sendf = sendf;

    sp->p_active_dispatcher = NULL;
    sp->p_dispatchers = NULL;
    sp->f_default_receiver = dflt_rcvr;

    const osThreadAttr_t thread_attr = { .name = "sp" };

    sp->mutex = osMutexNew(NULL);
    sp->thread = osThreadNew(sp_thread_loop, sp, &thread_attr);
    sp->timeout_timer = osTimerNew(&serial_protocol_timeout, osTimerOnce, sp->thread, NULL);

    return true;
}

bool serial_protocol_add_dispatcher(serial_protocol_t * sp,
                                    uint8_t dispatch,
                                    serial_dispatcher_t * dispatcher,
                                    serial_receive_f * rcvr,
                                    serial_send_done_f * sdf,
                                    void * user)
{
    while(osOK != osMutexAcquire(sp->mutex, osWaitForever));

    dispatcher->dispatch = dispatch;
    dispatcher->data = NULL;
    dispatcher->data_length = 0;
    dispatcher->ack = false;
    dispatcher->freceiver = rcvr;
    dispatcher->fsenddone = sdf;
    dispatcher->user = user;
    dispatcher->protocol = sp;

    dispatcher->next = NULL;

    serial_dispatcher_t** indirect = &(sp->p_dispatchers);
    while ((*indirect) != NULL)
    {
        indirect = &((*indirect)->next);
    }
    *indirect = dispatcher;

    osMutexRelease(sp->mutex);
    return true;
}

bool serial_protocol_remove_dispatcher(serial_protocol_t * sp,
                                       serial_dispatcher_t * dispatcher)
{
    while(osOK != osMutexAcquire(sp->mutex, osWaitForever));

    if (dispatcher == sp->p_active_dispatcher)
    {
        osMutexRelease(sp->mutex);
        return false; // the dispatcher is currently busy
    }
    else
    {
        serial_dispatcher_t** indirect = &(sp->p_dispatchers);
        while ((*indirect) != dispatcher)
        {
            if (NULL == (*indirect)->next)
            {
                osMutexRelease(sp->mutex);
                return false; // Reached the end and did not find the dispatcher
            }
            indirect = &((*indirect)->next);
        }
        *indirect = dispatcher->next;
    }

    osMutexRelease(sp->mutex);
    return true;
}

static bool serial_protocol_deliver(serial_protocol_t * sp, uint8_t dispatch,
                                    const uint8_t payload[], uint8_t length)
{
    serial_dispatcher_t* dp = sp->p_dispatchers;
    while (dp != NULL)
    {
        if (dp->dispatch == dispatch)
        {
            return dp->freceiver(dispatch, payload, length, dp->user);
        }
        dp = dp->next;
    }

    if (NULL != sp->f_default_receiver)
    {
        return sp->f_default_receiver(dispatch, payload, length, NULL);
    }
    else
    {
        debug1("no dp %02x", (unsigned int)dispatch);
    }

    return false;
}

static void serial_protocol_send_cb(void* argument)
{
    serial_protocol_t * sp = (serial_protocol_t *)argument;

    printk("sendcb\n");

    osStatus_t r;
    while(osOK != (r = osMutexAcquire(sp->mutex, 10000))) {
        printk("w %p %d\n", sp->mutex, (int)r);
        osDelay(10000);
    }

    printk("aqd\n");

    if(NULL != sp->p_active_dispatcher)
    {
        debug1("bsy");
        osMutexRelease(sp->mutex);
        return;
    }

    serial_dispatcher_t* dp = sp->p_dispatchers; // TODO make scheduling fair for dispatchers
    while (NULL != dp)
    {
        if (NULL != dp->data)
        {
            break;
        }
        dp = dp->next;
    }

    if ((NULL != dp) && (NULL != dp->data))
    {
        sp->p_active_dispatcher = dp;

        if (dp->ack)
        {
            uint8_t length = dp->data_length + sizeof(serial_protocol_ackpacket_t);
            if(length <= sizeof(sp->send_buffer))
            {
                serial_protocol_ackpacket_t* packet = (serial_protocol_ackpacket_t*)(sp->send_buffer);
                packet->protocol = SERIAL_PROTOCOL_ACKPACKET;
                packet->dispatch = dp->dispatch;
                packet->seq_num = ++(sp->tx_seq_num);
                printk("mc\n");
                memcpy(packet->payload, dp->data, dp->data_length);

                printk("cpd\n");
                dp->send_time = osKernelGetTickCount();
                dp->acked = false;
                printk("sendf\n");
                sp->sendf(sp->send_buffer, length);
                printk("sendfd\n");
            }
            else
            {
                err1("s"); // Packet dropped
            }

            osTimerStart(sp->timeout_timer, osKernelGetTickFreq()*SERIAL_PROTOCOL_ACK_TIMEOUT_MS/1000);
        }
        else
        {
            uint8_t length = dp->data_length + sizeof(serial_protocol_packet_t);
            if(length <= sizeof(sp->send_buffer))
            {
                serial_protocol_packet_t* packet = (serial_protocol_packet_t*)(sp->send_buffer);
                packet->protocol = SERIAL_PROTOCOL_PACKET;
                packet->dispatch = dp->dispatch;
                printk("mc\n");
                memcpy(packet->payload, dp->data, dp->data_length);
                printk("mcd\n");

                dp->send_time = osKernelGetTickCount();
                dp->acked = false;
                printk("sendf\n");
                sp->sendf(sp->send_buffer, length);
                printk("sendfd\n");
            }
            else
            {
                err1("s"); // Packet dropped
            }

            osThreadFlagsSet(sp->thread, 2);
        }
    }
    else
    {
        debug1("idle");
    }

    osMutexRelease(sp->mutex);
}

static void serial_protocol_sent_cb(void* argument)
{
    serial_protocol_t * sp = (serial_protocol_t *)argument;

    printk("sentf\n");

    while(osOK != osMutexAcquire(sp->mutex, osWaitForever));

    if (NULL != sp->p_active_dispatcher)
    {
        serial_dispatcher_t * dsp = sp->p_active_dispatcher;
        if (dsp->ack)
        {
            if (false == dsp->acked)
            {
                uint32_t passed = osKernelGetTickCount() - dsp->send_time;
                if (1000*passed/osKernelGetTickFreq() >= SERIAL_PROTOCOL_ACK_TIMEOUT_MS)
                {
                    // sent, but not acked
                }
                else
                {
                    uint32_t remaining = (osKernelGetTickFreq()*(SERIAL_PROTOCOL_ACK_TIMEOUT_MS-passed)/1000);
                    osTimerStart(sp->timeout_timer, remaining+1UL);
                    debug1("wait %"PRIu32, remaining);
                    osMutexRelease(sp->mutex);
                    return;
                }
            }
            else
            {
                // sent and acked
            }
        }
        else
        {
            // sent
        }

        // Copy arguments and release dispatcher
        const uint8_t * data = dsp->data;
        uint8_t length = dsp->data_length;
        bool acked = dsp->acked;

        // Free the dispatcher
        sp->p_active_dispatcher->data = NULL;
        sp->p_active_dispatcher->ack = false; // Ignore any late acks

        // Do the callback without holding the mutex
        osMutexRelease(sp->mutex);

        // dispatch, user and the callback are immutable as long as the dispatcher is registered
        dsp->fsenddone(dsp->dispatch, data, length, acked, dsp->user);

        // Unlock the dispatcher
        while(osOK != osMutexAcquire(sp->mutex, osWaitForever));
        sp->p_active_dispatcher = NULL;
        osMutexRelease(sp->mutex);

        // Check for pending messages
        osThreadFlagsSet(sp->thread, 1);
    }
    else
    {
        osMutexRelease(sp->mutex);
    }
}

void serial_protocol_receive_generic (void * sp, const uint8_t data[], uint8_t length)
{
    serial_protocol_receive((serial_protocol_t *)sp, data, length);
}

void serial_protocol_receive (serial_protocol_t * sp, const uint8_t data[], uint8_t length)
{
    while(osOK != osMutexAcquire(sp->mutex, osWaitForever));

    if (length > 0)
    {
        uint8_t protocol = data[0];
        switch (protocol)
        {
            case SERIAL_PROTOCOL_ACK: // TODO not actually used
                if (sizeof(serial_protocol_ack_t) <= length)
                {
                    serial_protocol_ack_t * ack = (serial_protocol_ack_t*)data;
                    if ((NULL != sp->p_active_dispatcher)&&(sp->p_active_dispatcher->ack))
                    {
                        if(ack->seq_num == sp->tx_seq_num)
                        {
                            debug1("ack %02X", (unsigned int)ack->seq_num);
                            sp->p_active_dispatcher->acked = true;
                            osThreadFlagsSet(sp->thread, 2);
                        }
                        else
                        {
                            warn1("tx ack %02X != %02X",
                                  (unsigned int)ack->seq_num,
                                  (unsigned int)sp->tx_seq_num);
                        }
                    }
                    else
                    {
                        warn1("ack? %02X", (unsigned int)ack->seq_num);
                    }
                }
            break;

            case SERIAL_PROTOCOL_ACKPACKET:
                if (sizeof(serial_protocol_ackpacket_t) <= length)
                {
                    bool ack = false;
                    bool duplicate = false;
                    serial_protocol_ackpacket_t* packet = (serial_protocol_ackpacket_t*)data;

                    if (packet->seq_num == sp->rx_seq_num)
                    {
                        // NOTE: default sf implementation is broken and always
                        //       sends seqnum 0.
                        //       Additionally seqnums should include a TTL
                        #ifndef SERIAL_PROTOCOL_IGNORE_SEQNUM
                            duplicate = true;
                        #else
                            #warning "Duplicate packets will not be dropped!"
                        #endif//SERIAL_PROTOCOL_IGNORE_SEQNUM
                    }

                    if (duplicate)
                    {
                        warn1("dup %02X", (unsigned int)(packet->seq_num));
                        ack = true;
                    }
                    else
                    {
                        uint8_t len = length - sizeof(serial_protocol_ackpacket_t);
                        ack = serial_protocol_deliver(sp, packet->dispatch,
                                                      packet->payload, len);
                    }

                    if (ack)
                    {
                        sp->rx_seq_num = packet->seq_num;

                        serial_protocol_ack_t ackp = {SERIAL_PROTOCOL_ACK, packet->seq_num};
                        sp->sendf((uint8_t*)&ackp, sizeof(serial_protocol_ack_t));
                    }
                    else
                    {
                        warn1("drop %02X", (unsigned int)(packet->seq_num));
                    }
                }
            break;

            case SERIAL_PROTOCOL_PACKET:
                if (sizeof(serial_protocol_packet_t) <= length)
                {
                    serial_protocol_packet_t * packet = (serial_protocol_packet_t*)data;
                    uint8_t len = length - sizeof(serial_protocol_packet_t);
                    serial_protocol_deliver(sp, packet->dispatch,
                                            packet->payload, len);
                }
            break;
        }
    }

    osMutexRelease(sp->mutex);
}

bool serial_protocol_send (serial_dispatcher_t* dispatcher, const uint8_t data[], uint8_t length, bool ack)
{
    serial_protocol_t * sp = dispatcher->protocol;
    bool ret = true;

    while(osOK != osMutexAcquire(sp->mutex, osWaitForever));

    if (NULL != dispatcher->data)
    {
        ret = false;
    }
    else
    {
        dispatcher->data = data;
        dispatcher->data_length = length;
        dispatcher->ack = ack;
        osThreadFlagsSet(sp->thread, 1); // Defer
    }

    osMutexRelease(sp->mutex);

    return ret;
}
