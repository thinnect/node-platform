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

#include "serial_hdlc.h"

#include "cmsis_os2.h"

#include <string.h>

#include "loglevels.h"
#define __MODUUL__ "serp"
#define __LOG_LEVEL__ (LOG_LEVEL_serial_protocol & BASE_LOG_LEVEL)
#include "log.h"

static uint8_t m_tx_seq_num;
static uint8_t m_rx_seq_num;

static osMutexId_t m_sp_mutex;
static osTimerId_t m_timeout_timer;
static osTimerId_t m_send_timer;

static serial_dispatcher_t * mp_active_dispatcher;
static serial_dispatcher_t * mp_dispatchers;
static serial_receive_f * mp_default_receiver;

static void serial_protocol_timeout_cb(void* argument);
static void serial_protocol_send_cb(void* argument);

void serial_protocol_init (serial_receive_f * dflt_rcvr)
{
    m_tx_seq_num = 0;
    m_rx_seq_num = 0;

    mp_active_dispatcher = NULL;
    mp_dispatchers = NULL;
    mp_default_receiver = dflt_rcvr;

    m_sp_mutex = osMutexNew(NULL);
    m_timeout_timer = osTimerNew(&serial_protocol_timeout_cb, osTimerOnce, NULL, NULL);
    m_send_timer = osTimerNew(&serial_protocol_send_cb, osTimerOnce, NULL, NULL);
}

bool serial_protocol_add_dispatcher(uint8_t dispatch,
                                    serial_dispatcher_t * dispatcher,
                                    serial_receive_f * rcvr,
                                    serial_send_done_f * sdf)
{
    dispatcher->dispatch = dispatch;
    dispatcher->data = NULL;
    dispatcher->data_length = 0;
    dispatcher->ack = false;
    dispatcher->freceiver = rcvr;
    dispatcher->fsenddone = sdf;

    dispatcher->next = NULL;

    serial_dispatcher_t** indirect = &mp_dispatchers;
    while ((*indirect) != NULL)
    {
        indirect = &((*indirect)->next);
    }
    *indirect = dispatcher;

    return true;
}

bool serial_protocol_remove_dispatcher(serial_dispatcher_t * dispatcher)
{
    serial_dispatcher_t** indirect = &mp_dispatchers;
    while ((*indirect) != dispatcher)
    {
        if (NULL == (*indirect)->next)
        {
            return false;
        }
        indirect = &((*indirect)->next);
    }
    *indirect = dispatcher->next;
    return true;
}

static bool serial_protocol_deliver(uint8_t dispatch, const uint8_t payload[], uint8_t length)
{
    serial_dispatcher_t* dp = mp_dispatchers;
    while (dp != NULL)
    {
        if (dp->dispatch == dispatch)
        {
            return dp->freceiver(dispatch, payload, length);
        }
        dp = dp->next;
    }

    if (NULL != mp_default_receiver)
    {
        return mp_default_receiver(dispatch, payload, length);
    }
    else
    {
        debug1("no dp %02x", (unsigned int)dispatch);
    }

    return false;
}

static void serial_protocol_send_done(bool acked)
{
    osTimerStop(m_timeout_timer);

    mp_active_dispatcher->fsenddone(mp_active_dispatcher->dispatch,
                                    mp_active_dispatcher->data,
                                    mp_active_dispatcher->data_length,
                                    acked);

    mp_active_dispatcher->data = NULL;
    mp_active_dispatcher = NULL;

    osTimerStart(m_send_timer, 1UL);
}

// Allocate memory for send, don't use Timer thread stack, which might be small
static uint8_t m_send_buffer[255];

static void serial_protocol_send_cb(void* argument)
{
    osMutexAcquire(m_sp_mutex, osWaitForever);

    serial_dispatcher_t* dp = mp_dispatchers; // TODO make scheduling fair for dispatchers
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
        if (dp->ack)
        {
            uint8_t length = dp->data_length + sizeof(serial_protocol_ackpacket_t);
            if(length <= sizeof(m_send_buffer))
            {
                serial_protocol_ackpacket_t* packet = (serial_protocol_ackpacket_t*)m_send_buffer;
                packet->protocol = SERIAL_PROTOCOL_ACKPACKET;
                packet->dispatch = dp->dispatch;
                packet->seq_num = ++m_tx_seq_num;
                memcpy(packet->payload, dp->data, dp->data_length);

                serial_hdlc_send(m_send_buffer, length);
            }
            else
            {
                err1("s"); // Packet silently dropped
            }

            osTimerStart(m_timeout_timer, 100UL);
        }
        else
        {
            uint8_t length = dp->data_length + sizeof(serial_protocol_packet_t);
            if(length <= sizeof(m_send_buffer))
            {

                serial_protocol_packet_t* packet = (serial_protocol_packet_t*)m_send_buffer;
                packet->protocol = SERIAL_PROTOCOL_PACKET;
                packet->dispatch = dp->dispatch;
                memcpy(packet->payload, dp->data, dp->data_length);

                serial_hdlc_send(m_send_buffer, length);
            }
            else
            {
                err1("s"); // Packet silently dropped
            }

            osTimerStart(m_timeout_timer, 1UL); // Minimal possible delay
        }

        mp_active_dispatcher = dp;
    }
    else
    {
        debug1("idle");
    }

    osMutexRelease(m_sp_mutex);
}

static void serial_protocol_timeout_cb(void* argument)
{
    osMutexAcquire(m_sp_mutex, osWaitForever);

    if (NULL != mp_active_dispatcher)
    {
        serial_protocol_send_done(false);
    }

    osMutexRelease(m_sp_mutex);
}

void serial_protocol_receive (const uint8_t data[], uint8_t length)
{
    osMutexAcquire(m_sp_mutex, osWaitForever);

    if (length > 0)
    {
        uint8_t protocol = data[0];
        switch (protocol)
        {
            case SERIAL_PROTOCOL_ACK: // TODO not actually used
                if (sizeof(serial_protocol_ack_t) <= length)
                {
                    serial_protocol_ack_t * ack = (serial_protocol_ack_t*)data;
                    if ((NULL != mp_active_dispatcher)&&(mp_active_dispatcher->ack))
                    {
                        if(ack->seq_num == m_tx_seq_num)
                        {
                            debug1("ack %02X", (unsigned int)ack->seq_num);
                            serial_protocol_send_done(true);
                        }
                        else
                        {
                            warn1("tx ack %02X != %02X",
                                  (unsigned int)ack->seq_num,
                                  (unsigned int)m_tx_seq_num);
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

                    if (packet->seq_num == m_rx_seq_num)
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
                        ack = serial_protocol_deliver(packet->dispatch,
                                                      packet->payload, len);
                    }

                    if (ack)
                    {
                        m_rx_seq_num = packet->seq_num;

                        serial_protocol_ack_t ackp = {SERIAL_PROTOCOL_ACK, packet->seq_num};
                        serial_hdlc_send((uint8_t*)&ackp, sizeof(serial_protocol_ack_t));
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
                    serial_protocol_deliver(packet->dispatch,
                                            packet->payload, len);
                }
            break;
        }
    }

    osMutexRelease(m_sp_mutex);
}

bool serial_protocol_send (serial_dispatcher_t* dispatcher, const uint8_t data[], uint8_t length, bool ack)
{
    bool ret = true;

    osMutexAcquire(m_sp_mutex, osWaitForever);

    if (NULL != dispatcher->data)
    {
        ret = false;
    }
    else
    {
        dispatcher->data = data;
        dispatcher->data_length = length;
        dispatcher->ack = ack;

        if (NULL == mp_active_dispatcher)
        {
            osTimerStart(m_send_timer, 1UL); // Defer, 0 not possible
        }
    }

    osMutexRelease(m_sp_mutex);

    return ret;
}
