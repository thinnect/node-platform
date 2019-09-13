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

static bool m_tx_ack_wait;
static uint8_t m_tx_seq_num;
static uint8_t m_rx_seq_num;

// Data packet being sent, NULL if idle
static const uint8_t* m_data;
static uint8_t m_data_length;

static osMutexId_t m_sp_mutex;
static osTimerId_t m_ack_timeout_timer;

static serial_receive_f * mp_receiver;
static serial_send_done_f * mp_senddone;

static void serial_protocol_ack_timeout_cb(void* argument);

void serial_protocol_init (serial_receive_f* rcvr, serial_send_done_f * sdf)
{
    m_data = NULL;
    m_data_length = 0;

    m_tx_ack_wait = false;
    m_tx_seq_num = 0;
    m_rx_seq_num = 0;
    mp_receiver = rcvr;
    mp_senddone = sdf;

    m_sp_mutex = osMutexNew(NULL);
    m_ack_timeout_timer = osTimerNew(&serial_protocol_ack_timeout_cb, osTimerOnce, NULL, NULL);
}

static void serial_protocol_ack_timeout_cb(void* argument)
{
    osMutexAcquire(m_sp_mutex, osWaitForever);
    if (NULL != m_data)
    {
        mp_senddone(m_data, m_data_length, false);
        m_data = NULL;
        m_tx_ack_wait = false;
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
                    if (m_tx_ack_wait)
                    {
                        if(ack->seq_num == m_tx_seq_num)
                        {
                            debug1("ack %02X", (unsigned int)ack->seq_num);
                            osTimerStop(m_ack_timeout_timer);
                            mp_senddone(m_data, m_data_length, true);
                            m_data = NULL;
                            m_tx_ack_wait = false;
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
                        ack = mp_receiver(&(data[sizeof(serial_protocol_ackpacket_t)]),
                                          length - sizeof(serial_protocol_ackpacket_t));
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
                    mp_receiver(&(data[sizeof(serial_protocol_packet_t)]),
                        length - sizeof(serial_protocol_packet_t));
                }
            break;
        }
    }

    osMutexRelease(m_sp_mutex);
}

bool serial_protocol_send (const uint8_t data[], uint8_t length, bool ack)
{
    bool ret = true;

    osMutexAcquire(m_sp_mutex, osWaitForever);

    if (NULL != m_data)
    {
        ret = false;
    }
    else if (ack)
    {
        uint8_t payload[length + sizeof(serial_protocol_packet_t)];
        serial_protocol_ackpacket_t* packet = (serial_protocol_ackpacket_t*)payload;
        packet->protocol = SERIAL_PROTOCOL_ACKPACKET;
        packet->seq_num = ++m_tx_seq_num;
        memcpy(packet->payload, data, length);
        serial_hdlc_send(payload, sizeof(payload));

        m_data = data;
        m_data_length = length;
        m_tx_ack_wait = true;
        osTimerStart(m_ack_timeout_timer, 100UL);
    }
    else
    {
        uint8_t payload[length + sizeof(serial_protocol_packet_t)];
        serial_protocol_packet_t* packet = (serial_protocol_packet_t*)payload;
        packet->protocol = SERIAL_PROTOCOL_PACKET;
        memcpy(packet->payload, data, length);
        serial_hdlc_send(payload, sizeof(payload));

        m_data = data;
        m_data_length = length;
        m_tx_ack_wait = false;
        osTimerStart(m_ack_timeout_timer, 1UL); // Minimal possible delay
    }

    osMutexRelease(m_sp_mutex);

    return ret;
}
