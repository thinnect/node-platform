/*
 * TinyOS Serial packet protocol implementation on top of serial_hdlc.
 *
 * The serial protocol covers acknowledged packets and their acks and
 * packets that do not expect an acknowledgement.
 *
 * Copyright Thinnect Inc. 2019
 * @license MIT
 * @author Raido Pahtma, Konstantin Bilozor
 */

#include "serial_protocol.h"

#include "serial_hdlc.h"

#include <string.h>

#define SERIAL_PROTOCOL_PACKET    0x45
#define SERIAL_PROTOCOL_ACKPACKET 0x44
#define SERIAL_PROTOCOL_ACK       0x43

#pragma pack(push, 1)
typedef struct serial_protocol_ack
{
    uint8_t protocol; // SERIAL_PROTOCOL_ACK - 0x43
    uint8_t seq_num;
    // uint8_t crc[2]; - handled by lower layer
} serial_protocol_ack_t;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct serial_protocol_ackpacket
{
    uint8_t protocol; // SERIAL_PROTOCOL_ACKPACKET - 0x44
    uint8_t seq_num;
    uint8_t payload[]; // First byte of the "payload" would be the dispatcher
    // uint8_t crc[2]; - handled by lower layer
} serial_protocol_ackpacket_t;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct serial_protocol_packet
{
    uint8_t protocol; // SERIAL_PROTOCOL_PACKET - 0x45
    uint8_t payload[]; // First byte of the "payload" would be the dispatcher
} serial_protocol_packet_t;
#pragma pack(pop)

#include "loglevels.h"
#define __MODUUL__ "serp"
#define __LOG_LEVEL__ (LOG_LEVEL_serial_protocol & BASE_LOG_LEVEL)
#include "log.h"

static bool m_tx_ack_wait;
static uint8_t m_tx_seq_num;
static uint8_t m_rx_seq_num;

static serial_receive_f * mp_receiver;

void serial_protocol_init (serial_receive_f* receiver)
{
    m_tx_ack_wait = false;
    m_tx_seq_num = 0;
    m_rx_seq_num = 0;
    mp_receiver = receiver;
}

void serial_protocol_receive (const uint8_t data[], uint8_t length)
{
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
                            m_tx_ack_wait = false;
                            debug1("ack %02X", (unsigned int)ack->seq_num);
                            // TODO signal send-done?
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
}

bool serial_protocol_send (const uint8_t data[], uint8_t length)
{
    uint8_t payload[length + sizeof(serial_protocol_packet_t)];
    serial_protocol_packet_t* packet = (serial_protocol_packet_t*)payload;
    packet->protocol = SERIAL_PROTOCOL_PACKET;
    memcpy(packet->payload, data, length);
    serial_hdlc_send(payload, sizeof(payload));
    return true;
}

int16_t serial_protocol_send_ackpacket (const uint8_t data[], uint8_t length)
{
    uint8_t payload[length + sizeof(serial_protocol_packet_t)];
    serial_protocol_ackpacket_t* packet = (serial_protocol_ackpacket_t*)payload;
    packet->protocol = SERIAL_PROTOCOL_ACKPACKET;
    packet->seq_num = ++m_tx_seq_num;
    memcpy(packet->payload, data, length);
    serial_hdlc_send(payload, sizeof(payload));
    return m_tx_seq_num;
}
