/**
 * SerialProtocol packet definitions.
 *
 * Copyright Thinnect Inc. 2019
 * @license MIT
 * @author Raido Pahtma
 */
#ifndef SERIAL_PROTOCOL_PACKETS_H_
#define SERIAL_PROTOCOL_PACKETS_H_

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
    uint8_t dispatch;
    uint8_t payload[]; // First byte of the "payload" would be the dispatcher
    // uint8_t crc[2]; - handled by lower layer
} serial_protocol_ackpacket_t;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct serial_protocol_packet
{
    uint8_t protocol; // SERIAL_PROTOCOL_PACKET - 0x45
    uint8_t dispatch;
    uint8_t payload[]; // First byte of the "payload" would be the dispatcher
} serial_protocol_packet_t;
#pragma pack(pop)

#endif//SERIAL_PROTOCOL_PACKETS_H_
