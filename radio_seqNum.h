/**
* Radio message sequence number checking and storage.
*
* Copyright Thinnect.
* @license <PROPRIETARY>
* @author Konstantin Bilozor
*/
#pragma once

#include <stdint.h>

#ifndef SEQNUM_TABLE_SIZE
#define SEQNUM_TABLE_SIZE 50
#endif

#ifndef SEQNUM_TIMEOUT_SEC
#define SEQNUM_TIMEOUT_SEC 15
#endif

typedef struct seqSrcTs {
    uint8_t seqNum;
    uint16_t src;
    uint16_t timestamp;
} seqSrcTs_t;


/**
 * @brief   Save source address, sequence number and timestamp for received
 * 		    packet. If same sequence number is received within timeout, then
 *		    it is discarded.
 * @params  src      Received packet source address.
 * @params  seqNum   Received packet sequnce number.
 * @params  currTime Timestamp for received packet in seconds.
 * @returns True if received sequence number was saved, false if discarded.
 */
bool radio_seqNum_save(uint16_t src, uint8_t seqNum, uint16_t currTime);

