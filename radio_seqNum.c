/**
* Radio message sequence number checking and storage.
*
* Copyright Thinnect.
* @license <PROPRIETARY>
* @author Konstantin Bilozor
*/
#include <stdint.h>
#include <stdbool.h>
#include <inttypes.h>
#include "radio_seqNum.h"

#include "loglevels.h"
#define __MODUUL__ "radio_seqNum"
#define __LOG_LEVEL__ (LOG_LEVEL_radio_seqNum & BASE_LOG_LEVEL)
#include "log.h"

seqSrcTs_t srcAddrTable[SEQNUM_TABLE_SIZE];


bool radio_seqNum_save(uint16_t src, uint8_t seqNum, uint16_t currTime) {
    
    uint8_t srcPos = 0;

    for (srcPos=0; srcPos<SEQNUM_TABLE_SIZE; srcPos++) {
        if ((srcAddrTable[srcPos].src == src) && (srcAddrTable[srcPos].timestamp > (currTime-SEQNUM_TIMEOUT_SEC))) {
            if (srcAddrTable[srcPos].seqNum == seqNum) {
                srcAddrTable[srcPos].timestamp = currTime;
                return false;
            }
            break;
        } else if (srcAddrTable[srcPos].timestamp < (currTime-SEQNUM_TIMEOUT_SEC)) {
            break;
        } else if (srcAddrTable[srcPos].src == 0) {
            break;
        }
    }

    if (srcPos == SEQNUM_TABLE_SIZE) {
        uint8_t oldestPos = 0;
        uint16_t oldestTimestamp = srcAddrTable[0].timestamp;
        warn1("seqNums table is full!");
        info1("src: %04"PRIX16", seqNum: %02"PRIX8", ts: %"PRIu16, srcAddrTable[0].src, srcAddrTable[0].seqNum, srcAddrTable[0].timestamp);
        for (uint8_t i=1; i<SEQNUM_TABLE_SIZE; i++) {
            info1("src: %04"PRIX16", seqNum: %02"PRIX8", ts: %"PRIu16, srcAddrTable[i].src, srcAddrTable[i].seqNum, srcAddrTable[i].timestamp);
            if (srcAddrTable[i].timestamp < oldestTimestamp) {
                oldestPos = i;
                oldestTimestamp = srcAddrTable[i].timestamp;
            }
        }
        srcPos = oldestPos;
        info1("oldest seqNum: %"PRIu8, srcPos);
    }

    srcAddrTable[srcPos].seqNum = seqNum;
    srcAddrTable[srcPos].src = src;
    srcAddrTable[srcPos].timestamp = currTime;

    return true;
}
