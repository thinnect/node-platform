/**
 * Data types and manipulation functions for UUID's.
 * @author Raido Pahtma
 * @license MIT
*/
#ifndef UNIVERSALLYUNIQUEIDENTIFIER_H_
#define UNIVERSALLYUNIQUEIDENTIFIER_H_

#include <string.h>

#ifndef NESC
#include "nesc_to_c_compat.h"
#endif

typedef struct uuid {
	uint32_t time_low;
	uint16_t time_mid;
	uint16_t time_hi_and_version;
	uint8_t  clock_seq_hi_and_reserved;
	uint8_t  clock_seq_low;
	uint8_t  node[6];
} uuid_t;

#pragma pack(push, 1)
typedef nx_struct nx_uuid {
	nx_uint32_t time_low;
	nx_uint16_t time_mid;
	nx_uint16_t time_hi_and_version;
	nx_uint8_t  clock_seq_hi_and_reserved;
	nx_uint8_t  clock_seq_low;
	nx_uint8_t  node[6];
} nx_uuid_t;
#pragma pack(pop)

nx_uuid_t* hton_uuid(nx_uuid_t* net_uuid, uuid_t* host_uuid);
uuid_t* ntoh_uuid(uuid_t* host_uuid, nx_uuid_t* net_uuid);

#endif // UNIVERSALLYUNIQUEIDENTIFIER_H_
