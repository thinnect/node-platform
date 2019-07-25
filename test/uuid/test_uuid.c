/**
 * Basic UUID tests.
 *
 * Copyright Thinnect 2019.
 * @license MIT
 */
#include <string.h>
#include <inttypes.h>
#include <stdio.h>

#include "UniversallyUniqueIdentifier.h"


int main() {
	printf("tests start\n");

	// 392c7861-cad3-40ce-b59c-7ff93edffc34
	uuid_t uuid1;
	uuid1.time_low = 0x392c7861;
	uuid1.time_mid = 0xcad3;
	uuid1.time_hi_and_version = 0x40ce;
	uuid1.clock_seq_hi_and_reserved = 0xb5;
	uuid1.clock_seq_low = 0x9c;
	uuid1.node[0] = 0x7f;
	uuid1.node[1] = 0xf9;
	uuid1.node[2] = 0x3e;
	uuid1.node[3] = 0xdf;
	uuid1.node[4] = 0xfc;
	uuid1.node[5] = 0x34;

	nx_uuid_t nxuuid;
	hton_uuid(&nxuuid, &uuid1);

	if(memcmp(&nxuuid, "\x39\x2c\x78\x61\xca\xd3\x40\xce\xb5\x9c\x7f\xf9\x3e\xdf\xfc\x34", 16) != 0) {
		printf("ERROR: nx uuid does not match!\n");
		return 1;
	}

	uuid_t uuid2;
	ntoh_uuid(&uuid2, &nxuuid);

	if((uuid1.time_low != uuid2.time_low)
	 ||(uuid1.time_mid != uuid2.time_mid)
	 ||(uuid1.time_hi_and_version != uuid2.time_hi_and_version)
	 ||(uuid1.clock_seq_hi_and_reserved != uuid2.clock_seq_hi_and_reserved)
	 ||(uuid1.clock_seq_low != uuid2.clock_seq_low)
	 ||(uuid1.node[0] != uuid2.node[0])
	 ||(uuid1.node[1] != uuid2.node[1])
	 ||(uuid1.node[2] != uuid2.node[2])
	 ||(uuid1.node[3] != uuid2.node[3])
	 ||(uuid1.node[4] != uuid2.node[4])
	 ||(uuid1.node[5] != uuid2.node[5])) {
		printf("ERROR: uuid2 does not match uuid1!\n");
		return 1;
	}

	printf("SUCCESS?\n");
	return 0;
}
