/**
 * Receive license file and write it to USER DATA area.
 *
 * Copyright Thinnect Inc. 2020
 * @license MIT
 */
#ifndef LICENSE_RCVR_H_
#define LICENSE_RCVR_H_

#include "mist_comm.h"
#include "mist_comm_iface.h"

#define AMID_LICENSE_RCVR 0x1A
#define EUI_START_POS 17

#define EUI64_REQUEST 0x01
#define EUI64_RESPONSE 0x02
#define LICFILE_RESPONSE 0x03
#define LICFILE_DONE 0x04
#define LICFILE_EXISTS 0x05

typedef struct lic_rcvr
{
	comms_layer_t * comms;
	comms_receiver_t rcvr;
	comms_msg_t msg;

	am_addr_t respond;
	am_addr_t lic_done;
	am_addr_t exists;

	uint8_t data[104];

	osMutexId_t mutex;
	osThreadId_t thread;
} lic_rcvr_t;

#pragma pack(push, 1)
typedef struct am__packet
{
	uint8_t header;
} am_license_packet_t;

typedef struct am__packet_file
{
	uint8_t header;
	uint8_t data[104];
} am_license_file_t;

typedef struct am__packet_eui
{
	uint8_t header;
	uint8_t eui[8];
} am_license_eui_packet_t;
#pragma pack(pop)

/**
 * Initialize receiver for license file.
 *
 * @param comms Comms layer for license receiver.
 * @param lic   Memory for the license module.
 */
void license_rcvr_init (comms_layer_t * comms, lic_rcvr_t * lic);

#endif//LICENSE_RCVR_H_
